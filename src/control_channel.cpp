// Copyright 2026 play_launch developers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "play_launch_container/control_channel.hpp"

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "play_launch_container/control_json.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

namespace play_launch_container
{

namespace
{

/// Must match `CONTROL_PROTOCOL_VERSION` in
/// `src/play_launch/src/ipc/container_protocol.rs`.
constexpr uint32_t kProtocolVersion = 2;

/// Same cap as the supervisor's: a frame larger than this means the stream
/// desynced, not that a load request grew.
constexpr size_t kMaxFrameBytes = 8u * 1024u * 1024u;

const char * kFdEnv = "PLAY_LAUNCH_CONTROL_FD";

/// TEST-ONLY fault injection, off unless `PLAY_LAUNCH_CONTROL_DROP` names a
/// fault. It exists because the two most consequential paths in the load
/// policy — "the container never saw this load" and "the container never
/// answers" — cannot be reached by any legitimate input, and an untested
/// branch that decides whether to fork a second copy of a node is not a branch
/// worth having.
///
///   first_load : silently discard the first `load` frame
///   status     : never answer a `query`
///
/// Nothing in a normal run reads this; the supervisor never sets it.
bool fault_enabled(const char * fault)
{
  const char * env = std::getenv("PLAY_LAUNCH_CONTROL_DROP");
  if (env == nullptr || *env == '\0') {
    return false;
  }
  return std::string(env).find(fault) != std::string::npos;
}

/// Read exactly `n` bytes, or return false (EOF, error, or shutdown).
bool read_exact(int fd, void * buf, size_t n)
{
  auto * p = static_cast<uint8_t *>(buf);
  size_t got = 0;
  while (got < n) {
    const ssize_t r = ::read(fd, p + got, n - got);
    if (r > 0) {
      got += static_cast<size_t>(r);
      continue;
    }
    if (r == 0) {
      return false;  // supervisor closed its end
    }
    if (errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

bool write_all(int fd, const void * buf, size_t n)
{
  const auto * p = static_cast<const uint8_t *>(buf);
  size_t sent = 0;
  while (sent < n) {
    const ssize_t w = ::write(fd, p + sent, n - sent);
    if (w > 0) {
      sent += static_cast<size_t>(w);
      continue;
    }
    if (w < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

/// One JSON parameter object → `rcl_interfaces/msg/Parameter`.
///
/// The type tag is authoritative: the supervisor sends the values already
/// typed by the model, so nothing is inferred here. An unknown tag yields
/// PARAMETER_NOT_SET, which is what the service path would produce for a
/// parameter it could not represent either.
rcl_interfaces::msg::Parameter parameter_from_json(const json::Value & v)
{
  using rcl_interfaces::msg::ParameterType;

  rcl_interfaces::msg::Parameter param;
  param.name = v["name"].as_string();
  const uint8_t type = static_cast<uint8_t>(v["type"].as_uint64(ParameterType::PARAMETER_NOT_SET));
  param.value.type = type;

  switch (type) {
    case ParameterType::PARAMETER_BOOL:
      param.value.bool_value = v["bool_value"].as_bool();
      break;
    case ParameterType::PARAMETER_INTEGER:
      param.value.integer_value = v["integer_value"].as_int64();
      break;
    case ParameterType::PARAMETER_DOUBLE:
      param.value.double_value = v["double_value"].as_double();
      break;
    case ParameterType::PARAMETER_STRING:
      param.value.string_value = v["string_value"].as_string();
      break;
    case ParameterType::PARAMETER_BYTE_ARRAY:
      for (const auto & item : v["byte_array_value"].items()) {
        param.value.byte_array_value.push_back(static_cast<uint8_t>(item.as_uint64()));
      }
      break;
    case ParameterType::PARAMETER_BOOL_ARRAY:
      for (const auto & item : v["bool_array_value"].items()) {
        param.value.bool_array_value.push_back(item.as_bool());
      }
      break;
    case ParameterType::PARAMETER_INTEGER_ARRAY:
      for (const auto & item : v["integer_array_value"].items()) {
        param.value.integer_array_value.push_back(item.as_int64());
      }
      break;
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      for (const auto & item : v["double_array_value"].items()) {
        param.value.double_array_value.push_back(item.as_double());
      }
      break;
    case ParameterType::PARAMETER_STRING_ARRAY:
      for (const auto & item : v["string_array_value"].items()) {
        param.value.string_array_value.push_back(item.as_string());
      }
      break;
    default:
      break;
  }
  return param;
}

}  // namespace

const char * phase_name(LoadPhase phase)
{
  switch (phase) {
    case LoadPhase::Queued:
      return "queued";
    case LoadPhase::Constructing:
      return "constructing";
    case LoadPhase::Loaded:
      return "loaded";
    case LoadPhase::Failed:
      return "failed";
    case LoadPhase::Unknown:
      break;
  }
  return "unknown";
}

std::shared_ptr<ControlChannel> ControlChannel::from_env()
{
  const char * env = std::getenv(kFdEnv);
  if (env == nullptr || *env == '\0') {
    return nullptr;
  }
  errno = 0;
  char * end = nullptr;
  const long fd = std::strtol(env, &end, 10);  // NOLINT(runtime/int) — strtol's own return type
  if (errno != 0 || end == env || *end != '\0' || fd < 0 || fd > 1024 * 1024) {
    std::fprintf(
      stderr, "[play_launch_container] ignoring %s='%s' (want an fd number)\n", kFdEnv, env);
    return nullptr;
  }
  // A stale value would otherwise get us writing frames into whatever else
  // happens to hold that descriptor.
  if (::fcntl(static_cast<int>(fd), F_GETFD) < 0) {
    std::fprintf(
      stderr, "[play_launch_container] %s=%ld is not an open descriptor: %s\n", kFdEnv, fd,
      std::strerror(errno));
    return nullptr;
  }
  return std::shared_ptr<ControlChannel>(new ControlChannel(static_cast<int>(fd)));
}

ControlChannel::ControlChannel(int fd) : fd_(fd)
{
  reader_ = std::thread(&ControlChannel::reader_loop, this);
}

ControlChannel::~ControlChannel() { stop(); }

void ControlChannel::stop()
{
  if (stopping_.exchange(true)) {
    return;
  }
  active_ = false;
  const int fd = fd_.load();
  if (fd >= 0) {
    // Wake the reader out of its blocking read. Not a close: another thread
    // may be inside `send_raw` on this descriptor, and closing it there would
    // let a later `open` hand the same number to something else mid-write.
    ::shutdown(fd, SHUT_RDWR);
  }
  if (reader_.joinable()) {
    reader_.join();
  }
  // Take the write lock so the close happens after any send in flight; every
  // send re-reads `fd_` under the same lock, so none starts afterwards.
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (fd >= 0) {
    ::close(fd);
  }
  fd_ = -1;
}

void ControlChannel::send_hello(bool loads_over_socket)
{
  json::ObjectWriter w;
  w.field("t", "hello")
    .field("protocol", static_cast<uint64_t>(kProtocolVersion))
    .field("pid", static_cast<int64_t>(::getpid()))
    .field("loads_over_socket", loads_over_socket);
  send_raw(w.str());
}

void ControlChannel::set_handlers(
  LoadHandler on_load, QueryHandler on_query, CancelHandler on_cancel)
{
  std::vector<PendingFrame> replay;
  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    load_handler_ = std::move(on_load);
    query_handler_ = std::move(on_query);
    cancel_handler_ = std::move(on_cancel);
    replay.swap(backlog_);
  }
  for (const auto & frame : replay) {
    deliver(frame);
  }
}

void ControlChannel::deliver(const PendingFrame & frame)
{
  // A throw here would reach the top of the reader thread and call
  // std::terminate, taking the container and every component in it down over
  // one bad request. Answering the request that caused it is both safer and
  // more informative.
  try {
    switch (frame.kind) {
      case PendingFrame::Kind::Load:
        if (load_handler_) {
          load_handler_(frame.seq, frame.request);
        }
        break;
      case PendingFrame::Kind::Query:
        if (query_handler_) {
          query_handler_(frame.query_seq, frame.query_id);
        }
        break;
      case PendingFrame::Kind::Cancel:
        if (cancel_handler_) {
          cancel_handler_(frame.cancel_id, frame.reason);
        }
        break;
    }
  } catch (const std::exception & ex) {
    std::fprintf(stderr, "[play_launch_container] control handler threw: %s\n", ex.what());
    if (frame.kind == PendingFrame::Kind::Load) {
      send_rejected(frame.seq, ex.what());
    } else if (frame.kind == PendingFrame::Kind::Cancel) {
      // A cancel MUST be answered: the supervisor waits for the confirmation
      // before it considers a resend, and silence there is a stuck load.
      send_load_failed(frame.cancel_id, ex.what(), true);
    }
  } catch (...) {
    std::fprintf(stderr, "[play_launch_container] control handler threw a non-exception\n");
    if (frame.kind == PendingFrame::Kind::Load) {
      send_rejected(frame.seq, "load handler threw a non-exception");
    } else if (frame.kind == PendingFrame::Kind::Cancel) {
      send_load_failed(frame.cancel_id, "cancel handler threw a non-exception", true);
    }
  }
}

void ControlChannel::send_accepted(uint64_t seq, uint64_t unique_id)
{
  json::ObjectWriter w;
  w.field("t", "accepted").field("seq", seq).field("unique_id", unique_id);
  send_raw(w.str());
}

void ControlChannel::send_rejected(uint64_t seq, const std::string & error)
{
  json::ObjectWriter w;
  w.field("t", "rejected").field("seq", seq).field("error", error);
  send_raw(w.str());
}

void ControlChannel::send_loaded(
  uint64_t unique_id, const std::string & full_node_name, int pid, uint64_t start_time)
{
  json::ObjectWriter w;
  w.field("t", "loaded")
    .field("unique_id", unique_id)
    .field("full_node_name", full_node_name)
    .field("pid", pid)
    .field("start_time", start_time);
  send_raw(w.str());
}

void ControlChannel::send_load_failed(uint64_t unique_id, const std::string & error, bool cancelled)
{
  json::ObjectWriter w;
  w.field("t", "load_failed")
    .field("unique_id", unique_id)
    .field("error", error)
    .field("cancelled", cancelled);
  send_raw(w.str());
}

void ControlChannel::send_constructing(
  uint64_t unique_id, int pid, uint64_t elapsed_ms, const std::string & plugin, LoadPhase phase,
  uint64_t cpu_ms)
{
  json::ObjectWriter w;
  w.field("t", "constructing")
    .field("unique_id", unique_id)
    .field("pid", pid)
    .field("elapsed_ms", elapsed_ms)
    .field("plugin", plugin)
    .field("phase", phase_name(phase))
    .field("cpu_ms", cpu_ms);
  send_raw(w.str());
}

void ControlChannel::send_status(
  std::optional<uint64_t> seq, uint64_t unique_id, LoadPhase phase, int pid, uint64_t elapsed_ms,
  uint64_t cpu_ms, const std::string & plugin, bool cancellable, const std::string & full_node_name)
{
  json::ObjectWriter w;
  w.field("t", "status");
  if (seq.has_value()) {
    w.field("seq", *seq);
  }
  w.field("unique_id", unique_id)
    .field("phase", phase_name(phase))
    .field("pid", pid)
    .field("elapsed_ms", elapsed_ms)
    .field("cpu_ms", cpu_ms)
    .field("plugin", plugin)
    .field("cancellable", cancellable)
    .field("full_node_name", full_node_name);
  send_raw(w.str());
}

void ControlChannel::send_unloaded(uint64_t unique_id, const std::string & full_node_name)
{
  json::ObjectWriter w;
  w.field("t", "unloaded").field("unique_id", unique_id).field("full_node_name", full_node_name);
  send_raw(w.str());
}

void ControlChannel::send_crashed(
  uint64_t unique_id, const std::string & full_node_name, const std::string & error, int pid)
{
  json::ObjectWriter w;
  w.field("t", "crashed")
    .field("unique_id", unique_id)
    .field("full_node_name", full_node_name)
    .field("error", error)
    .field("pid", pid);
  send_raw(w.str());
}

void ControlChannel::send_raw(const std::string & payload)
{
  if (!active_.load()) {
    return;
  }
  const uint32_t len = static_cast<uint32_t>(payload.size());
  uint8_t header[4];
  header[0] = static_cast<uint8_t>(len & 0xFF);
  header[1] = static_cast<uint8_t>((len >> 8) & 0xFF);
  header[2] = static_cast<uint8_t>((len >> 16) & 0xFF);
  header[3] = static_cast<uint8_t>((len >> 24) & 0xFF);

  std::lock_guard<std::mutex> lock(write_mutex_);
  const int fd = fd_.load();
  if (fd < 0) {
    return;  // stopped while we waited for the lock
  }
  if (!write_all(fd, header, sizeof(header)) || !write_all(fd, payload.data(), payload.size())) {
    // The supervisor is gone (or the socket broke). Stop trying: the
    // container keeps running and its ROS interfaces are untouched, which is
    // exactly the fallback the LoadNode service exists to be.
    if (active_.exchange(false)) {
      std::fprintf(
        stderr, "[play_launch_container] control channel write failed: %s\n", std::strerror(errno));
    }
  }
}

void ControlChannel::reader_loop()
{
  // The descriptor cannot change under this thread: `stop()` shuts the socket
  // down (which ends the read) and only closes it after joining here.
  const int fd = fd_.load();
  while (true) {
    uint8_t header[4];
    if (!read_exact(fd, header, sizeof(header))) {
      break;
    }
    const uint32_t len =
      static_cast<uint32_t>(header[0]) | (static_cast<uint32_t>(header[1]) << 8) |
      (static_cast<uint32_t>(header[2]) << 16) | (static_cast<uint32_t>(header[3]) << 24);
    if (len > kMaxFrameBytes) {
      std::fprintf(
        stderr, "[play_launch_container] control frame of %u bytes exceeds the cap\n", len);
      break;
    }
    std::string payload(len, '\0');
    if (len > 0 && !read_exact(fd, payload.data(), len)) {
      break;
    }
    dispatch(payload);
  }
  active_ = false;
}

void ControlChannel::dispatch(const std::string & payload)
{
  json::Value msg;
  std::string error;
  if (!json::parse(payload, &msg, &error) || !msg.is_object()) {
    std::fprintf(stderr, "[play_launch_container] unparseable control frame: %s\n", error.c_str());
    return;
  }

  const std::string type = msg["t"].as_string();
  if (type == "hello") {
    const uint32_t peer = static_cast<uint32_t>(msg["protocol"].as_uint64());
    if (peer != kProtocolVersion) {
      std::fprintf(
        stderr,
        "[play_launch_container] control protocol v%u from play_launch != v%u here; "
        "falling back to the LoadNode service\n",
        peer, kProtocolVersion);
      active_ = false;
    }
    return;
  }

  if (type == "query") {
    if (fault_enabled("status")) {
      std::fprintf(stderr, "[play_launch_container] FAULT INJECTION: ignoring query\n");
      return;
    }
    PendingFrame frame;
    frame.kind = PendingFrame::Kind::Query;
    // Presence, not value: the supervisor asks by `seq` before an id exists
    // and by `unique_id` afterwards, and answering the wrong key would report
    // `unknown` for a load that is in fact in flight.
    if (!msg["seq"].is_null()) {
      frame.query_seq = msg["seq"].as_uint64();
    }
    if (!msg["unique_id"].is_null()) {
      frame.query_id = msg["unique_id"].as_uint64();
    }
    enqueue_or_deliver(frame);
    return;
  }

  if (type == "cancel") {
    PendingFrame frame;
    frame.kind = PendingFrame::Kind::Cancel;
    frame.cancel_id = msg["unique_id"].as_uint64();
    frame.reason = msg["reason"].as_string("cancelled by supervisor");
    enqueue_or_deliver(frame);
    return;
  }

  if (type != "load") {
    // Unknown message kinds are skipped rather than fatal: a newer supervisor
    // may say more than this container understands, and the framing is
    // unaffected.
    std::fprintf(stderr, "[play_launch_container] ignoring control frame '%s'\n", type.c_str());
    return;
  }

  auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
  request->package_name = msg["package"].as_string();
  request->plugin_name = msg["plugin"].as_string();
  request->node_name = msg["node_name"].as_string();
  request->node_namespace = msg["node_namespace"].as_string();
  for (const auto & item : msg["remap_rules"].items()) {
    request->remap_rules.push_back(item.as_string());
  }
  for (const auto & item : msg["parameters"].items()) {
    request->parameters.push_back(parameter_from_json(item));
  }
  for (const auto & item : msg["extra_arguments"].items()) {
    request->extra_arguments.push_back(parameter_from_json(item));
  }

  // The per-node log directory rides its own field on the wire, but reaches
  // the spawn path the way it always has: as an `extra_arguments` entry. That
  // keeps ONE reader of it (`spawn_child_process`) rather than two.
  const std::string log_dir = msg["log_dir"].as_string();
  if (!log_dir.empty()) {
    rcl_interfaces::msg::Parameter param;
    param.name = "log_dir";
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    param.value.string_value = log_dir;
    request->extra_arguments.push_back(param);
  }

  PendingFrame frame;
  frame.kind = PendingFrame::Kind::Load;
  frame.seq = msg["seq"].as_uint64();
  frame.request = request;

  static std::atomic<bool> dropped_first_load{false};
  if (fault_enabled("first_load") && !dropped_first_load.exchange(true)) {
    std::fprintf(
      stderr, "[play_launch_container] FAULT INJECTION: dropping load seq %" PRIu64 "\n",
      frame.seq);
    return;
  }

  enqueue_or_deliver(frame);
}

/// Hand a frame to its handler, or hold it in order until handlers exist.
void ControlChannel::enqueue_or_deliver(const PendingFrame & frame)
{
  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    if (!load_handler_) {
      // The supervisor can send frames the instant it sees our hello, which is
      // before the component manager exists. Holding them — in arrival order,
      // across kinds — is what keeps a `query` from answering `unknown` for a
      // load still sitting behind it in this very list.
      backlog_.push_back(frame);
      return;
    }
  }
  deliver(frame);
}

}  // namespace play_launch_container
