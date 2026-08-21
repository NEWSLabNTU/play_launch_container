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

#ifndef PLAY_LAUNCH_CONTAINER__CONTROL_CHANNEL_HPP_
#define PLAY_LAUNCH_CONTAINER__CONTROL_CHANNEL_HPP_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "composition_interfaces/srv/load_node.hpp"

namespace play_launch_container
{

/// Phase 64: the container half of the private control channel to play_launch.
///
/// play_launch hands this process one end of a `socketpair(2)` and names the
/// fd in `PLAY_LAUNCH_CONTROL_FD`. Load requests and load status travel over
/// it instead of over `composition_interfaces/srv/LoadNode` and the
/// `ComponentEvent` topic — not because those are wrong, but because during a
/// 150-process startup they are the most congested path on the machine, and
/// both ends of THIS conversation are play_launch's own code.
///
/// Protocol: 4-byte little-endian length, then a JSON object. Defined in
/// `src/play_launch/src/ipc/container_protocol.rs`.
///
/// Nothing here needs `rclcpp`: the channel is opened and the hello sent
/// BEFORE `rclcpp::init`, so the supervisor learns the channel is live without
/// waiting for DDS discovery. That is also why it logs to stderr.
/// Where a load is, as the container sees it. Mirrors `LoadPhase` in
/// `src/play_launch/src/ipc/container_protocol.rs`.
enum class LoadPhase { Queued, Constructing, Loaded, Failed, Unknown };

/// The wire spelling of a phase. `unknown` is the only one that authorises the
/// supervisor to resend, so these strings are load-bearing.
const char * phase_name(LoadPhase phase);

class ControlChannel
{
public:
  /// Called on the reader thread for each `load` frame. `request` is the same
  /// type the LoadNode service delivers, so everything downstream of it is
  /// the code that was already there.
  using LoadHandler = std::function<void(
    uint64_t seq, std::shared_ptr<composition_interfaces::srv::LoadNode::Request> request)>;

  /// Answer "what is the state of this load?". Exactly one of the two keys is
  /// set: `unique_id` once the load was accepted, `seq` before that.
  using QueryHandler =
    std::function<void(std::optional<uint64_t> seq, std::optional<uint64_t> unique_id)>;

  /// Stop a load and destroy whatever it created, then confirm. The
  /// confirmation is what makes a resend safe, so a handler must always
  /// answer — including for an id it does not recognise.
  using CancelHandler = std::function<void(uint64_t unique_id, const std::string & reason)>;

  /// Open the channel named by `PLAY_LAUNCH_CONTROL_FD`, or return nullptr
  /// when the variable is absent (a container started by hand, or by a
  /// play_launch old enough not to offer one).
  static std::shared_ptr<ControlChannel> from_env();

  ~ControlChannel();

  /// Announce ourselves. `loads_over_socket` is false for the non-isolated
  /// manager, where loading has to run on the executor thread; there the
  /// channel carries status only and LoadNode stays in charge.
  void send_hello(bool loads_over_socket);

  /// Install the handlers and replay everything that arrived before they
  /// existed — the supervisor may send frames the instant it sees the hello,
  /// which is before the component manager has been constructed.
  ///
  /// The backlog is replayed IN ORDER across kinds. A `query` answered out of
  /// order would report `unknown` for a load still sitting in the backlog, and
  /// `unknown` is precisely the answer that authorises a resend — so the
  /// ordering is not tidiness, it is what stops a double load.
  void set_handlers(LoadHandler on_load, QueryHandler on_query, CancelHandler on_cancel);

  void send_accepted(uint64_t seq, uint64_t unique_id);
  void send_rejected(uint64_t seq, const std::string & error);
  void send_loaded(
    uint64_t unique_id, const std::string & full_node_name, int pid, uint64_t start_time);
  void send_load_failed(uint64_t unique_id, const std::string & error, bool cancelled = false);
  void send_constructing(
    uint64_t unique_id, int pid, uint64_t elapsed_ms, const std::string & plugin, LoadPhase phase,
    uint64_t cpu_ms);
  /// Answer a `query`. `seq` is echoed when the question was asked by seq.
  void send_status(
    std::optional<uint64_t> seq, uint64_t unique_id, LoadPhase phase, int pid, uint64_t elapsed_ms,
    uint64_t cpu_ms, const std::string & plugin, bool cancellable,
    const std::string & full_node_name);
  void send_unloaded(uint64_t unique_id, const std::string & full_node_name);
  void send_crashed(
    uint64_t unique_id, const std::string & full_node_name, const std::string & error, int pid);

  /// Whether the channel is still usable (the supervisor has not gone away
  /// and the protocol version matched).
  bool active() const { return active_.load(); }

  /// Close the socket and join the reader thread. Idempotent.
  void stop();

private:
  explicit ControlChannel(int fd);

  void reader_loop();
  void dispatch(const std::string & payload);
  void send_raw(const std::string & json);

  /// Atomic because `stop()` clears it while a send may be in flight; the
  /// close itself is ordered behind `write_mutex_`.
  std::atomic<int> fd_;
  std::atomic<bool> active_{true};
  std::atomic<bool> stopping_{false};
  std::thread reader_;
  std::mutex write_mutex_;

  /// One frame held until handlers exist. Kept as a single ordered list
  /// rather than one queue per kind — see `set_handlers`.
  struct PendingFrame
  {
    enum class Kind { Load, Query, Cancel } kind;
    uint64_t seq = 0;
    std::shared_ptr<composition_interfaces::srv::LoadNode::Request> request;
    std::optional<uint64_t> query_seq;
    std::optional<uint64_t> query_id;
    uint64_t cancel_id = 0;
    std::string reason;
  };

  void deliver(const PendingFrame & frame);
  void enqueue_or_deliver(const PendingFrame & frame);

  std::mutex handler_mutex_;
  LoadHandler load_handler_;
  QueryHandler query_handler_;
  CancelHandler cancel_handler_;
  std::vector<PendingFrame> backlog_;
};

}  // namespace play_launch_container

#endif  // PLAY_LAUNCH_CONTAINER__CONTROL_CHANNEL_HPP_
