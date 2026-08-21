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

#include "play_launch_container/clone_isolated_component_manager.hpp"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace play_launch_container
{

// ── Helper: resolve component_node binary path ──────────────────────────
//
// component_node is installed next to component_container under
// lib/<project_name>/.  We find our own directory via /proc/self/exe.

static std::string resolve_component_node_path()
{
  char buf[4096];
  ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (len <= 0) {
    return "component_node";  // fallback: hope it's on PATH
  }
  buf[len] = '\0';
  std::string exe_path(buf);
  auto slash = exe_path.rfind('/');
  if (slash == std::string::npos) {
    return "component_node";
  }
  return exe_path.substr(0, slash + 1) + "component_node";
}

// ── Helper: how long to wait for a forked component to report ready ─────
//
// A component's constructor runs before it can answer, and some of them take
// far longer than a service call normally would: Autoware's traffic light
// classifier spends ~33s building a TensorRT engine from ONNX on a cold cache
// and ~45s of wall clock even with the engine cached. At the former fixed 30s
// this manager SIGKILLed the child seconds before it would have reported, the
// LoadNode retry forked a fresh child that died the same way, and the node
// simply never existed -- with the rest of the pipeline up and publishing empty
// results, which is a hard failure to read from the outside.
//
// The default stays 30s so nothing changes for anyone not asking. Set
// PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS to raise it for stacks that load
// inference models. Note this is the *first* wait: the Rust side already
// budgets 60s per retry and up to 600s total for exactly these slow
// constructors, so a 30s hard kill here undercuts that design.
// ── Helper: how many spawn workers, and when a spawn may proceed ────────
//
// The worker pool used to be a fixed 4, and that number was the real limit on
// how fast a container could bring its composables up: each worker is held for
// the whole of a child's CONSTRUCTOR, so five slow constructors ran in two
// waves regardless of what the machine could have done. Measured with a 20 s
// constructor: six of them plus one fast node landed at t+20s (x4) and t+40s
// (x2) — two waves, purely from the pool.
//
// That matters on a perception stack. Autoware's camera-LiDAR-fusion preset
// puts SIX TensorRT-loading components in the traffic light container alone,
// each ~45 s of construction with the engine cached.
//
// A count is the wrong governor for this. It cannot tell a component that is
// slow because it is COMPUTING (a TensorRT build: CPU-hungry, memory-hungry,
// and on Tegra its GPU allocations come out of system RAM — genuinely worth
// throttling) from one that is slow because it is WAITING (on discovery, on a
// service, on a parameter server — costing nothing while it holds a slot). So
// the pool is sized to stay out of the way, and admission is governed by what
// is actually scarce: free memory.
static size_t spawn_worker_count()
{
  const char * env = std::getenv("PLAY_LAUNCH_SPAWN_WORKERS");
  if (env != nullptr && *env != '\0') {
    errno = 0;
    char * end = nullptr;
    const int64_t parsed = std::strtoll(env, &end, 10);
    if (errno == 0 && end != env && *end == '\0' && parsed > 0 && parsed <= 256) {
      return static_cast<size_t>(parsed);
    }
    fprintf(
      stderr,
      "[play_launch_container] ignoring PLAY_LAUNCH_SPAWN_WORKERS='%s' "
      "(want an integer in 1..256)\n",
      env);
  }
  const unsigned hw = std::thread::hardware_concurrency();
  const size_t n = (hw == 0) ? 4u : static_cast<size_t>(hw);
  // Enough that the pool is not the limiter; the memory gate below is.
  return std::min<size_t>(std::max<size_t>(n, 4), 32);
}

/// MemAvailable in kB, or -1 if it cannot be read.
static int64_t mem_available_kb()
{
  std::ifstream f("/proc/meminfo");
  std::string key;
  int64_t value = 0;
  std::string unit;
  while (f >> key >> value >> unit) {
    if (key == "MemAvailable:") {
      return value;
    }
  }
  return -1;
}

/// Free-memory floor below which a spawn waits, in kB. 0 disables the gate.
///
/// Absolute rather than a share of RAM, for the same reason play_launch's own
/// process floor is: what it guards against is one more child allocating
/// before the next check, and a component needs what it needs regardless of
/// how big the machine is. Capped at a quarter of RAM so a small board is not
/// asked to keep more free than it has.
static int64_t spawn_floor_kb()
{
  const char * env = std::getenv("PLAY_LAUNCH_SPAWN_MIN_AVAIL_MB");
  if (env != nullptr && *env != '\0') {
    errno = 0;
    char * end = nullptr;
    const int64_t parsed = std::strtoll(env, &end, 10);
    if (errno == 0 && end != env && *end == '\0' && parsed >= 0 && parsed <= 1024LL * 1024LL) {
      return parsed * 1024L;  // MB -> kB; 0 disables
    }
    fprintf(
      stderr,
      "[play_launch_container] ignoring PLAY_LAUNCH_SPAWN_MIN_AVAIL_MB='%s' "
      "(want an integer number of MB, 0 to disable)\n",
      env);
  }

  constexpr int64_t kDefaultFloorKb = 1024LL * 1024LL;  // 1 GiB
  std::ifstream f("/proc/meminfo");
  std::string key;
  int64_t value = 0;
  std::string unit;
  while (f >> key >> value >> unit) {
    if (key == "MemTotal:") {
      return std::min(kDefaultFloorKb, value / 4);
    }
  }
  return kDefaultFloorKb;
}

static int ready_timeout_ms()
{
  const char * env = std::getenv("PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS");
  if (env != nullptr && *env != '\0') {
    errno = 0;
    char * end = nullptr;
    const int64_t parsed = std::strtoll(env, &end, 10);
    if (errno == 0 && end != env && *end == '\0' && parsed >= 0 && parsed <= 3600000) {
      return static_cast<int>(parsed);
    }
    fprintf(
      stderr,
      "[play_launch_container] ignoring PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS='%s' "
      "(want an integer number of milliseconds in 0..3600000, 0 for no deadline)\n",
      env);
  }
  // No deadline. There is no number that is right on every platform: the wait
  // covers the node's CONSTRUCTOR, and a first-run TensorRT engine build takes
  // however long that machine's GPU takes — measured at ~33s cold and ~45s even
  // with the engine cached for Autoware's traffic light classifier, on one
  // board. A fixed default is a guess about hardware we do not have, and when
  // it is wrong it does not degrade: it SIGKILLs the node partway through work
  // that is proceeding normally, discards it (the .engine is written last), and
  // does the same on every relaunch.
  //
  // So the wait is bounded by LIVENESS instead of by time — see the poll loop,
  // which gives up the moment the child dies and otherwise reports progress.
  // Set the variable to bound a node that wedges rather than works.
  return 0;
}

// starttime (clock ticks since boot) from /proc/<pid>/stat field 22.
// Returns 0 on any parse/read failure. Field 2 (comm) may contain spaces or
// parens, so parse from AFTER the last ')'.
static uint64_t proc_start_time(pid_t pid)
{
  std::ifstream stat_file("/proc/" + std::to_string(pid) + "/stat");
  if (!stat_file.is_open()) {
    return 0;
  }

  std::string line;
  if (!std::getline(stat_file, line)) {
    return 0;
  }

  auto close_paren = line.rfind(')');
  if (close_paren == std::string::npos) {
    return 0;
  }

  // Fields after ')': starting with field 3 (state). starttime is field 22
  // overall, i.e. the 20th field counting from field 3.
  std::istringstream rest(line.substr(close_paren + 1));
  std::string field;
  // Skip fields 3..21 (19 fields) to land on field 22 (starttime).
  for (int i = 0; i < 19; ++i) {
    if (!(rest >> field)) {
      return 0;
    }
  }

  uint64_t starttime = 0;
  if (!(rest >> starttime)) {
    return 0;
  }
  return starttime;
}

// CPU time (utime+stime) of a process in milliseconds, or 0 if unreadable.
//
// This is the evidence half of the stall question. A constructor burning CPU
// is working; one burning none MIGHT be wedged, and might equally be blocked
// on a service that has not come up yet — which is the ordinary shape of an
// Autoware startup. So the container reports it and never acts on it.
static uint64_t proc_cpu_ms(pid_t pid)
{
  std::ifstream stat_file("/proc/" + std::to_string(pid) + "/stat");
  if (!stat_file.is_open()) {
    return 0;
  }
  std::string line;
  if (!std::getline(stat_file, line)) {
    return 0;
  }
  const auto close_paren = line.rfind(')');
  if (close_paren == std::string::npos) {
    return 0;
  }
  // Fields after ')' start at field 3. utime is field 14, stime is field 15,
  // i.e. the 12th and 13th counting from field 3.
  std::istringstream rest(line.substr(close_paren + 1));
  std::string field;
  for (int i = 0; i < 11; ++i) {
    if (!(rest >> field)) {
      return 0;
    }
  }
  uint64_t utime = 0;
  uint64_t stime = 0;
  if (!(rest >> utime >> stime)) {
    return 0;
  }
  const long ticks = sysconf(_SC_CLK_TCK);
  if (ticks <= 0) {
    return 0;
  }
  return ((utime + stime) * 1000ULL) / static_cast<uint64_t>(ticks);
}

// ── Constructor / Destructor ────────────────────────────────────────────

CloneIsolatedComponentManager::CloneIsolatedComponentManager(
  std::weak_ptr<rclcpp::Executor> executor, bool use_multi_threaded, std::string node_name,
  const rclcpp::NodeOptions & node_options)
: ObservableComponentManager(executor, std::move(node_name), node_options),
  use_multi_threaded_(use_multi_threaded),
  component_node_path_(resolve_component_node_path())
{
  RCLCPP_INFO(get_logger(), "Using fork()+exec() per-node process isolation (non-blocking load)");
  RCLCPP_DEBUG(get_logger(), "component_node binary: %s", component_node_path_.c_str());

  // Start worker thread pool for async node spawning
  workers_running_ = true;
  const size_t worker_count = spawn_worker_count();
  for (size_t i = 0; i < worker_count; ++i) {
    worker_threads_.emplace_back(&CloneIsolatedComponentManager::worker_loop, this);
  }

  // Set up child death monitor (epoll on pidfds)
  epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
  if (epoll_fd_ < 0) {
    RCLCPP_WARN(
      get_logger(), "epoll_create1 failed: %s — child crash monitoring disabled",
      std::strerror(errno));
    return;
  }

  stop_fd_ = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
  if (stop_fd_ < 0) {
    RCLCPP_WARN(
      get_logger(), "eventfd failed: %s — child crash monitoring disabled", std::strerror(errno));
    close(epoll_fd_);
    epoll_fd_ = -1;
    return;
  }

  // Register stop_fd_ with epoll (data.u64=0 as sentinel — node IDs start at 1)
  epoll_event ev{};
  ev.events = EPOLLIN;
  ev.data.u64 = 0;
  epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, stop_fd_, &ev);

  monitor_running_ = true;
  monitor_thread_ = std::thread(&CloneIsolatedComponentManager::monitor_loop, this);
}

CloneIsolatedComponentManager::~CloneIsolatedComponentManager()
{
  // Phase 64: close the control channel before anything else. Its reader
  // thread calls handle_control_load on this object, and `stop()` joins that
  // thread — after which no load can arrive into a half-destroyed manager.
  if (control_) {
    control_->stop();
  }

  // Stop worker threads first (they may hold load_mutex_)
  {
    std::lock_guard<std::mutex> lock(work_queue_mutex_);
    workers_running_ = false;
  }
  work_queue_cv_.notify_all();
  for (auto & t : worker_threads_) {
    if (t.joinable()) {
      t.join();
    }
  }

  // Stop monitor thread before cleaning up children
  if (monitor_thread_.joinable()) {
    monitor_running_ = false;
    if (stop_fd_ >= 0) {
      uint64_t val = 1;
      if (write(stop_fd_, &val, sizeof(val)) < 0) {
        // best-effort wakeup — ignore errors
      }
    }
    monitor_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(children_mutex_);
    for (auto & [node_id, child] : children_) {
      cleanup_child(child);
    }
    children_.clear();
  }

  if (stop_fd_ >= 0) {
    close(stop_fd_);
  }
  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
  }
}

// ── Worker thread pool ──────────────────────────────────────────────────

// Wait until there is room in memory for another child.
//
// Serialised on a mutex so the check and the fork that follows it are not
// raced by every other worker: without that, N workers all observe the same
// MemAvailable, all conclude there is room, and all fork — the gate reads as
// satisfied exactly once and then admits everyone. Holding the mutex costs
// microseconds when memory is plentiful, because the first check passes and
// nothing sleeps.
//
// Never blocks forever. A launch that will not start is worse than one that
// starts under pressure, so after `kMaxWaitMs` the spawn proceeds with a
// warning naming the condition — the same rule play_launch applies to its own
// admission gates.
void CloneIsolatedComponentManager::await_spawn_capacity(
  uint64_t node_id, const std::string & plugin)
{
  const int64_t floor_kb = spawn_floor_kb();
  if (floor_kb <= 0) {
    return;  // gate disabled
  }

  constexpr int kPollMs = 250;
  constexpr int kMaxWaitMs = 120000;
  constexpr int kProgressEveryMs = 15000;

  std::lock_guard<std::mutex> admit(spawn_admit_mutex_);

  int waited_ms = 0;
  bool waited = false;
  while (true) {
    const int64_t avail = mem_available_kb();
    if (avail < 0) {
      return;  // cannot measure; do not pretend to govern
    }
    if (avail >= floor_kb) {
      break;
    }
    waited = true;
    if (waited_ms >= kMaxWaitMs) {
      RCLCPP_WARN(
        get_logger(),
        "Spawning '%s' with only %ld MiB available (floor %ld MiB) after waiting %ds — "
        "proceeding rather than stalling the launch",
        plugin.c_str(), avail / 1024, floor_kb / 1024, waited_ms / 1000);
      break;
    }
    if (waited_ms % kProgressEveryMs == 0) {
      RCLCPP_INFO(
        get_logger(), "Holding spawn of '%s': %ld MiB available, want %ld MiB", plugin.c_str(),
        avail / 1024, floor_kb / 1024);
      // Phase 64 W2: say it on the socket too. Nothing has been forked yet, so
      // there is no pid and no liveness to report — `queued` IS the report,
      // and without it this two-minute window looks identical to a load that
      // was never received.
      if (control_) {
        control_->send_constructing(
          node_id, 0, static_cast<uint64_t>(waited_ms), plugin, LoadPhase::Queued, 0);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    waited_ms += kPollMs;
  }

  // Only when memory was actually tight: give the child just admitted a moment
  // to show up in MemAvailable, so the next caller measures the world it is
  // really entering. Skipped entirely on the common path.
  if (waited) {
    std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
  }
}

void CloneIsolatedComponentManager::worker_loop()
{
  while (true) {
    std::function<void()> work;
    {
      std::unique_lock<std::mutex> lock(work_queue_mutex_);
      work_queue_cv_.wait(lock, [this] { return !workers_running_ || !work_queue_.empty(); });
      if (!workers_running_) {
        // Exit immediately on shutdown — don't process remaining items,
        // the rcl context may already be invalid.
        return;
      }
      work = std::move(work_queue_.front());
      work_queue_.pop();
    }
    work();
  }
}

void CloneIsolatedComponentManager::submit_work(std::function<void()> work)
{
  {
    std::lock_guard<std::mutex> lock(work_queue_mutex_);
    work_queue_.push(std::move(work));
  }
  work_queue_cv_.notify_one();
}

// ── Parameter serialization ─────────────────────────────────────────────
//
// Convert LoadNode::Request parameters (rcl_interfaces/Parameter[])
// to a YAML file that can be passed via --params-file.

std::string CloneIsolatedComponentManager::write_params_file(
  const std::shared_ptr<LoadNode::Request> & request)
{
  if (request->parameters.empty()) {
    return "";
  }

  // Use wildcard namespace — component_node uses use_global_arguments(true)
  // so the YAML namespace must match any node name.
  std::ostringstream yaml;
  yaml << "/**:\n";
  yaml << "  ros__parameters:\n";

  for (const auto & param : request->parameters) {
    yaml << "    " << param.name << ": ";

    const auto & val = param.value;
    switch (val.type) {
      case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
        yaml << (val.bool_value ? "true" : "false");
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
        yaml << val.integer_value;
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE: {
        char dbuf[64];
        std::snprintf(dbuf, sizeof(dbuf), "%.17g", val.double_value);
        std::string ds(dbuf);
        // Ensure decimal point for ROS type preservation
        if (ds.find('.') == std::string::npos && ds.find('e') == std::string::npos) {
          ds += ".0";
        }
        yaml << ds;
        break;
      }
      case rcl_interfaces::msg::ParameterType::PARAMETER_STRING: {
        const auto & sv = param.value.string_value;
        // Use YAML double-quoted style with proper escaping for strings that
        // contain quotes, colons, or other YAML-special characters.
        // This handles values like URDF XML or ROS message type paths (::).
        bool needs_quoting =
          sv.empty() || sv.find('\'') != std::string::npos || sv.find('"') != std::string::npos ||
          sv.find(':') != std::string::npos || sv.find('#') != std::string::npos ||
          sv.find('\n') != std::string::npos || sv.find('{') != std::string::npos ||
          sv.find('}') != std::string::npos || sv.find('[') != std::string::npos ||
          sv.find(']') != std::string::npos || sv.find('*') != std::string::npos ||
          sv.find('&') != std::string::npos || sv.find('!') != std::string::npos ||
          sv.find('|') != std::string::npos || sv.find('>') != std::string::npos ||
          sv.find('<') != std::string::npos || sv.find('%') != std::string::npos ||
          sv.find('+') != std::string::npos;
        if (needs_quoting) {
          // Escape \ and " for YAML double-quoted strings
          yaml << '"';
          for (char c : sv) {
            if (c == '"') {
              yaml << "\\\"";
            } else if (c == '\\') {
              yaml << "\\\\";
            } else if (c == '\n') {
              yaml << "\\n";
            } else if (c == '\t') {
              yaml << "\\t";
            } else {
              yaml << c;
            }
          }
          yaml << '"';
        } else {
          yaml << "'" << sv << "'";
        }
        break;
      }
      case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY: {
        yaml << "[";
        for (size_t i = 0; i < val.byte_array_value.size(); ++i) {
          if (i > 0) yaml << ", ";
          yaml << static_cast<int>(val.byte_array_value[i]);
        }
        yaml << "]";
        break;
      }
      case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY: {
        yaml << "[";
        for (size_t i = 0; i < val.bool_array_value.size(); ++i) {
          if (i > 0) yaml << ", ";
          yaml << (val.bool_array_value[i] ? "true" : "false");
        }
        yaml << "]";
        break;
      }
      case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY: {
        yaml << "[";
        for (size_t i = 0; i < val.integer_array_value.size(); ++i) {
          if (i > 0) yaml << ", ";
          yaml << val.integer_array_value[i];
        }
        yaml << "]";
        break;
      }
      case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY: {
        yaml << "[";
        for (size_t i = 0; i < val.double_array_value.size(); ++i) {
          if (i > 0) yaml << ", ";
          char dbuf[64];
          std::snprintf(dbuf, sizeof(dbuf), "%.17g", val.double_array_value[i]);
          std::string ds(dbuf);
          // Ensure decimal point for ROS type preservation
          if (ds.find('.') == std::string::npos && ds.find('e') == std::string::npos) {
            ds += ".0";
          }
          yaml << ds;
        }
        yaml << "]";
        break;
      }
      case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY: {
        yaml << "[";
        for (size_t i = 0; i < val.string_array_value.size(); ++i) {
          if (i > 0) yaml << ", ";
          yaml << "'" << val.string_array_value[i] << "'";
        }
        yaml << "]";
        break;
      }
      default:
        yaml << "null";
        break;
    }
    yaml << "\n";
  }

  // Write to temp file
  char tmp_path[] = "/tmp/play_launch_params_XXXXXX";
  int fd = mkstemp(tmp_path);
  if (fd < 0) {
    RCLCPP_WARN(get_logger(), "Failed to create temp params file: %s", std::strerror(errno));
    return "";
  }

  std::string content = yaml.str();
  size_t written = 0;
  while (written < content.size()) {
    auto n = ::write(fd, content.data() + written, content.size() - written);
    if (n <= 0) {
      break;
    }
    written += static_cast<size_t>(n);
  }
  close(fd);

  return std::string(tmp_path);
}

// ── spawn_child_process ─────────────────────────────────────────────────
//
// Fork+exec the component_node binary for a single composable node.
// Reads the ready pipe to get the node's full name or error message.

CloneIsolatedComponentManager::ChildInfo CloneIsolatedComponentManager::spawn_child_process(
  uint64_t node_id, const std::shared_ptr<LoadNode::Request> & request)
{
  // Build argument list for component_node
  std::vector<std::string> args;
  args.push_back(component_node_path_);
  args.push_back("--package");
  args.push_back(request->package_name);
  args.push_back("--plugin");
  args.push_back(request->plugin_name);

  // Ready pipe fd — will be set after pipe() below
  args.push_back("--ready-fd");
  args.push_back("");  // placeholder, filled after pipe()

  if (use_multi_threaded_) {
    args.push_back("--use-multi-threaded-executor");
  }

  // Check extra_arguments for use_intra_process_comms and log_dir
  std::string log_dir;
  for (const auto & extra : request->extra_arguments) {
    if (
      extra.name == "use_intra_process_comms" &&
      extra.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL &&
      extra.value.bool_value) {
      args.push_back("--use-intra-process-comms");
    } else if (
      extra.name == "log_dir" &&
      extra.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      log_dir = extra.value.string_value;
    }
  }

  // Serialize parameters to temp YAML
  std::string param_file = write_params_file(request);

  // --ros-args section
  args.push_back("--ros-args");

  // Node name and namespace remapping
  if (!request->node_name.empty()) {
    args.push_back("-r");
    args.push_back("__node:=" + request->node_name);
  }
  if (!request->node_namespace.empty()) {
    std::string ns = request->node_namespace;
    if (ns[0] != '/') {
      ns = "/" + ns;
    }
    args.push_back("-r");
    args.push_back("__ns:=" + ns);
  }

  // Log level (uint8: 0=unset, 10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=FATAL)
  if (request->log_level > 0) {
    args.push_back("--log-level");
    args.push_back(std::to_string(request->log_level));
  }

  // Extra remap rules
  for (const auto & remap : request->remap_rules) {
    args.push_back("-r");
    args.push_back(remap);
  }

  // Params file
  if (!param_file.empty()) {
    args.push_back("--params-file");
    args.push_back(param_file);
  }

  // Create ready pipe
  int pipefd[2];
  if (pipe(pipefd) != 0) {
    if (!param_file.empty()) {
      unlink(param_file.c_str());
    }
    throw std::runtime_error("pipe() failed: " + std::string(std::strerror(errno)));
  }

  // Fill in the ready-fd placeholder
  args[6] = std::to_string(pipefd[1]);

  // Build C-style argv
  std::vector<char *> c_argv;
  for (auto & a : args) {
    c_argv.push_back(a.data());
  }
  c_argv.push_back(nullptr);

  pid_t parent_pid = getpid();
  pid_t child_pid = fork();
  if (child_pid < 0) {
    int err = errno;
    close(pipefd[0]);
    close(pipefd[1]);
    if (!param_file.empty()) {
      unlink(param_file.c_str());
    }
    throw std::runtime_error("fork() failed: " + std::string(std::strerror(err)));
  }

  if (child_pid == 0) {
    // ── Child process ──
    close(pipefd[0]);  // close read end

    // Ask the kernel to send SIGKILL to this child if the parent (container)
    // dies for any reason (including SIGKILL).  This prevents orphans.
    // Must be called after fork() but before exec() — PR_SET_PDEATHSIG is
    // reset across setuid exec but preserved across normal exec.
    // Uses SIGKILL (not SIGTERM) because ROS nodes stuck in blocking calls
    // may not exit on SIGTERM, leaving orphan processes.
    prctl(PR_SET_PDEATHSIG, SIGKILL);

    // Guard against a race: if the parent already died between fork() and
    // the prctl() above, the child has been reparented.  Compare against
    // the saved parent PID rather than checking for init (PID 1), because
    // play_launch sets PR_SET_CHILD_SUBREAPER which causes reparenting to
    // play_launch instead of init.
    if (getppid() != parent_pid) {
      _exit(1);
    }

    // Redirect stdout/stderr to per-node log files if log_dir is set
    if (!log_dir.empty()) {
      std::string out_path = log_dir + "/out";
      std::string err_path = log_dir + "/err";
      int out_fd = open(out_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
      int err_fd = open(err_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
      if (out_fd >= 0) {
        dup2(out_fd, STDOUT_FILENO);
        close(out_fd);
      }
      if (err_fd >= 0) {
        dup2(err_fd, STDERR_FILENO);
        close(err_fd);
      }
    }

    // The write end (pipefd[1]) must NOT be close-on-exec since component_node
    // reads the fd number from --ready-fd.  pipe() fds are not CLOEXEC by default.

    execvp(c_argv[0], c_argv.data());

    // If exec fails, write error to pipe and exit
    std::string err = "ERR execvp failed: " + std::string(std::strerror(errno));
    err += "\n";
    if (write(pipefd[1], err.data(), err.size()) < 0) {
      // Suppress warn_unused_result; best-effort write before _exit
    }
    close(pipefd[1]);
    _exit(127);
  }

  // ── Parent process ──
  close(pipefd[1]);  // close write end

  // The child exists: the load has left `queued` and a query can now be
  // answered with a pid.
  note_constructing(node_id, child_pid);

  // Wait for the child to report ready. This covers its CONSTRUCTOR, so it can
  // legitimately run for minutes; see ready_timeout_ms() for why there is no
  // default deadline.
  struct pollfd pfd
  {
  };
  pfd.fd = pipefd[0];
  pfd.events = POLLIN;
  const int kReadyTimeoutMs = ready_timeout_ms();  // 0 = no deadline

  // Poll in slices so liveness and progress can be checked while waiting.
  constexpr int kPollSliceMs = 1000;
  constexpr int kProgressEverySec = 15;

  // Why the wait ended. Recorded at each exit rather than reconstructed
  // afterwards: a post-hoc `waitpid` cannot tell a dead child from a reaped
  // one — with SIGCHLD auto-reaping it returns ECHILD either way — and
  // guessing produced the exact misreport this replaces, blaming an expired
  // PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS on a run where none was set.
  enum class WaitOutcome { Ready, ChildGone, Deadline, PollError };
  WaitOutcome outcome = WaitOutcome::PollError;

  std::string ready_buf;
  bool got_response = false;
  int waited_ms = 0;

  while (true) {
    int ret = poll(&pfd, 1, kPollSliceMs);
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      outcome = WaitOutcome::PollError;
      break;
    }
    if (ret == 0) {
      waited_ms += kPollSliceMs;

      // Liveness is the real bound. `kill(pid, 0)` rather than `waitpid`,
      // because it answers "does this process still exist" whether or not it
      // has been reaped.
      if (kill(child_pid, 0) != 0 && errno == ESRCH) {
        outcome = WaitOutcome::ChildGone;
        break;
      }

      if (kReadyTimeoutMs > 0 && waited_ms >= kReadyTimeoutMs) {
        outcome = WaitOutcome::Deadline;
        break;
      }

      // Say what is happening. A constructor that runs for minutes is
      // indistinguishable from a hang unless something reports it, and that
      // ambiguity is what made the old fixed timeout look reasonable.
      if (waited_ms % (kProgressEverySec * 1000) == 0) {
        RCLCPP_INFO(
          get_logger(), "Component '%s' constructing for %ds (pid %d, alive)",
          request->plugin_name.c_str(), waited_ms / 1000, child_pid);
        // Phase 64: say the same thing to play_launch. This is what makes a
        // slow constructor distinguishable from a wedged one at the
        // supervisor, instead of both looking like a load that has not
        // reported within some timeout.
        if (control_) {
          control_->send_constructing(
            node_id, child_pid, static_cast<uint64_t>(waited_ms), request->plugin_name,
            LoadPhase::Constructing, proc_cpu_ms(child_pid));
        }
      }
      continue;
    }

    char buf[1024];
    ssize_t n = read(pipefd[0], buf, sizeof(buf) - 1);
    if (n <= 0) {
      // EOF: every write end is closed, which for a child that never reported
      // means it is gone. This is the usual way a death is noticed — the poll
      // wakes readable rather than timing out — which is why the liveness
      // check above is not enough on its own.
      outcome = WaitOutcome::ChildGone;
      break;
    }
    buf[n] = '\0';
    ready_buf += buf;
    if (ready_buf.find('\n') != std::string::npos) {
      got_response = true;
      outcome = WaitOutcome::Ready;
      break;
    }
  }
  close(pipefd[0]);

  if (got_response && waited_ms >= kProgressEverySec * 1000) {
    RCLCPP_INFO(
      get_logger(), "Component '%s' finished constructing after %ds", request->plugin_name.c_str(),
      waited_ms / 1000);
  }

  // Parse response
  std::string node_name;
  if (!got_response || ready_buf.empty()) {
    // `outcome` was recorded where the wait ended, so nothing is inferred
    // here. Only signal a child that is still there: once gone, that pid may
    // already belong to something else.
    if (outcome != WaitOutcome::ChildGone) {
      kill(child_pid, SIGKILL);
      waitpid(child_pid, nullptr, 0);
    }
    if (!param_file.empty()) {
      unlink(param_file.c_str());
    }

    // Name which of the two happened. "timeout or crash" merged a node killed
    // at a deadline with one that died in its constructor, and that ambiguity
    // sent the first investigation of issue 0019 down the wrong path.
    switch (outcome) {
      case WaitOutcome::ChildGone:
        throw std::runtime_error(
          "component_node exited during construction after " + std::to_string(waited_ms / 1000) +
          "s (check the component's own stderr in the load_node log directory)");
      case WaitOutcome::Deadline:
        throw std::runtime_error(
          "component_node still constructing after " + std::to_string(waited_ms / 1000) +
          "s when PLAY_LAUNCH_COMPONENT_READY_TIMEOUT_MS expired (it was alive; unset the "
          "variable to wait as long as it stays alive)");
      default:
        throw std::runtime_error("component_node ready pipe failed while waiting for it");
    }
  }

  // Trim trailing newline
  if (!ready_buf.empty() && ready_buf.back() == '\n') {
    ready_buf.pop_back();
  }

  if (ready_buf.substr(0, 3) == "OK ") {
    node_name = ready_buf.substr(3);
  } else if (ready_buf.substr(0, 4) == "ERR ") {
    std::string err_msg = ready_buf.substr(4);
    kill(child_pid, SIGKILL);
    waitpid(child_pid, nullptr, 0);
    if (!param_file.empty()) {
      unlink(param_file.c_str());
    }
    throw std::runtime_error("component_node failed: " + err_msg);
  } else {
    kill(child_pid, SIGKILL);
    waitpid(child_pid, nullptr, 0);
    if (!param_file.empty()) {
      unlink(param_file.c_str());
    }
    throw std::runtime_error("component_node: unexpected response: " + ready_buf);
  }

  // Get pidfd for race-free monitoring (Linux 5.3+)
  int pidfd = static_cast<int>(syscall(SYS_pidfd_open, child_pid, 0));

  // Register pidfd with epoll for crash monitoring
  if (epoll_fd_ >= 0 && pidfd >= 0) {
    epoll_event ev{};
    ev.events = EPOLLIN;
    ev.data.u64 = node_id;
    epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, pidfd, &ev);
  }

  RCLCPP_INFO(
    get_logger(), "Spawned isolated child PID %d for node '%s' (id %lu)", child_pid,
    node_name.c_str(), static_cast<uint64_t>(node_id));

  return ChildInfo{child_pid, pidfd, node_id, node_name, param_file};
}

bool CloneIsolatedComponentManager::plugin_available(
  const std::string & package, const std::string & plugin)
{
  // Look up component resources (ament index, fast).
  //
  // This THROWS for a package with no component resources at all, which is
  // indistinguishable to a caller from a package whose plugin list simply
  // does not contain `plugin` — so report both the same way. On the service
  // path an escaping exception would leave the executor; on the control-socket
  // path it would reach the top of the reader thread. Neither is a reason to
  // lose a container over a misspelled package name.
  try {
    for (const auto & resource : get_component_resources(package)) {
      if (resource.first == plugin) {
        return true;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Failed to list components of package '%s': %s", package.c_str(), ex.what());
  }
  return false;
}

uint64_t CloneIsolatedComponentManager::reserve_node_id(uint64_t seq, const std::string & plugin)
{
  uint64_t node_id;
  {
    std::lock_guard<std::mutex> lock(load_mutex_);
    node_id = unique_id_++;
    if (0 == node_id) {
      throw std::overflow_error("exhausted the unique ids for components in this process");
    }
  }
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    PendingLoad pending;
    pending.seq = seq;
    pending.plugin = plugin;
    pending.phase = LoadPhase::Queued;
    pending.started = std::chrono::steady_clock::now();
    pending_loads_.emplace(node_id, std::move(pending));
    if (seq != 0) {
      seq_to_id_[seq] = node_id;
    }
  }
  return node_id;
}

void CloneIsolatedComponentManager::note_constructing(uint64_t node_id, pid_t pid)
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  auto it = pending_loads_.find(node_id);
  if (it != pending_loads_.end()) {
    it->second.phase = LoadPhase::Constructing;
    it->second.pid = pid;
  }
}

bool CloneIsolatedComponentManager::take_pending(uint64_t node_id, std::string * cancel_reason)
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  auto it = pending_loads_.find(node_id);
  if (it == pending_loads_.end()) {
    return false;
  }
  const bool cancelled = it->second.cancelled;
  if (cancelled && cancel_reason != nullptr) {
    *cancel_reason = it->second.cancel_reason;
  }
  pending_loads_.erase(it);
  return cancelled;
}

void CloneIsolatedComponentManager::submit_spawn(
  uint64_t node_id, const std::shared_ptr<LoadNode::Request> & request)
{
  auto pkg = request->package_name;
  auto plugin = request->plugin_name;

  submit_work([this, request, node_id, pkg, plugin]() {
    // Guard: if shutdown already started, skip spawn entirely.
    if (!rclcpp::ok()) {
      take_pending(node_id, nullptr);
      return;
    }

    try {
      await_spawn_capacity(node_id, plugin);

      // A cancel that arrived while this load sat behind the memory gate is
      // honoured HERE, before the fork — the cheapest place to stop, and the
      // one that makes "cancelled" mean the same thing whether or not a child
      // ever existed.
      {
        std::string reason;
        bool cancelled = false;
        {
          std::lock_guard<std::mutex> lock(pending_mutex_);
          auto it = pending_loads_.find(node_id);
          if (it != pending_loads_.end() && it->second.cancelled) {
            cancelled = true;
            reason = it->second.cancel_reason;
            pending_loads_.erase(it);
          }
        }
        if (cancelled) {
          RCLCPP_INFO(
            get_logger(), "Load of '%s' (id %lu) cancelled before spawn: %s", plugin.c_str(),
            static_cast<uint64_t>(node_id), reason.c_str());
          if (control_) {
            control_->send_load_failed(node_id, "cancelled before spawn: " + reason, true);
          }
          return;
        }
      }

      auto child = spawn_child_process(node_id, request);
      std::string actual_name = child.node_name;
      const int32_t child_pid = static_cast<int32_t>(child.pid);
      const uint64_t child_start_time = proc_start_time(child.pid);

      // Re-check after spawn — SIGTERM may have arrived
      if (!rclcpp::ok()) {
        RCLCPP_DEBUG(
          get_logger(), "Shutdown during spawn of '%s' (id %lu), killing child", plugin.c_str(),
          static_cast<uint64_t>(node_id));
        cleanup_child(child);
        take_pending(node_id, nullptr);
        return;
      }

      {
        std::lock_guard<std::mutex> lock(children_mutex_);
        children_[node_id] = std::move(child);
      }

      take_pending(node_id, nullptr);

      RCLCPP_INFO(
        get_logger(), "Component '%s' loaded as '%s' (id %lu)", plugin.c_str(), actual_name.c_str(),
        static_cast<uint64_t>(node_id));

      // Phase 64: the private channel first — it is ordered, lossless and
      // does not queue behind a startup's worth of DDS discovery, which is
      // the whole reason it exists.
      if (control_) {
        control_->send_loaded(node_id, actual_name, child_pid, child_start_time);
      }

      // Publish LOADED event
      if (rclcpp::ok()) {
        auto event = play_launch_msgs::msg::ComponentEvent();
        event.stamp = now();
        event.event_type = play_launch_msgs::msg::ComponentEvent::LOADED;
        event.unique_id = node_id;
        event.full_node_name = actual_name;
        event.package_name = pkg;
        event.plugin_name = plugin;
        event.pid = child_pid;
        event.start_time = child_start_time;
        event_pub_->publish(event);
      }
    } catch (const std::exception & ex) {
      std::string cancel_reason;
      const bool cancelled = take_pending(node_id, &cancel_reason);

      if (!rclcpp::ok()) {
        RCLCPP_DEBUG(
          get_logger(), "Spawn interrupted by shutdown for '%s' (id %lu)", plugin.c_str(),
          static_cast<uint64_t>(node_id));
        return;
      }

      // A child killed by `handle_control_cancel` dies during construction and
      // surfaces here as an exception. Reporting that as an unexplained death
      // would be a lie by omission — and the supervisor needs the
      // `cancelled` flag, because a confirmed cancellation is the one thing
      // that makes a resend safe.
      const std::string error =
        cancelled ? ("cancelled during construction: " + cancel_reason) : std::string(ex.what());

      if (cancelled) {
        RCLCPP_INFO(
          get_logger(), "Load of '%s' (id %lu) cancelled: %s", plugin.c_str(),
          static_cast<uint64_t>(node_id), cancel_reason.c_str());
      } else {
        RCLCPP_ERROR(
          get_logger(), "Failed to spawn component '%s' (id %lu): %s", plugin.c_str(),
          static_cast<uint64_t>(node_id), ex.what());
      }

      if (control_) {
        control_->send_load_failed(node_id, error, cancelled);
      }

      // Publish LOAD_FAILED event
      auto event = play_launch_msgs::msg::ComponentEvent();
      event.stamp = now();
      event.event_type = play_launch_msgs::msg::ComponentEvent::LOAD_FAILED;
      event.unique_id = node_id;
      event.package_name = pkg;
      event.plugin_name = plugin;
      event.error_message = error;
      event_pub_->publish(event);
    }
  });
}

// ── Non-blocking on_load_node ───────────────────────────────────────────
//
// Phase 1 (synchronous): validate plugin exists, pre-assign unique_id, respond.
// Phase 2 (async worker): fork+exec component_node, read ready pipe, publish event.

void CloneIsolatedComponentManager::on_load_node(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<LoadNode::Request> request, std::shared_ptr<LoadNode::Response> response)
{
  // ── Phase 1: synchronous validation + pre-assign unique_id ──

  if (!plugin_available(request->package_name, request->plugin_name)) {
    response->success = false;
    response->error_message =
      "Failed to find class with the requested plugin name '" + request->plugin_name + "'";
    RCLCPP_ERROR(get_logger(), "%s", response->error_message.c_str());
    return;
  }

  const uint64_t node_id = reserve_node_id(0, request->plugin_name);

  // Approximate full_node_name for the immediate response.
  std::string approx_name;
  if (!request->node_namespace.empty() && request->node_namespace != "/") {
    approx_name = request->node_namespace + "/" + request->node_name;
  } else if (!request->node_name.empty()) {
    approx_name = "/" + request->node_name;
  } else {
    approx_name = request->plugin_name;
  }

  // Respond immediately — node is not yet spawned
  response->success = true;
  response->unique_id = node_id;
  response->full_node_name = approx_name;

  RCLCPP_INFO(
    get_logger(),
    "Accepted load request for '%s' (pre-assigned id %lu), "
    "spawning async...",
    request->plugin_name.c_str(), static_cast<uint64_t>(node_id));

  // ── Phase 2: async spawn on worker thread ──
  submit_spawn(node_id, request);
}

// ── Phase 64: the same load, asked for over the private control channel ──
//
// Identical work, different answer path: `Accepted` carries the pre-assigned
// id the LoadNode response used to carry, and it is written to a socket
// nobody else is contending for. Runs on the channel's reader thread, which
// is safe here for the reason `accepts_socket_loads` gives — phase 1 touches
// only this class's own mutexes, and phase 2 is the worker pool it always was.

void CloneIsolatedComponentManager::handle_control_load(
  uint64_t seq, std::shared_ptr<LoadNode::Request> request)
{
  if (!plugin_available(request->package_name, request->plugin_name)) {
    const std::string error =
      "Failed to find class with the requested plugin name '" + request->plugin_name + "'";
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    if (control_) {
      control_->send_rejected(seq, error);
    }
    return;
  }

  uint64_t node_id = 0;
  try {
    node_id = reserve_node_id(seq, request->plugin_name);
  } catch (const std::exception & ex) {
    if (control_) {
      control_->send_rejected(seq, ex.what());
    }
    return;
  }

  if (control_) {
    control_->send_accepted(seq, node_id);
  }

  RCLCPP_INFO(
    get_logger(), "Accepted control-channel load for '%s' (pre-assigned id %lu), spawning async...",
    request->plugin_name.c_str(), static_cast<uint64_t>(node_id));

  submit_spawn(node_id, request);
}

// ── Phase 64 W2: answering, instead of being guessed at ─────────────────
//
// The supervisor cannot tell a slow constructor from a lost load, and every
// mechanism that tried to infer it from a clock got one of the two wrong. This
// manager knows: it holds the pending map, the child pids and the children
// map. `unknown` here is a positive statement — "nothing is running for this
// id and nothing is queued for it" — and it is the ONLY answer that lets the
// supervisor resend.

void CloneIsolatedComponentManager::handle_control_query(
  std::optional<uint64_t> seq, std::optional<uint64_t> unique_id)
{
  if (!control_) {
    return;
  }

  uint64_t id = unique_id.value_or(0);
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    // Asked by seq (the load was never acknowledged, as far as the supervisor
    // knows): resolve it ourselves rather than reporting `unknown` for a load
    // we did accept.
    if (!unique_id.has_value() && seq.has_value()) {
      auto mapped = seq_to_id_.find(*seq);
      if (mapped != seq_to_id_.end()) {
        id = mapped->second;
      }
    }

    auto pending = pending_loads_.find(id);
    if (id != 0 && pending != pending_loads_.end()) {
      const auto & load = pending->second;
      const uint64_t elapsed_ms = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - load.started)
          .count());
      const uint64_t cpu_ms = load.pid > 0 ? proc_cpu_ms(load.pid) : 0;
      control_->send_status(
        seq, id, load.phase, static_cast<int>(load.pid), elapsed_ms, cpu_ms, load.plugin,
        load.pid > 0, "");
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lock(children_mutex_);
    auto child = children_.find(id);
    if (id != 0 && child != children_.end()) {
      control_->send_status(
        seq, id, LoadPhase::Loaded, static_cast<int>(child->second.pid), 0,
        proc_cpu_ms(child->second.pid), "", true, child->second.node_name);
      return;
    }
  }

  control_->send_status(seq, id, LoadPhase::Unknown, 0, 0, 0, "", false, "");
}

void CloneIsolatedComponentManager::handle_control_cancel(
  uint64_t unique_id, const std::string & reason)
{
  // Three cases, and each must answer: the supervisor treats a confirmed
  // cancellation as "nothing is running for this id", which is what it waits
  // for before resending. A cancel that is silently dropped is a load that
  // never resolves.
  pid_t victim = 0;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    auto pending = pending_loads_.find(unique_id);
    if (pending != pending_loads_.end()) {
      pending->second.cancelled = true;
      pending->second.cancel_reason = reason;
      victim = pending->second.pid;
    }
  }

  if (victim > 0) {
    // Kill the child; the spawn path's liveness check ends the ready-wait,
    // the exception handler unwinds it, and the `load_failed { cancelled }`
    // is sent from there — one reporting path, whether the kill lands before
    // or after the constructor would have finished.
    RCLCPP_WARN(
      get_logger(), "Cancelling load %lu: killing pid %d (%s)",
      static_cast<uint64_t>(unique_id), static_cast<int>(victim), reason.c_str());
    kill(victim, SIGKILL);
    return;
  }
  if (victim == 0) {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_loads_.count(unique_id) > 0) {
      // Queued, not yet forked: the worker will see the flag and report.
      RCLCPP_INFO(
        get_logger(), "Load %lu cancelled while queued (%s)", static_cast<uint64_t>(unique_id),
        reason.c_str());
      return;
    }
  }

  // Already finished, or never existed. Answer with what IS true rather than
  // with a cancellation that did not happen: an already-loaded component is
  // unloaded, not cancelled, and the supervisor reconciles from the status.
  handle_control_query(std::nullopt, unique_id);
}
// ── on_unload_node override ─────────────────────────────────────────────

void CloneIsolatedComponentManager::on_unload_node(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<UnloadNode::Request> request,
  std::shared_ptr<UnloadNode::Response> response)
{
  // Reject unload if node is still being constructed
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_loads_.count(request->unique_id) > 0) {
      response->success = false;
      response->error_message =
        "Node with unique_id " + std::to_string(request->unique_id) + " is still being constructed";
      RCLCPP_WARN(get_logger(), "%s", response->error_message.c_str());
      return;
    }
  }

  std::string full_name;
  {
    std::lock_guard<std::mutex> lock(children_mutex_);
    auto it = children_.find(request->unique_id);
    if (it == children_.end()) {
      response->success = false;
      response->error_message = "No node with unique_id " + std::to_string(request->unique_id);
      return;
    }

    full_name = it->second.node_name;

    // Deregister pidfd from epoll BEFORE killing
    if (epoll_fd_ >= 0 && it->second.pidfd >= 0) {
      epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, it->second.pidfd, nullptr);
    }

    cleanup_child(it->second);
    children_.erase(it);
  }

  response->success = true;

  RCLCPP_INFO(
    get_logger(), "Unloaded node '%s' (id %lu)", full_name.c_str(),
    static_cast<uint64_t>(request->unique_id));

  if (control_) {
    control_->send_unloaded(request->unique_id, full_name);
  }

  // Publish UNLOADED event
  if (rclcpp::ok()) {
    auto event = play_launch_msgs::msg::ComponentEvent();
    event.stamp = now();
    event.event_type = play_launch_msgs::msg::ComponentEvent::UNLOADED;
    event.unique_id = request->unique_id;
    event.full_node_name = full_name;
    event_pub_->publish(event);
  }
}

// ── on_list_nodes override (thread-safe, excludes pending) ──────────────

void CloneIsolatedComponentManager::on_list_nodes(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<ListNodes::Request> /*request*/,
  std::shared_ptr<ListNodes::Response> response)
{
  std::lock_guard<std::mutex> lock1(children_mutex_);
  std::lock_guard<std::mutex> lock2(pending_mutex_);

  for (const auto & [id, child] : children_) {
    // Skip nodes still being constructed
    if (pending_loads_.count(id) > 0) {
      continue;
    }
    response->unique_ids.push_back(id);
    response->full_node_names.push_back(child.node_name);
  }
}

// ── add_node_to_executor ────────────────────────────────────────────────
//
// Not used in fork+exec mode (children manage their own executors).
// Kept as a no-op to satisfy the override requirement.

void CloneIsolatedComponentManager::add_node_to_executor(uint64_t /*node_id*/)
{
  // No-op: fork+exec children manage their own executor.
}

// ── remove_node_from_executor ───────────────────────────────────────────
//
// Not used in fork+exec mode (children are killed directly).
// Kept as a no-op to satisfy the override requirement.

void CloneIsolatedComponentManager::remove_node_from_executor(uint64_t /*node_id*/)
{
  // No-op: fork+exec children are killed directly in on_unload_node.
}

// ── monitor_loop — epoll on pidfds ──────────────────────────────────────
//
// Blocks on epoll_wait until a child pidfd becomes readable (child exited)
// or stop_fd_ is signalled (shutdown).  Zero CPU when idle.

void CloneIsolatedComponentManager::monitor_loop()
{
  constexpr int kMaxEvents = 8;
  epoll_event events[kMaxEvents];

  while (monitor_running_) {
    int n = epoll_wait(epoll_fd_, events, kMaxEvents, -1);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    for (int i = 0; i < n; ++i) {
      if (events[i].data.u64 == 0) {
        // Shutdown signal — drain eventfd and exit
        uint64_t val;
        if (read(stop_fd_, &val, sizeof(val)) < 0) {
          // best-effort drain — ignore errors
        }
        continue;
      }
      handle_child_death(events[i].data.u64);
    }
  }
}

// ── handle_child_death — cleanup crashed child + publish event ──────────

void CloneIsolatedComponentManager::handle_child_death(uint64_t node_id)
{
  std::lock_guard<std::mutex> lock(children_mutex_);
  auto it = children_.find(node_id);
  if (it == children_.end()) {
    // Graceful unload already handled this child
    return;
  }

  auto & child = it->second;

  // Reap child and get exit details
  int status = 0;
  waitpid(child.pid, &status, WNOHANG);

  std::string error_msg;
  if (WIFSIGNALED(status)) {
    error_msg = "killed by signal " + std::to_string(WTERMSIG(status)) + " (" +
                std::string(strsignal(WTERMSIG(status))) + ")";
#ifdef WCOREDUMP
    if (WCOREDUMP(status)) {
      error_msg += " (core dumped)";
    }
#endif
  } else if (WIFEXITED(status)) {
    error_msg = "exited with status " + std::to_string(WEXITSTATUS(status));
  } else {
    error_msg = "died (unknown cause)";
  }

  RCLCPP_ERROR(
    get_logger(), "Child PID %d crashed for node '%s': %s", child.pid, child.node_name.c_str(),
    error_msg.c_str());

  if (control_) {
    control_->send_crashed(node_id, child.node_name, error_msg, static_cast<int>(child.pid));
  }

  // Publish CRASHED event
  try {
    auto event = play_launch_msgs::msg::ComponentEvent();
    event.stamp = now();
    event.event_type = play_launch_msgs::msg::ComponentEvent::CRASHED;
    event.unique_id = node_id;
    event.full_node_name = child.node_name;
    event.error_message = error_msg;
    event.pid = static_cast<int32_t>(child.pid);
    event_pub_->publish(event);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "Failed to publish CRASHED event: %s", ex.what());
  }

  // Clean up resources
  if (child.pidfd >= 0) {
    close(child.pidfd);
  }
  if (!child.param_file.empty()) {
    unlink(child.param_file.c_str());
  }

  children_.erase(it);
}

// ── cleanup_child (destructor / unload helper) ──────────────────────────

void CloneIsolatedComponentManager::cleanup_child(ChildInfo & child)
{
  // Graceful shutdown: SIGTERM, wait, then SIGKILL
  kill(child.pid, SIGTERM);
  int status = 0;
  for (int i = 0; i < 50; ++i) {
    if (waitpid(child.pid, &status, WNOHANG) != 0) {
      break;
    }
    usleep(10000);  // 10ms, up to 500ms total
  }
  if (waitpid(child.pid, &status, WNOHANG) == 0) {
    kill(child.pid, SIGKILL);
    waitpid(child.pid, &status, 0);
  }

  if (child.pidfd >= 0) {
    close(child.pidfd);
  }
  if (!child.param_file.empty()) {
    unlink(child.param_file.c_str());
  }
}

}  // namespace play_launch_container
