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

#ifndef PLAY_LAUNCH_CONTAINER__CLONE_ISOLATED_COMPONENT_MANAGER_HPP_
#define PLAY_LAUNCH_CONTAINER__CLONE_ISOLATED_COMPONENT_MANAGER_HPP_

#include <sys/types.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "play_launch_container/observable_component_manager.hpp"

namespace play_launch_container
{

/// Component manager that isolates each composable node in its own Linux process
/// via fork()+exec() of the `component_node` binary.
///
/// Each node runs as a fully independent process with its own address space,
/// glibc, TLS, and DDS participant.  This avoids all TLS/glibc issues that
/// plague clone(CLONE_VM) and supports full MultiThreadedExecutor.
///
/// Class hierarchy:
///   ComponentManager -> ObservableComponentManager -> CloneIsolatedComponentManager
///
/// Inherits event publishing from ObservableComponentManager.
/// Overrides add_node_to_executor / remove_node_from_executor (same pattern as
/// upstream ComponentManagerIsolated, but fork+exec instead of std::thread).
class CloneIsolatedComponentManager : public ObservableComponentManager
{
public:
  CloneIsolatedComponentManager(
    std::weak_ptr<rclcpp::Executor> executor =
      std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
    bool use_multi_threaded = false, std::string node_name = "ComponentManager",
    const rclcpp::NodeOptions & node_options =
      rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));

  ~CloneIsolatedComponentManager() override;

  /// Phase 64: yes. Loading here already happens off the executor, on the
  /// worker pool, so a request arriving on the control channel's reader
  /// thread runs exactly the path a LoadNode request would have run — minus
  /// the service call, which during a 150-process startup is the expensive
  /// part.
  bool accepts_socket_loads() const override { return true; }

protected:
  void handle_control_load(
    uint64_t seq, std::shared_ptr<LoadNode::Request> request) override;

  void handle_control_query(
    std::optional<uint64_t> seq, std::optional<uint64_t> unique_id) override;

  void handle_control_cancel(uint64_t unique_id, const std::string & reason) override;

  void on_load_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<LoadNode::Request> request,
    std::shared_ptr<LoadNode::Response> response) override;

  void on_unload_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<UnloadNode::Request> request,
    std::shared_ptr<UnloadNode::Response> response) override;

  void on_list_nodes(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<ListNodes::Request> request,
    std::shared_ptr<ListNodes::Response> response) override;

  void add_node_to_executor(uint64_t node_id) override;
  void remove_node_from_executor(uint64_t node_id) override;

private:
  /// Whether `plugin` is registered by `package` (ament index lookup).
  bool plugin_available(const std::string & package, const std::string & plugin);

  /// Reserve the next unique_id and record it as pending. Shared by the
  /// service and control-socket entry points so an id means the same thing on
  /// both; `seq` is 0 for a load that did not come from the socket.
  uint64_t reserve_node_id(uint64_t seq, const std::string & plugin);

  /// Queue the fork+exec of `request` on the worker pool and report the
  /// outcome on every channel that is listening. This is phase 2 of a load;
  /// the two entry points differ only in how they answer phase 1.
  void submit_spawn(uint64_t node_id, const std::shared_ptr<LoadNode::Request> & request);

  void worker_loop();
  void submit_work(std::function<void()> work);

  struct ChildInfo
  {
    pid_t pid;
    int pidfd;
    uint64_t node_id;
    std::string node_name;
    std::string param_file;  // temp YAML to unlink on cleanup
  };

  /// Serialize LoadNode::Request parameters to a temp YAML file.
  /// Returns empty string if no parameters.
  std::string write_params_file(const std::shared_ptr<LoadNode::Request> & request);

  /// Fork+exec component_node for the given load request.
  /// Returns ChildInfo on success, throws on failure.
  ChildInfo spawn_child_process(
    uint64_t node_id, const std::shared_ptr<LoadNode::Request> & request);

  void cleanup_child(ChildInfo & child);
  void monitor_loop();
  void handle_child_death(uint64_t node_id);

  bool use_multi_threaded_;
  std::string component_node_path_;  // resolved once in constructor

  // Thread safety for inherited protected members (node_wrappers_, loaders_, unique_id_)
  std::mutex load_mutex_;

  // Worker thread pool for async node construction
  std::vector<std::thread> worker_threads_;
  std::mutex work_queue_mutex_;
  std::condition_variable work_queue_cv_;
  std::queue<std::function<void()>> work_queue_;
  std::atomic<bool> workers_running_{false};
  /// Serialises spawn admission so the memory check and the fork it guards
  /// are not raced by every other worker (see `await_spawn_capacity`).
  std::mutex spawn_admit_mutex_;

  /// Block until there is memory headroom for another child, or until the
  /// wait cap expires. This is the governor for spawn concurrency: the worker
  /// pool is deliberately sized not to be the limit, because a count cannot
  /// tell a component that is slow because it is computing from one that is
  /// slow because it is waiting.
  ///
  /// Reports `phase: queued` to the supervisor while it waits: this gate can
  /// hold a fork for up to two minutes, and before phase 64 W2 that window was
  /// silent — the load had been accepted and nothing had been forked, so there
  /// was no pid to be alive and no report to make.
  void await_spawn_capacity(uint64_t node_id, const std::string & plugin);

  /// Mark a pending load as forked and note its pid, so a query can answer
  /// `constructing` with something concrete.
  void note_constructing(uint64_t node_id, pid_t pid);

  /// Drop a load from the pending map. Returns whether it had been cancelled,
  /// and by what reason, so the spawn path can report a kill as a
  /// cancellation rather than as an unexplained death.
  bool take_pending(uint64_t node_id, std::string * cancel_reason);

  /// A load that has been accepted but has not finished, and everything the
  /// supervisor could otherwise only guess at.
  ///
  /// Phase 64 W2: this used to be a bare `std::set<uint64_t>` of ids, which
  /// could answer "is it pending" and nothing else — so a composable waiting
  /// on the memory gate and one whose constructor had been running for three
  /// minutes were the same state to anybody asking, and neither could be
  /// distinguished from a load that had been lost. The supervisor's whole
  /// retry policy turns on that distinction.
  struct PendingLoad
  {
    uint64_t seq = 0;
    std::string plugin;
    LoadPhase phase = LoadPhase::Queued;
    pid_t pid = 0;
    std::chrono::steady_clock::time_point started;
    /// Set by `handle_control_cancel`; the worker checks it before forking and
    /// the spawn path reports the failure as a cancellation rather than as a
    /// death it could not explain.
    bool cancelled = false;
    std::string cancel_reason;
  };

  // Loads with a unique_id assigned that have not reached a terminal state.
  std::map<uint64_t, PendingLoad> pending_loads_;
  /// seq -> unique_id, so a query asked before the supervisor saw `accepted`
  /// can still be answered. Kept for the container's life: it is 16 bytes per
  /// load and answering "unknown" to a load that exists is the one mistake
  /// this whole mechanism is built to avoid.
  std::map<uint64_t, uint64_t> seq_to_id_;
  std::mutex pending_mutex_;

  std::mutex children_mutex_;
  std::map<uint64_t, ChildInfo> children_;

  int epoll_fd_ = -1;
  int stop_fd_ = -1;
  std::thread monitor_thread_;
  std::atomic<bool> monitor_running_{false};
};

}  // namespace play_launch_container

#endif  // PLAY_LAUNCH_CONTAINER__CLONE_ISOLATED_COMPONENT_MANAGER_HPP_
