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

#ifndef PLAY_LAUNCH_CONTAINER__CLONE_VM_COMPONENT_MANAGER_HPP_
#define PLAY_LAUNCH_CONTAINER__CLONE_VM_COMPONENT_MANAGER_HPP_

#include <sys/types.h>

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "play_launch_container/observable_component_manager.hpp"

namespace play_launch_container
{

/// EXPERIMENTAL. Component manager that runs each composable node in a
/// `clone(CLONE_VM)` child: its own PID and signal disposition, sharing the
/// container's address space.
///
/// This is the middle point the other two managers do not occupy:
///
/// | | Observable (thread) | CloneIsolated (fork+exec) | CloneVm |
/// |---|---|---|---|
/// | SIGSEGV kills only that node | no | yes | **yes** |
/// | zero-copy intra-process | yes | no | **yes** |
/// | DDS participants | 1 per container | 1 per node | **1 per container** |
/// | per-node PID / cgroup / kill | no | yes | **yes** |
/// | per-node memory limit | no | yes | no |
///
/// The per-process floor a separate participant costs is not small: on a
/// 12-core AGX Orin a composable that does nothing at all draws ~4.9% of a core
/// once it is its own process, and 84 of them loaded as threads drew 153.7%
/// between them.
///
/// ## Why this is hidden, and what it is not
///
/// It is selected by `--container-mode clone-vm`, which clap does not print.
/// Three things are measured and one large one is not:
///
///  * All three shipped RMWs — `rmw_fastrtps_cpp`, `rmw_cyclonedds_cpp` and
///    `rmw_zenoh_cpp` — run a node to completion in a clone child, including a
///    second node loaded while the first is spinning, and shut down clean.
///    Cyclone was refused here for a while on the strength of a crash inside
///    `dds_take`; that crash was this manager's own wrong thread pointer, not
///    the backend's fault. See `add_node_to_executor`.
///  * A SIGSEGV in one child kills only that child, but ONLY because the child
///    is made undumpable first — see `child_main`.
///  * A child may only be stopped by `executor->cancel()`. A signal leaves
///    whatever rclcpp or DDS mutex it held locked forever in the address space
///    the parent shares, and the container then deadlocks in shutdown.
///  * NOT measured: what happens when a node actually crashes. The SIGSEGV
///    boundary is the reason to want this, and mutex poisoning from a real
///    crash is exactly the case `cancel()` cannot help with.
///
/// See `experiments/clone-vm-rmw/README.md` for the measurements and
/// `docs/archive/clone-vm-container-design.md` for the risk enumeration.
///
/// Class hierarchy:
///   ComponentManager -> ObservableComponentManager -> CloneVmComponentManager
class CloneVmComponentManager : public ObservableComponentManager
{
public:
  explicit CloneVmComponentManager(
    std::weak_ptr<rclcpp::Executor> executor =
      std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
    std::string node_name = "ComponentManager",
    const rclcpp::NodeOptions & node_options =
      rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));

  ~CloneVmComponentManager() override;

  /// True when this manager can run on the RMW currently loaded.
  ///
  /// Refuses nothing today — all three shipped backends are measured working.
  /// Kept because the next backend tried here deserves the same
  /// "measured / not measured" answer, reported through `reason_out`.
  static bool rmw_is_supported(std::string * reason_out);

protected:
  /// Give the node its own executor and spin it in a clone child.
  void add_node_to_executor(uint64_t node_id) override;

  /// Cancel the child's executor and reap it. Never signals it first.
  void remove_node_from_executor(uint64_t node_id) override;

private:
  struct Child
  {
    pid_t pid = -1;
    std::shared_ptr<rclcpp::Executor> executor;
    void * stack = nullptr;
    std::size_t stack_size = 0;
    void * tls = nullptr;
  };

  /// Stop one child cooperatively and release its stack and TLS.
  /// Returns false if it did not exit within the grace period, in which case
  /// its resources are deliberately LEAKED: the child is still running in this
  /// address space, and unmapping its stack underneath it is worse than a leak.
  bool stop_child(uint64_t node_id, Child & child);

  std::mutex mutex_;
  std::map<uint64_t, Child> children_;
};

}  // namespace play_launch_container

#endif  // PLAY_LAUNCH_CONTAINER__CLONE_VM_COMPONENT_MANAGER_HPP_
