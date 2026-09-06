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

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "play_launch_container/clone_isolated_component_manager.hpp"
#include "play_launch_container/clone_vm_component_manager.hpp"
#include "play_launch_container/control_channel.hpp"
#include "play_launch_container/observable_component_manager.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  /// Observable component container with configurable executor and isolation.
  ///
  /// Usage:
  ///   component_container                              # single-threaded (default)
  ///   component_container --use_multi_threaded_executor # multi-threaded
  ///   component_container --isolated                   # fork+exec per-node
  ///   component_container --isolated --use_multi_threaded_executor
  ///   component_container --clone-vm                   # EXPERIMENTAL, hidden:
  ///                                                    # clone(CLONE_VM) per-node
  // Phase 64: scan argv and answer play_launch BEFORE rclcpp::init.
  //
  // The hello is what tells the supervisor which transport its loads take, and
  // it holds its first load until the answer arrives. `rclcpp::init` performs
  // DDS setup, which under a 150-process startup is exactly the delay this
  // channel exists to route around, so the answer must not wait for it.
  bool use_multi_threaded = false;
  bool use_isolated = false;
  bool use_clone_vm = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--use_multi_threaded_executor") {
      use_multi_threaded = true;
    } else if (arg == "--isolated") {
      use_isolated = true;
    } else if (arg == "--clone-vm") {
      use_clone_vm = true;
    }
  }

  // Mutually exclusive: both claim add_node_to_executor, and a container that
  // silently picked one would be indistinguishable from the other in the logs.
  if (use_isolated && use_clone_vm) {
    std::fprintf(stderr, "component_container: --isolated and --clone-vm are exclusive\n");
    return 2;
  }

  auto control = play_launch_container::ControlChannel::from_env();
  if (control) {
    // Only the isolated manager takes loads over the socket; see
    // ObservableComponentManager::accepts_socket_loads. clone-vm loads on the
    // executor like the observable manager does, so it answers false.
    control->send_hello(use_isolated);
  }

  rclcpp::init(argc, argv);

  // After init, because the answer depends on which rmw actually loaded, and
  // before any node exists, because the failure this catches is a segfault in
  // a child on its first message rather than an error anyone could act on.
  if (use_clone_vm) {
    std::string reason;
    const bool supported =
      play_launch_container::CloneVmComponentManager::rmw_is_supported(&reason);
    if (!supported) {
      std::fprintf(stderr, "component_container: --clone-vm refused: %s\n", reason.c_str());
      rclcpp::shutdown();
      return 3;
    }
    if (!reason.empty()) {
      std::fprintf(stderr, "component_container: --clone-vm warning: %s\n", reason.c_str());
    }
  }

  // Create executor
  std::shared_ptr<rclcpp::Executor> exec;
  if (use_multi_threaded) {
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  } else {
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  // Create manager (CloneIsolated inherits from Observable)
  std::shared_ptr<play_launch_container::ObservableComponentManager> node;
  if (use_isolated) {
    node = std::make_shared<play_launch_container::CloneIsolatedComponentManager>(
      exec, use_multi_threaded);
  } else if (use_clone_vm) {
    // Every clone child spins its own SingleThreadedExecutor regardless of
    // use_multi_threaded: the container's executor choice governs the manager's
    // own services, not the children.
    node = std::make_shared<play_launch_container::CloneVmComponentManager>(exec);
  } else {
    node = std::make_shared<play_launch_container::ObservableComponentManager>(exec);
  }

  // Hand the manager the channel. Loads sent between the hello and this point
  // were queued by the channel and are replayed here, so the supervisor never
  // has to wait for the manager to exist before asking.
  node->set_control_channel(control);

  // Handle thread_num parameter for MT mode
  if (use_multi_threaded && node->has_parameter("thread_num")) {
    const auto thread_num = node->get_parameter("thread_num").as_int();
    exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions{}, thread_num);
    node->set_executor(exec);
  }

  exec->add_node(node);
  exec->spin();

  // Explicit cleanup order: remove the manager from the parent executor,
  // destroy it (which kills any isolated child processes), then shutdown.
  // Without this, auto-destruction order can trigger "Node needs to be
  // associated with an executor" during rclcpp::shutdown().
  exec->remove_node(node);
  node.reset();
  rclcpp::shutdown();
}
