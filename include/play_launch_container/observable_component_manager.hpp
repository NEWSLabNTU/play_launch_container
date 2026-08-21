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

#ifndef PLAY_LAUNCH_CONTAINER__OBSERVABLE_COMPONENT_MANAGER_HPP_
#define PLAY_LAUNCH_CONTAINER__OBSERVABLE_COMPONENT_MANAGER_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "play_launch_container/control_channel.hpp"
#include "play_launch_msgs/msg/component_event.hpp"
#include "rclcpp_components/component_manager.hpp"

namespace play_launch_container
{

/// Component manager that publishes load/unload events on a topic.
///
/// Subclasses rclcpp_components::ComponentManager (the same pattern used by
/// ComponentManagerIsolated in rclcpp_components) and overrides on_load_node
/// and on_unload_node to publish ComponentEvent messages after each operation.
///
/// Topic: ~/_container/component_events
/// QoS:   reliable, transient_local, depth 100
class ObservableComponentManager : public rclcpp_components::ComponentManager
{
public:
  ObservableComponentManager(
    std::weak_ptr<rclcpp::Executor> executor =
      std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
    std::string node_name = "ComponentManager",
    const rclcpp::NodeOptions & node_options =
      rclcpp::NodeOptions().start_parameter_services(false).start_parameter_event_publisher(false));

  ~ObservableComponentManager() override;

  /// Phase 64: adopt the private control channel to play_launch.
  ///
  /// Status for every load, unload and crash is mirrored onto it, and load
  /// REQUESTS arrive over it for managers that accept them (see
  /// `accepts_socket_loads`).
  void set_control_channel(std::shared_ptr<ControlChannel> channel);

  /// Whether this manager takes load requests over the control socket.
  ///
  /// False here: `ComponentManager::on_load_node` creates the node and adds it
  /// to the executor, and running that from the channel's reader thread would
  /// race the manager's own bookkeeping with the executor that is using it.
  /// The isolated manager overrides this — its loading already happens off the
  /// executor, on a worker pool.
  virtual bool accepts_socket_loads() const { return false; }

protected:
  /// Handle one `load` frame. The base implementation refuses, because the
  /// supervisor is told `loads_over_socket: false` and should not have sent
  /// one; answering keeps a mistake visible instead of silent.
  virtual void handle_control_load(
    uint64_t seq, std::shared_ptr<LoadNode::Request> request);

  /// Answer "what is the state of this load?".
  ///
  /// The base manager loads on its executor and keeps no per-load bookkeeping
  /// of its own, so it answers from `node_wrappers_`: a known id is `loaded`,
  /// anything else is `unknown`. That is honest here — this manager never has
  /// a load in flight that it could not already report, because
  /// `on_load_node` returns only when the node is constructed.
  virtual void handle_control_query(
    std::optional<uint64_t> seq, std::optional<uint64_t> unique_id);

  /// Stop a load. Not supported here: a thread-loaded component cannot be
  /// killed without taking its container's siblings with it, so this reports
  /// the refusal rather than pretending to have acted.
  virtual void handle_control_cancel(uint64_t unique_id, const std::string & reason);

  void on_load_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<LoadNode::Request> request,
    std::shared_ptr<LoadNode::Response> response) override;

  void on_unload_node(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<UnloadNode::Request> request,
    std::shared_ptr<UnloadNode::Response> response) override;

protected:
  rclcpp::Publisher<play_launch_msgs::msg::ComponentEvent>::SharedPtr event_pub_;
  /// Phase 64: null unless play_launch handed this process a control fd.
  std::shared_ptr<ControlChannel> control_;
};

}  // namespace play_launch_container

#endif  // PLAY_LAUNCH_CONTAINER__OBSERVABLE_COMPONENT_MANAGER_HPP_
