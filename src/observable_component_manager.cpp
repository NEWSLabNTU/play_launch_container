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

#include "play_launch_container/observable_component_manager.hpp"

#include <utility>

namespace play_launch_container
{

ObservableComponentManager::ObservableComponentManager(
  std::weak_ptr<rclcpp::Executor> executor, std::string node_name,
  const rclcpp::NodeOptions & node_options)
: ComponentManager(executor, std::move(node_name), node_options)
{
  // Transient local: late subscribers (e.g. play_launch restarts) get history
  auto qos = rclcpp::QoS(100).reliable().transient_local();
  event_pub_ =
    create_publisher<play_launch_msgs::msg::ComponentEvent>("~/_container/component_events", qos);
}

ObservableComponentManager::~ObservableComponentManager()
{
  // Stop the channel before anything else is torn down: its reader thread
  // calls back into this object, and `stop()` joins that thread.
  if (control_) {
    control_->stop();
  }
}

void ObservableComponentManager::set_control_channel(std::shared_ptr<ControlChannel> channel)
{
  control_ = std::move(channel);
  if (!control_) {
    return;
  }
  control_->set_handlers(
    [this](uint64_t seq, std::shared_ptr<LoadNode::Request> request) {
      this->handle_control_load(seq, std::move(request));
    },
    [this](std::optional<uint64_t> seq, std::optional<uint64_t> unique_id) {
      this->handle_control_query(seq, unique_id);
    },
    [this](uint64_t unique_id, const std::string & reason) {
      this->handle_control_cancel(unique_id, reason);
    });
}

void ObservableComponentManager::handle_control_load(
  uint64_t seq, std::shared_ptr<LoadNode::Request> request)
{
  RCLCPP_WARN(
    get_logger(),
    "Refusing socket load of '%s': this container answers LoadNode requests on its executor",
    request->plugin_name.c_str());
  if (control_) {
    control_->send_rejected(seq, "this container does not accept loads over the control socket");
  }
}

void ObservableComponentManager::handle_control_query(
  std::optional<uint64_t> seq, std::optional<uint64_t> unique_id)
{
  if (!control_) {
    return;
  }
  const uint64_t id = unique_id.value_or(0);
  auto it = node_wrappers_.find(id);
  if (id != 0 && it != node_wrappers_.end()) {
    control_->send_status(
      seq, id, LoadPhase::Loaded, 0, 0, 0, "", false,
      it->second.get_node_base_interface()->get_fully_qualified_name());
    return;
  }
  // Unknown, and honestly so: this manager constructs on the executor, so
  // there is no in-flight state it could be hiding. A load it is still
  // running would not be answering this query at all.
  control_->send_status(seq, id, LoadPhase::Unknown, 0, 0, 0, "", false, "");
}

void ObservableComponentManager::handle_control_cancel(
  uint64_t unique_id, const std::string & reason)
{
  RCLCPP_WARN(
    get_logger(), "Cannot cancel load %lu (%s): this container loads components as threads",
    static_cast<uint64_t>(unique_id), reason.c_str());
  if (control_) {
    // Answer anyway. The supervisor waits for a confirmation before it will
    // consider a resend, and a cancel that is never answered is a load that
    // is stuck forever — worse than one that is refused.
    control_->send_load_failed(
      unique_id, "cancel refused: components here are threads, not processes", false);
  }
}

void ObservableComponentManager::on_load_node(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<LoadNode::Request> request, std::shared_ptr<LoadNode::Response> response)
{
  // Parent does the actual loading (synchronous, response populated on return)
  ComponentManager::on_load_node(request_header, request, response);

  // Publish event with the result
  auto event = play_launch_msgs::msg::ComponentEvent();
  event.stamp = now();
  event.package_name = request->package_name;
  event.plugin_name = request->plugin_name;

  if (response->success) {
    event.event_type = play_launch_msgs::msg::ComponentEvent::LOADED;
    event.unique_id = response->unique_id;
    event.full_node_name = response->full_node_name;
  } else {
    event.event_type = play_launch_msgs::msg::ComponentEvent::LOAD_FAILED;
    event.error_message = response->error_message;
  }
  // Guard against shutdown race (ros2/rclcpp#812)
  if (rclcpp::ok()) {
    event_pub_->publish(event);
  }

  // Phase 64: mirror the outcome onto the private channel. The supervisor
  // treats whichever arrives first as authoritative and the other as a
  // no-op, so this costs a few hundred bytes and removes a dependency on a
  // DDS topic surviving a startup storm.
  if (control_) {
    if (response->success) {
      control_->send_loaded(response->unique_id, response->full_node_name, 0, 0);
    } else {
      control_->send_load_failed(response->unique_id, response->error_message);
    }
  }
}

void ObservableComponentManager::on_unload_node(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<UnloadNode::Request> request,
  std::shared_ptr<UnloadNode::Response> response)
{
  // Capture node name BEFORE parent erases it from node_wrappers_
  std::string full_name;
  auto it = node_wrappers_.find(request->unique_id);
  if (it != node_wrappers_.end()) {
    full_name = it->second.get_node_base_interface()->get_fully_qualified_name();
  }

  // Parent does the actual unloading (erases from node_wrappers_)
  ComponentManager::on_unload_node(request_header, request, response);

  // Guard against shutdown race (ros2/rclcpp#812)
  if (response->success && rclcpp::ok()) {
    auto event = play_launch_msgs::msg::ComponentEvent();
    event.stamp = now();
    event.event_type = play_launch_msgs::msg::ComponentEvent::UNLOADED;
    event.unique_id = request->unique_id;
    event.full_node_name = full_name;
    event_pub_->publish(event);
  }

  if (response->success && control_) {
    control_->send_unloaded(request->unique_id, full_name);
  }
}

}  // namespace play_launch_container
