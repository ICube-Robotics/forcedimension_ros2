// Copyright 2022, ICube Laboratory, University of Strasbourg
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


#include <Eigen/Dense>

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "fd_clutch_broadcaster/fd_clutch_broadcaster.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"


namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace fd_clutch_broadcaster
{

FdClutchBroadcaster::FdClutchBroadcaster() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FdClutchBroadcaster::on_init()
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FdClutchBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration FdClutchBroadcaster::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (clutch_interface_name_.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "No clutch interface name provided!");

  } else {
    state_interfaces_config.names.push_back(clutch_interface_name_);
  }
  return state_interfaces_config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FdClutchBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Declare parameters
  try {
    auto_declare<std::string>("clutch_interface_name", std::string("button/position"));
    auto_declare<bool>("is_interface_a_button", true);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get interface name from parameters
  clutch_interface_name_ = get_node()->get_parameter("clutch_interface_name").as_string();
  if (clutch_interface_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Please provide the clutch interface name!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  try {
    clutch_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      "fd_clutch",
      rclcpp::SystemDefaultsQoS());
    realtime_clutch_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
      clutch_publisher_);
  } catch (const std::exception & e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FdClutchBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (state_interfaces_.size() != 1) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Expecting exactly one state interface.");
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FdClutchBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FdClutchBroadcaster::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Entering update()");
  if (realtime_clutch_publisher_ && realtime_clutch_publisher_->trylock()) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Lock acquired");
    // Read provided state interface
    bool is_interface_a_button = get_node()->get_parameter("is_interface_a_button").as_bool();
    double read_value = state_interfaces_[0].get_value();

    bool clutch_state = false;
    if (is_interface_a_button) {
      // Clutched (workspace engaged) if button is pressed
      clutch_state = (read_value > 0.5) ? true : false;
    } else {
      // Clutched (workspace engaged) if handle angle is small (i.e., physically clutched)
      // E.g., 7th "joint" of Omega 6 / Sigma 7
      clutch_state = (read_value < 0.03) ? true : false;
    }

    // Publish clucth
    auto & fd_clutch_msg = realtime_clutch_publisher_->msg_;
    fd_clutch_msg.data = clutch_state;
    RCLCPP_DEBUG(get_node()->get_logger(), "publish and unlock");
    realtime_clutch_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace fd_clutch_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fd_clutch_broadcaster::FdClutchBroadcaster, controller_interface::ControllerInterface)
