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

#include "ee_pose_broadcaster/ee_pose_broadcaster.hpp"

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

namespace ee_pose_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

EePoseBroadcaster::EePoseBroadcaster() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EePoseBroadcaster::on_init()
{
  try {
    // definition of the parameters that need to be queried from the
    // controller configuration file with default values
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("buttons", std::vector<std::string>());
    auto_declare<std::vector<double>>("transform_translation", std::vector<double>());
    auto_declare<std::vector<double>>("transform_rotation", std::vector<double>());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
EePoseBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration EePoseBroadcaster::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (joints_.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "No joint name provided!");

  } else {
    for (const auto & joint : joints_) {
      state_interfaces_config.names.push_back(joint + "/" + HW_IF_POSITION);
    }
  }

  if (buttons_.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "No button name provided!");

  } else {
    for (const auto & button : buttons_) {
      state_interfaces_config.names.push_back(button + "/" + HW_IF_POSITION);
    }
  }

  return state_interfaces_config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EePoseBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  buttons_ = get_node()->get_parameter("buttons").as_string_array();
  joints_ = get_node()->get_parameter("joints").as_string_array();
  if (joints_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Please provide list of joints in config!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  auto transform_trans_param = get_node()->get_parameter("transform_translation").as_double_array();
  auto transform_rot_param = get_node()->get_parameter("transform_rotation").as_double_array();
  Eigen::Quaternion<double> q;
  Eigen::Vector3d trans;

  if (transform_trans_param.size() == 0) {
    trans << 0.0, 0.0, 0.0;
  } else if (transform_trans_param.size() == 3) {
    trans << transform_trans_param[0], transform_trans_param[1], transform_trans_param[2];
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Wrong translation format");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (transform_rot_param.size() == 0) {
    q = Eigen::Quaternion<double>(1, 0, 0, 0);
  } else if (transform_rot_param.size() == 3) {
    Eigen::AngleAxisd rollAngle(transform_rot_param[0], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(transform_rot_param[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(transform_rot_param[2], Eigen::Vector3d::UnitX());
    q = rollAngle * yawAngle * pitchAngle;
  } else if (transform_rot_param.size() == 4) {
    q = Eigen::Quaternion<double>(
      transform_rot_param[3], transform_rot_param[0],
      transform_rot_param[1], transform_rot_param[2]);
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Wrong rotation format: supported rpy, quaternion");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  transform_ = Eigen::Matrix4d::Identity();
  pose_ = Eigen::Matrix4d::Identity();
  transform_.block<3, 3>(0, 0) = q.matrix();
  transform_.block<3, 1>(0, 3) = trans;

  std::cout << transform_ << std::endl;

  try {
    ee_pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
      "ee_pose",
      rclcpp::SystemDefaultsQoS());

    realtime_ee_pose_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
      ee_pose_publisher_);

    fd_button_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      "button_state", rclcpp::SystemDefaultsQoS());

    realtime_fd_button_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
      fd_button_publisher_);
  } catch (const std::exception & e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EePoseBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (state_interfaces_.size() != (joints_.size())) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Not all requested interfaces exists.");
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
EePoseBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joints_.clear();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend()) {
    return interface_and_value->second;
  } else {
    return kUninitializedValue;
  }
}

controller_interface::return_type EePoseBroadcaster::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (const auto & state_interface : state_interfaces_) {
    name_if_value_mapping_[
      state_interface.get_prefix_name()][state_interface.get_interface_name()] =
      state_interface.get_value();
    RCLCPP_DEBUG(
      get_node()->get_logger(), "%s/%s: %f\n", state_interface.get_prefix_name().c_str(),
      state_interface.get_interface_name().c_str(), state_interface.get_value());
  }

  if (realtime_ee_pose_publisher_ && realtime_ee_pose_publisher_->trylock()) {
    pose_ = Eigen::Matrix4d::Identity();

    if (joints_.size() >= 3) {
      double p_x = get_value(name_if_value_mapping_, joints_[0], HW_IF_POSITION);
      double p_y = get_value(name_if_value_mapping_, joints_[1], HW_IF_POSITION);
      double p_z = get_value(name_if_value_mapping_, joints_[2], HW_IF_POSITION);

      if (std::isnan(p_x) || std::isnan(p_y) || std::isnan(p_z)) {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "Failed to retrieve fd pose! (fd_x, fd_y, fd_z)!");
        return controller_interface::return_type::ERROR;
      }
      pose_(0, 3) = p_x;
      pose_(1, 3) = p_y;
      pose_(2, 3) = p_z;
    }

    if (joints_.size() >= 6) {
      double roll = get_value(name_if_value_mapping_, joints_[3], HW_IF_POSITION);
      double pitch = get_value(name_if_value_mapping_, joints_[4], HW_IF_POSITION);
      double yaw = get_value(name_if_value_mapping_, joints_[5], HW_IF_POSITION);

      if (std::isnan(roll) || std::isnan(pitch) || std::isnan(yaw)) {
        RCLCPP_DEBUG(
          get_node()->get_logger(), "Failed to retrieve fd pose! (fd_x, fd_y, fd_z)!");
        return controller_interface::return_type::ERROR;
      }

      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
      pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    }

    pose_ = transform_ * pose_;
    Eigen::Quaternion<double> q(pose_.block<3, 3>(0, 0));

    auto & ee_pose_msg = realtime_ee_pose_publisher_->msg_;

    ee_pose_msg.header.stamp = get_node()->get_clock()->now();
    ee_pose_msg.header.frame_id = "fd_base";
    // update pose message
    ee_pose_msg.pose.position.x = pose_(0, 3);
    ee_pose_msg.pose.position.y = pose_(1, 3);
    ee_pose_msg.pose.position.z = pose_(2, 3);
    ee_pose_msg.pose.orientation.w = q.w();
    ee_pose_msg.pose.orientation.x = q.x();
    ee_pose_msg.pose.orientation.y = q.y();
    ee_pose_msg.pose.orientation.z = q.z();

    realtime_ee_pose_publisher_->unlockAndPublish();
  }

  if (!buttons_.empty()) {
    if (realtime_fd_button_publisher_ && realtime_fd_button_publisher_->trylock()) {
      auto & ee_button_msg = realtime_fd_button_publisher_->msg_;
      double button_status = get_value(name_if_value_mapping_, buttons_[0], HW_IF_POSITION);
      ee_button_msg.data = button_status > 0.5;
      realtime_fd_button_publisher_->unlockAndPublish();
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace ee_pose_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ee_pose_broadcaster::EePoseBroadcaster, controller_interface::ControllerInterface)
