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

#include "fd_inertia_broadcaster/fd_inertia_broadcaster.hpp"

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

namespace fd_inertia_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
const size_t sizeFlattenedInertia = 21;

unsigned int flattened_index_from_triangular_index(
  unsigned int idx_row,
  unsigned int idx_col,
  unsigned int dim = 6)
{
  unsigned int i = idx_row;
  unsigned int j = idx_col;
  if (idx_col < idx_row) {
    i = idx_col;
    j = idx_row;
  }
  return i * (2 * dim - i - 1) / 2 + j;
}
template<class Derived>
void matrixEigenToMsg(const Eigen::MatrixBase<Derived> & e, std_msgs::msg::Float64MultiArray & m)
{
  if (m.layout.dim.size() != 2) {
    m.layout.dim.resize(2);
  }
  m.layout.dim[0].stride = e.rows() * e.cols();
  m.layout.dim[0].size = e.rows();
  m.layout.dim[1].stride = e.cols();
  m.layout.dim[1].size = e.cols();
  if (static_cast<int>(m.data.size()) != e.size()) {
    m.data.resize(e.size());
  }
  int ii = 0;
  for (int i = 0; i < e.rows(); ++i) {
    for (int j = 0; j < e.cols(); ++j) {
      m.data[ii++] = e.coeff(i, j);
    }
  }
}

FdInertiaBroadcaster::FdInertiaBroadcaster() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FdInertiaBroadcaster::on_init()
{
  try {
    auto_declare<std::string>("inertia_interface_name", std::string("fd_inertia"));
    auto_declare<std::vector<double>>("transform_translation", std::vector<double>());
    auto_declare<std::vector<double>>("transform_rotation", std::vector<double>());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FdInertiaBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration FdInertiaBroadcaster::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (inertia_interface_name_.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "No inertia interface name provided!");

  } else {
    // Map upper triangular part of inertia to inertia state interface
    for (uint row = 0; row < 6; row++) {
      for (uint col = row; col < 6; col++) {
        state_interfaces_config.names.push_back(
          inertia_interface_name_ +
          "/" +
          std::to_string(row) + "" + std::to_string(col)
        );
      }
    }
  }
  return state_interfaces_config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FdInertiaBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  inertia_interface_name_ = get_node()->get_parameter("inertia_interface_name").as_string();
  if (inertia_interface_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Please provide the inertia interface name!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Get transform parameters
  auto transform_trans_param = get_node()->get_parameter("transform_translation").as_double_array();
  auto transform_rot_param = get_node()->get_parameter("transform_rotation").as_double_array();
  Eigen::Quaternion<double> q;
  Eigen::Vector3d trans = Eigen::Vector3d::Zero();

  if (transform_trans_param.size() == 0) {
    RCLCPP_INFO(get_node()->get_logger(), "No (linear) transformation provided. Using t = 0,0,0");
    trans << 0.0, 0.0, 0.0;
  } else if (transform_trans_param.size() == 3) {
    trans << transform_trans_param[0], transform_trans_param[1], transform_trans_param[2];
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Wrong translation format");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  if (transform_rot_param.size() == 0) {
    q = Eigen::Quaternion<double>(1, 0, 0, 0);
    RCLCPP_INFO(get_node()->get_logger(),
        "No (angular) transformation provided. Using q = 1,0,0,0");
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
  transform_.block<3, 3>(0, 0) = q.matrix();
  transform_.block<3, 1>(0, 3) = trans;

  std::cout << transform_ << std::endl;

  try {
    inertia_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "fd_inertia",
      rclcpp::SystemDefaultsQoS());
    realtime_inertia_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      inertia_publisher_);
  } catch (const std::exception & e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }


  inertia_in_base_ = Eigen::Matrix<double, 6, 6>::Zero();
  inertia_ = Eigen::Matrix<double, 6, 6>::Zero();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FdInertiaBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (state_interfaces_.size() != sizeFlattenedInertia) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "Not all requested interfaces exists.");
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FdInertiaBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FdInertiaBroadcaster::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "Entering update()");
  if (realtime_inertia_publisher_ && realtime_inertia_publisher_->trylock()) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Lock acquired");
    inertia_in_base_ = Eigen::Matrix<double, 6, 6>::Zero();

    // Map upper triangular part of inertia to inertia state interface
    for (uint row = 0; row < 6; row++) {
      for (uint col = 0; col < 6; col++) {
        inertia_in_base_(row, col) =
          state_interfaces_[flattened_index_from_triangular_index(row, col)].get_value();
      }
    }

    // Registration of inertia values
    inertia_.block<3, 3>(0, 0) =
      transform_.block<3, 3>(0, 0) * inertia_in_base_.block<3, 3>(0, 0) *
      transform_.block<3, 3>(0, 0).transpose();
    inertia_.block<3, 3>(3, 3) =
      transform_.block<3, 3>(0, 0) * inertia_in_base_.block<3, 3>(3, 3) *
      transform_.block<3, 3>(0, 0).transpose();

    // Publish inertia
    auto & fd_inertia_msg = realtime_inertia_publisher_->msg_;
    matrixEigenToMsg(inertia_, fd_inertia_msg);
    RCLCPP_DEBUG(get_node()->get_logger(), "publish and unlock");
    realtime_inertia_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace fd_inertia_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fd_inertia_broadcaster::FdInertiaBroadcaster, controller_interface::ControllerInterface)
