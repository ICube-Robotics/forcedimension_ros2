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

#ifndef FD_INERTIA_BROADCASTER__FD_INERTIA_BROADCASTER_HPP_
#define FD_INERTIA_BROADCASTER__FD_INERTIA_BROADCASTER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "fd_inertia_broadcaster/visibility_control.h"

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

namespace fd_inertia_broadcaster
{
class FdInertiaBroadcaster : public controller_interface::ControllerInterface
{
public:
  FD_INERTIA_BROADCASTER_PUBLIC
  FdInertiaBroadcaster();

  FD_INERTIA_BROADCASTER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;

  FD_INERTIA_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  FD_INERTIA_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  FD_INERTIA_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  FD_INERTIA_BROADCASTER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  FD_INERTIA_BROADCASTER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  FD_INERTIA_BROADCASTER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::string inertia_interface_name_;

  Eigen::Matrix4d transform_;
  Eigen::Matrix<double, 6, 6> inertia_in_base_, inertia_;

  //  Publishers
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> inertia_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
  realtime_inertia_publisher_;
};

}  // namespace fd_inertia_broadcaster

#endif  // FD_INERTIA_BROADCASTER__FD_INERTIA_BROADCASTER_HPP_
