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

#ifndef EE_POSE_BROADCASTER__EE_POSE_BROADCASTER_HPP_
#define EE_POSE_BROADCASTER__EE_POSE_BROADCASTER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ee_pose_broadcaster/visibility_control.h"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace ee_pose_broadcaster
{
class EePoseBroadcaster : public controller_interface::ControllerInterface
{
public:
  EE_POSE_BROADCASTER_PUBLIC
  EePoseBroadcaster();

  EE_POSE_BROADCASTER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;

  EE_POSE_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  EE_POSE_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  EE_POSE_BROADCASTER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  EE_POSE_BROADCASTER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  EE_POSE_BROADCASTER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  EE_POSE_BROADCASTER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  //  For the PoseStamped message,
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> ee_pose_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>
  realtime_ee_pose_publisher_;
  std::vector<std::string> joints_;
  std::unordered_map<std::string, std::unordered_map<std::string, double>> name_if_value_mapping_;

  Eigen::Matrix4d transform_, pose_;
};

}  // namespace ee_pose_broadcaster

#endif  // EE_POSE_BROADCASTER__EE_POSE_BROADCASTER_HPP_
