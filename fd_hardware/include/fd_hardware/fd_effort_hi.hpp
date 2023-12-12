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

#ifndef FD_HARDWARE__FD_EFFORT_HI_HPP_
#define FD_HARDWARE__FD_EFFORT_HI_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "fd_hardware/visibility_control.h"


namespace fd_hardware
{
class FDEffortHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FDEffortHardwareInterface);

  virtual ~FDEffortHardwareInterface();

  FD_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  FD_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  FD_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  FD_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  FD_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  FD_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  FD_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // ID of the interface (Rq: "-1" = invalid/any that is connected)
  char interface_ID_ = -1;
  // Turned to true after the connection
  bool isConnected_ = false;

  // Store the command for the robot
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;

  /**
  Initiate the USB communication with the device.
  Warning: Once this method is called, the forces are enable on the device...
  @return flag = true if has succeeded
  */
  bool connectToDevice();

  /**
  Terminate the USB communication with the device.
  See disconnectFromDevice(std_msgs::msg::String& str) for details.

  @return flag = true if has succeeded
  */
  bool disconnectFromDevice();
};


}  // namespace fd_hardware

#endif  // FD_HARDWARE__FD_EFFORT_HI_HPP_
