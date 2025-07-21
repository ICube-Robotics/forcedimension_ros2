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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <iterator>
#include <iostream>

#include "fd_hardware/fd_effort_hi.hpp"

#include "fd_sdk_vendor/dhd.hpp"
#include "fd_sdk_vendor/drd.hpp"

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fd_hardware
{

rclcpp::Logger LOGGER = rclcpp::get_logger("FDEffortHardwareInterface");

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


FDEffortHardwareInterface::~FDEffortHardwareInterface()
{
  // If controller manager is shutdown via Ctrl + C, the on_deactivate methods won't be called.
  // We need to call them here to ensure that the device is stopped and disconnected.
  on_deactivate(rclcpp_lifecycle::State());
}

// ------------------------------------------------------------------------------------------
CallbackReturn FDEffortHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }


  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_inertia_.resize(15, std::numeric_limits<double>::quiet_NaN());
  hw_button_state_.resize(info_.gpios.size(), std::numeric_limits<double>::quiet_NaN());


  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // PHI has currently exactly 3 states and 1 command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        LOGGER,
        "Joint '%s' has %lu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        LOGGER,
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        LOGGER,
        "Joint '%s' has %ld state interface. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        LOGGER,
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        LOGGER,
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        LOGGER,
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }
  for (const hardware_interface::ComponentInfo & button : info_.gpios) {
    if (button.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        LOGGER,
        "Button '%s' has %lu state interface. 1 expected.", button.name.c_str(),
        button.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (button.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        LOGGER,
        "Button '%s' have %s state interface. '%s' expected.", button.name.c_str(),
        button.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }
  for (const hardware_interface::ComponentInfo & button : info_.gpios) {
    if (button.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        LOGGER,
        "Button '%s' has %lu state interface. 1 expected.", button.name.c_str(),
        button.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (button.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        LOGGER,
        "Button '%s' have %s state interface. '%s' expected.", button.name.c_str(),
        button.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }


  // Get parameters
  auto it_interface_id = info_.hardware_parameters.find("interface_id");
  if (it_interface_id != info_.hardware_parameters.end()) {
    interface_ID_ = stoi(it_interface_id->second);
    RCLCPP_INFO(LOGGER, "Using interface ID: %d", interface_ID_);
  } else {
    interface_ID_ = -1;
  }


  auto it_interface_serial_number = info_.hardware_parameters.find("interface_serial_number");
  if (it_interface_serial_number != info_.hardware_parameters.end()) {
    interface_SN_ = stoi(it_interface_serial_number->second);
    RCLCPP_INFO(LOGGER, "Using interface serial number: %d", interface_SN_);
  } else {
    interface_SN_ = -1;
  }

  auto it_emulate_button = info_.hardware_parameters.find("emulate_button");
  if (it_emulate_button != info_.hardware_parameters.end()) {
    emulate_button_ = hardware_interface::parse_bool(it_emulate_button->second);
  } else {
    emulate_button_ = false;
  }
  RCLCPP_INFO(LOGGER, "Emulating button: %s", emulate_button_ ? "true" : "false");

  auto it_fd_inertia = info_.hardware_parameters.find("inertia_interface_name");
  if (it_fd_inertia != info_.hardware_parameters.end()) {
    inertia_interface_name_ = it_fd_inertia->second;
  } else {
    inertia_interface_name_ = "fd_inertia";
  }

  auto it_interface_mass = info_.hardware_parameters.find("effector_mass");
  if (it_interface_mass != info_.hardware_parameters.end()) {
    effector_mass_ = hardware_interface::stod(it_interface_mass->second);
    RCLCPP_INFO(LOGGER, "Interface mass parameter found: %lf Kg", effector_mass_);
  } else {
    effector_mass_ = -1.0;
  }


  auto it_ignore_orientation = info_.hardware_parameters.find("ignore_orientation_readings");
  if (it_ignore_orientation != info_.hardware_parameters.end()) {
    ignore_orientation_ = hardware_interface::parse_bool(it_ignore_orientation->second);
  } else {
    ignore_orientation_ = false;
  }
  RCLCPP_INFO(LOGGER, "Ignoring orientation readings: %s", ignore_orientation_ ? "true" : "false");

  // Contingency for emulated button
  // (commanded clutch force might be always left to NaN...)
  if (emulate_button_ &&
    (info_.joints.size() == 4 || info_.joints.size() > 6))
  {
    // Prevent NaN in clutch cmd
    hw_commands_effort_[info_.joints.size() - 1] = 0.0;
  }

  return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
FDEffortHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
  }
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
  }
  for (uint i = 0; i < info_.gpios.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.gpios[i].name, hardware_interface::HW_IF_POSITION, &hw_button_state_[i]));
  }

  // Map upper triangular part of inertia to inertia state interface
  for (uint row = 0; row < 6; row++) {
    for (uint col = row; col < 6; col++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          inertia_interface_name_,
          std::to_string(row) + "" + std::to_string(col),
          &hw_states_inertia_[flattened_index_from_triangular_index(row, col)]));
    }
  }

  return state_interfaces;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
FDEffortHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
  }

  return command_interfaces;
}
// ------------------------------------------------------------------------------------------
CallbackReturn FDEffortHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;  // hush "-Wunused-parameter" warning

  RCLCPP_INFO(LOGGER, "Starting ...please wait...");

  if (connectToDevice()) {
    RCLCPP_INFO(LOGGER, "System Successfully started!");
    return CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(LOGGER, "System Not started!");
    return CallbackReturn::ERROR;
  }
}
// ------------------------------------------------------------------------------------------
CallbackReturn FDEffortHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;  // hush "-Wunused-parameter" warning
  RCLCPP_INFO(LOGGER, "Stopping ...please wait...");

  if (disconnectFromDevice()) {
    RCLCPP_INFO(LOGGER, "System successfully stopped!");
    return CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(LOGGER, "System Not stopped!");
    return CallbackReturn::ERROR;
  }
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type FDEffortHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // "Success" flag
  int flag = 0;

  // Get position & orientation
  flag += dhdGetPosition(
    &hw_states_position_[0], &hw_states_position_[1], &hw_states_position_[2],
    interface_ID_);
  if (ignore_orientation_ && hw_states_position_.size() == 4) {
    // No orientation, skip!
  } else if (!ignore_orientation_ && hw_states_position_.size() > 3) {
    flag += dhdGetOrientationRad(
      &hw_states_position_[3], &hw_states_position_[4],
      &hw_states_position_[5], interface_ID_);
  } else if (ignore_orientation_ && hw_states_position_.size() > 3) {
    // Force orientation to zero
    hw_states_position_[3] = 0.0;
    hw_states_position_[4] = 0.0;
    hw_states_position_[5] = 0.0;
  }

  // Get gripper angle
  if (dhdHasGripper(interface_ID_) && hw_states_position_.size() == 4) {
    flag += dhdGetGripperAngleRad(&hw_states_position_[3], interface_ID_);
  } else if (dhdHasGripper(interface_ID_) && hw_states_position_.size() > 6) {
    flag += dhdGetGripperAngleRad(&hw_states_position_[6], interface_ID_);
  }

  // Get velocity
  flag += dhdGetLinearVelocity(
    &hw_states_velocity_[0], &hw_states_velocity_[1],
    &hw_states_velocity_[2], interface_ID_);
  if (dhdHasWrist(interface_ID_) && hw_states_velocity_.size() == 4) {
    // No orientation, skip!
  } else if (!ignore_orientation_ && hw_states_velocity_.size() > 3) {
    flag += dhdGetAngularVelocityRad(
      &hw_states_velocity_[3], &hw_states_velocity_[4],
      &hw_states_velocity_[5], interface_ID_);
  } else if (ignore_orientation_ && hw_states_velocity_.size() > 3) {
    // Force angular velocity to zero
    hw_states_velocity_[3] = 0.0;
    hw_states_velocity_[4] = 0.0;
    hw_states_velocity_[5] = 0.0;
  }

  // Get gripper angular velocity
  if (dhdHasGripper(interface_ID_) && hw_states_velocity_.size() == 4) {
    flag += dhdGetGripperAngularVelocityRad(&hw_states_velocity_[3], interface_ID_);
  } else if (dhdHasGripper(interface_ID_) && hw_states_velocity_.size() > 6) {
    flag += dhdGetGripperAngularVelocityRad(&hw_states_velocity_[6], interface_ID_);
  }

  // Get forces
  double torque[3];
  double gripper_force;
  flag += dhdGetForceAndTorqueAndGripperForce(
    &hw_states_effort_[0], &hw_states_effort_[1], &hw_states_effort_[2],
    &torque[0], &torque[1], &torque[2],
    &gripper_force,
    interface_ID_
  );

  // Get torques
  if (dhdHasWrist(interface_ID_) && hw_states_velocity_.size() == 4) {
    // No orientation, skip!
  } else if (!ignore_orientation_ && hw_states_effort_.size() > 3) {
    hw_states_effort_[3] = torque[0];
    hw_states_effort_[4] = torque[1];
    hw_states_effort_[5] = torque[2];
  } else if (ignore_orientation_ && hw_states_effort_.size() > 3) {
    // Force angular forces to zero
    hw_states_effort_[3] = 0.0;
    hw_states_effort_[4] = 0.0;
    hw_states_effort_[5] = 0.0;
  }

  // Get gripper force
  if (dhdHasGripper(interface_ID_) && hw_states_effort_.size() == 4) {
    hw_states_effort_[3] = gripper_force;
  } else if (dhdHasGripper(interface_ID_) && hw_states_effort_.size() > 6) {
    hw_states_effort_[6] = gripper_force;
  }
  // Get inertia
  double inertia_array[6][6];
  double joint_position[DHD_MAX_DOF];
  // use "dhdGetJointAngles (double j[DHD_MAX_DOF], char ID=-1)" to get the joint positions
  flag += dhdEnableExpertMode();
  flag += dhdGetJointAngles(joint_position, interface_ID_);
  // use "dhdJointAnglesToInertiaMatrix (double j[DHD_MAX_DOF], double inertia[6][6], char ID=-1)"
  // to get the current inertia matrix
  flag += dhdJointAnglesToInertiaMatrix(joint_position, inertia_array, interface_ID_);
  flag += dhdDisableExpertMode();

  // Map upper triangular part of inertia to inertia state interface
  for (uint row = 0; row < 6; row++) {
    for (uint col = row; col < 6; col++) {
      hw_states_inertia_[flattened_index_from_triangular_index(row, col)] = inertia_array[row][col];
    }
  }

  // Get button status, TODO multiple buttons from index
  int button_status = dhdGetButton(0, interface_ID_);
  if (button_status == 1) {
    hw_button_state_[0] = 1.0;
  } else if (button_status == 0) {
    hw_button_state_[0] = 0.0;
  } else {
    RCLCPP_ERROR(LOGGER, "Invalid button reading!");
    flag += -1;
  }

  if (flag >= 0) {
    return hardware_interface::return_type::OK;
  } else {
    RCLCPP_ERROR(LOGGER, "Updating from system failed!");
    return hardware_interface::return_type::ERROR;
  }
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type FDEffortHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  bool isNan = false;
  for (auto & command : hw_commands_effort_) {
    if (command != command) {
      isNan = true;
    }
  }

  if (!isNan) {
    if (dhdHasGripper(interface_ID_) && hw_states_effort_.size() > 6) {
      dhdSetForceAndTorqueAndGripperForce(
        hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
        hw_commands_effort_[3], hw_commands_effort_[4], hw_commands_effort_[5],
        hw_commands_effort_[6], interface_ID_);
    } else if (dhdHasWrist(interface_ID_) && hw_states_effort_.size() == 4) {
      dhdSetForceAndTorqueAndGripperForce(
        hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
        0.0, 0.0, 0.0, hw_commands_effort_[3], interface_ID_);
    } else if (dhdHasWrist(interface_ID_) && hw_states_effort_.size() > 3) {
      // No clutch joint
      dhdSetForceAndTorqueAndGripperForce(
        hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
        hw_commands_effort_[3], hw_commands_effort_[4], hw_commands_effort_[5],
        0, interface_ID_);
    } else {
      // Only translation is actuated
      dhdSetForceAndTorqueAndGripperForce(
        hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
        0, 0, 0, 0, interface_ID_);
    }
  } else {
    dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, interface_ID_);
  }


  return hardware_interface::return_type::OK;
}
// ------------------------------------------------------------------------------------------
bool FDEffortHardwareInterface::connectToDevice()
{
  int major, minor, release, revision;
  dhdGetSDKVersion(&major, &minor, &release, &revision);
  RCLCPP_INFO(
    LOGGER,
    "dhd : Using SDK version %d.%d (release %d / revision %d)",
    major, minor, release, revision);

  // Open connection
  bool dhd_open_success = false;
  if (interface_SN_ >= 0) {
    // Open specified device from serial number
    RCLCPP_INFO(
      LOGGER, "dhd : Connecting to device with serial number %d, please wait...", interface_SN_);
    uint16_t serialNumber = static_cast<uint16_t>(interface_SN_);
    interface_ID_ = dhdOpenSerial(serialNumber);
    dhd_open_success = (interface_ID_ >= 0);
    // dhd_open_success = (dhdOpenSerial(serialNumber) >= 0);
  } else if (interface_ID_ >= 0) {
    // Open specified device from device ID
    RCLCPP_INFO(LOGGER, "dhd : Connecting to device with ID= %d, please wait...", interface_ID_);
    interface_ID_ = dhdOpenID(interface_ID_);
    dhd_open_success = (interface_ID_ >= 0);
    // dhd_open_success = (dhdOpenID(interface_ID_) >= 0);
  } else {
    // Open default device
    RCLCPP_INFO(LOGGER, "dhd : Connecting to default device., please wait..");
    interface_ID_ = dhdOpen();
    dhd_open_success = (interface_ID_ >= 0);
    // dhd_open_success = (dhdOpen() >= 0);
  }

  // Check connection and setup dhd device
  if (dhd_open_success) {
    RCLCPP_INFO(LOGGER, "dhd : %s device detected", dhdGetSystemName(interface_ID_));
    uint16_t serialNumber = 0;
    if (dhdGetSerialNumber(&serialNumber, interface_ID_) < 0) {
      RCLCPP_WARN(LOGGER, "dhd : Impossible to retrieve device serial number: %s!",
        dhdErrorGetLastStr());
    } else {
      RCLCPP_INFO(LOGGER, "dhd : device serial number = %d", serialNumber);
    }
    RCLCPP_INFO(LOGGER, "dhd : device interface ID = %d", interface_ID_);

    // Check if the device has 3 dof or more
    if (dhdHasWrist(interface_ID_)) {
      RCLCPP_INFO(LOGGER, "dhd : Rotation supported");
    } else {
      RCLCPP_INFO(LOGGER, "dhd : Rotation not supported");
    }

    // Retrieve the mass of the device
    double current_effector_mass = 0.0;
    if (dhdGetEffectorMass(&current_effector_mass, interface_ID_) == DHD_NO_ERROR) {
      RCLCPP_INFO(LOGGER, "dhd : Effector Mass = %f (g)", current_effector_mass * 1000.0);
    } else {
      RCLCPP_WARN(LOGGER, "dhd : Impossible to retrieve effector mass !");
    }

    // Set force limit & enable force
    double forceMax = 12;   // N
    if (dhdSetMaxForce(forceMax, interface_ID_) < DHD_NO_ERROR) {
      RCLCPP_ERROR(LOGGER, "dhd : Could not set max force!");
      disconnectFromDevice();
    }
    // apply zero force
    dhdSetBrakes(DHD_OFF, interface_ID_);

    if (dhdEnableForce(DHD_ON, interface_ID_) < DHD_NO_ERROR) {
      RCLCPP_ERROR(LOGGER, "dhd : Could not enable force control!");
      disconnectFromDevice();
      return false;
    }
    // Set effector mass
    if (effector_mass_ > 0.0) {
      RCLCPP_INFO(
        LOGGER,
        "dhd : Changing effector mass from %fto %f (g)!",
        current_effector_mass * 1000.0,
        effector_mass_ * 1000.0);
      if (dhdSetEffectorMass(effector_mass_, interface_ID_) < DHD_NO_ERROR) {
        RCLCPP_ERROR(LOGGER, "dhd : Failed to set effector mass!");
        disconnectFromDevice();
        return false;
      }
    }
    // Gravity compensation
    if (dhdSetGravityCompensation(DHD_ON, interface_ID_) < DHD_NO_ERROR) {
      RCLCPP_ERROR(LOGGER, "dhd : Could not enable the gravity compensation !");
      disconnectFromDevice();
      return false;
    } else {
      RCLCPP_INFO(LOGGER, "dhd : Gravity compensation enabled");
    }
    RCLCPP_INFO(LOGGER, "dhd : Device connected !");

    // Emulate button
    if (emulate_button_ && !dhdHasGripper(interface_ID_)) {
      RCLCPP_ERROR(LOGGER, "dhd : Could not enable button emulation, no gripper found!");
    } else if (emulate_button_ && dhdHasGripper(interface_ID_)) {
      RCLCPP_INFO(LOGGER, "dhd : Emulating button from clutch joint...");
      if (dhdEmulateButton(DHD_ON, interface_ID_) < DHD_NO_ERROR) {
        RCLCPP_ERROR(LOGGER, "dhd : Could not enable button emulation!");
        disconnectFromDevice();
        return false;
      }
      RCLCPP_INFO(LOGGER, "dhd : OK, button will be emulated from clutch joint.");
    }
    // Set force to zero
    if (dhdSetForceAndTorqueAndGripperForce(
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        interface_ID_) < DHD_NO_ERROR)
    {
      RCLCPP_ERROR(LOGGER, "dhd : Could not initialize force control!");
      disconnectFromDevice();
      return false;
    }

    ignore_orientation_ |= !dhdHasWrist(interface_ID_);
    if (ignore_orientation_) {
      RCLCPP_INFO(LOGGER, "dhd : Orientation will be ignored !");
    }

    // Sleep 100 ms
    dhdSleep(0.1);
    isConnected_ = true;

    return isConnected_;
  } else {
    RCLCPP_ERROR(LOGGER, "dhd : Could not connect to device !");
    isConnected_ = false;
    return isConnected_;
  }
}
// ------------------------------------------------------------------------------------------
bool FDEffortHardwareInterface::disconnectFromDevice()
{
  // Stop the device: disables the force on the haptic device and puts it into BRAKE mode.
  int hasStopped = -1;
  while (hasStopped < 0) {
    RCLCPP_INFO(LOGGER, "dhd : stopping the device, please wait...");
    hasStopped = dhdStop(interface_ID_);
    // Sleep 100 ms
    dhdSleep(0.1);
  }

  // close device connection
  int connectionIsClosed = dhdClose(interface_ID_);
  if (connectionIsClosed >= 0) {
    RCLCPP_INFO(LOGGER, "dhd :  Disconnected ! ");
    interface_ID_ = -1;
    return true;
  } else {
    RCLCPP_ERROR(LOGGER, "dhd : Failed to disconnect !");
    return false;
  }
}

}  // namespace fd_hardware
   // ------------------------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  fd_hardware::FDEffortHardwareInterface, hardware_interface::SystemInterface)
