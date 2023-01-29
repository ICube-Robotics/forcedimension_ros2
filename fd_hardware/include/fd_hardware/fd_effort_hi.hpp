#ifndef FD_HARDWARE__FD_EFFORT_HI
#define FD_HARDWARE__FD_EFFORT_HI

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "fd_hardware/visibility_control.h"


namespace fd_hardware
{
class FDEffortHardwareInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FDEffortHardwareInterface);

  FD_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  FD_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  FD_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  FD_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  FD_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  FD_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  FD_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:

  // ID of the interface (Rq: "-1" = unvalid/any that is connected)
  char interface_ID_ = -1;
  // Turned to true after the connection
  bool isConnected_ = false;    

  // Store the command for the robot
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;
  std::vector<double> hw_button_state_;

  /**
  Initiate the USB communication with the device.
  Warning: Once this method is called, the forces are enable on the device...
  @return flag = true if has succeded
  */
  bool connectToDevice();

  /**
  Terminate the USB communication with the device. 
  See disconnectFromDevice(std_msgs::msg::String& str) for details.
  
  @return flag = true if has succeded
  */
  bool disconnectFromDevice();

};

}  // namespace FD_HARDWARE

#endif  // FD_HARDWARE__FD_EFFORT_HI