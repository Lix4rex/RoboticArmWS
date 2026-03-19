#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

namespace ros_hardware
{

class RosHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RosHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Noms des joints
  std::vector<std::string> joint_names_;


  // State interfaces
  std::vector<double> hw_states_position_;

  // Command interfaces (vitesse cible par joint)
  std::vector<double> hw_commands_position_;

  // ROS2 node pour pub/sub
  rclcpp::Node::SharedPtr node_;

  // Publisher : commandes vers ESP32
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr angle_cmd_pub_;

  // Topics configurables via URDF parameters
  std::string cmd_topic_;
};

} 