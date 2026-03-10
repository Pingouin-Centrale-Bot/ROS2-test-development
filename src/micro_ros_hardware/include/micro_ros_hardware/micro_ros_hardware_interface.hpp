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

#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_messages/msg/wheels_command.hpp"

namespace micro_ros_hardware
{

class MicroRosHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MicroRosHardwareInterface)

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
  // Noms des joints (ordre : lf, rf, lb, rb)
  std::vector<std::string> joint_names_;

  // State interfaces (position + vitesse par joint)
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // Command interfaces (vitesse cible par joint)
  std::vector<double> hw_commands_velocity_;

  // ROS2 node pour pub/sub
  rclcpp::Node::SharedPtr node_;

  // Publisher : commandes vers ESP32
  rclcpp::Publisher<robot_messages::msg::WheelsCommand>::SharedPtr wheel_cmd_pub_;

  // Subscriber : encodeurs depuis ESP32
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Buffer thread-safe des derniers encodeurs reçus
  sensor_msgs::msg::JointState latest_joint_states_;
  std::mutex joint_state_mutex_;
  bool joint_states_received_{false};

  // Topics configurables via URDF parameters
  std::string cmd_topic_;
  std::string state_topic_;
};

}  // namespace micro_ros_hardware