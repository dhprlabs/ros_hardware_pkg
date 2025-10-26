#include "ros2_control_demo_example_2/diffbot_system.hpp"

#include <cmath>
#include <limits>
#include <vector>
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_2
{

hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Setup wheels with configured ticks per revolution
  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Validate joint interfaces
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY ||
        joint.state_interfaces.size() != 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Invalid joint interface configuration for '%s'", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos);
  state_interfaces.emplace_back(wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel);
  state_interfaces.emplace_back(wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos);
  state_interfaces.emplace_back(wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd);
  command_interfaces.emplace_back(wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd);
  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating hardware, connecting to ESP32...");
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating hardware...");
  comms_.disconnect();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Read raw encoder counts from ESP32
  int left_enc = 0, right_enc = 0;
  comms_.readEncoderValues(left_enc, right_enc);

  wheel_l_.enc = left_enc;
  wheel_r_.enc = right_enc;

  double dt = period.seconds();

  // Compute wheel position (rad) from encoder ticks
  double prev_pos = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.enc * wheel_l_.rad_per_count;
  wheel_l_.vel = (wheel_l_.pos - prev_pos) / dt;

  prev_pos = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.enc * wheel_r_.rad_per_count;
  wheel_r_.vel = (wheel_r_.pos - prev_pos) / dt;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // wheel_l_.cmd and wheel_r_.cmd are commanded velocities in rad/s from the controller
  // Convert rad/s to rev/s or PWM as expected by comms_
  double cmd_l_rps = wheel_l_.cmd / (2.0 * M_PI);
  double cmd_r_rps = wheel_r_.cmd / (2.0 * M_PI);

  comms_.setMotorSpeeds(cmd_l_rps, cmd_r_rps);
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_2::DiffBotSystemHardware,
  hardware_interface::SystemInterface)
