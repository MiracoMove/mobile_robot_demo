#pragma once

#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace hardware_interface;

namespace mm_robot
{

class DiffDrive : public SystemInterface
{
public:
  DiffDrive();

  CallbackReturn on_init(const HardwareInfo & hardware_info) override;
  std::vector<StateInterface> export_state_interfaces() override;
  std::vector<CommandInterface> export_command_interfaces() override;
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  void start_serial_thread();

private:
  std::string left_front_wheel_name_ = "lf_wheel_joint";
  std::string right_front_wheel_name_ = "rf_wheel_joint";
  std::string left_back_wheel_name_ = "lb_wheel_joint";
  std::string right_back_wheel_name_ = "rb_wheel_joint";
  double wheel_separation_ = 0.162;
  double wheel_radius_ = 0.05;
  double max_real_vel_ = 0.0;

  double left_front_vel_ = 0;
  double right_front_vel_ = 0;
  double left_back_vel_ = 0;
  double right_back_vel_ = 0;
  double left_front_pos_ = 0;
  double right_front_pos_ = 0;
  double left_back_pos_ = 0;
  double right_back_pos_ = 0;

  double left_front_cmd_ = 0;
  double right_front_cmd_ = 0;
  double left_back_cmd_ = 0;
  double right_back_cmd_ = 0;
  double last_left_front_cmd_ = 0;
  double last_right_front_cmd_ = 0;
  double last_left_back_cmd_ = 0;
  double last_right_back_cmd_ = 0;

  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  rclcpp::Node::SharedPtr serial_node_;
  std::thread serial_thread_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr serial_executor_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_pub_;
};

}  // namespace mm_robot
