#include "mm_robot/diffdrive.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mm_robot
{

DiffDrive::DiffDrive()
: logger_(rclcpp::get_logger("DiffDrive"))
{
  clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);
  rclcpp::on_shutdown(
    [this]() {
      // 关闭 Can 通信
      // if (can_socket_ != 0) {
      //   close_can(can_socket_);
      // }
      if (serial_node_ != nullptr) {
        serial_executor_->remove_node(serial_node_);
        serial_executor_ = nullptr;
        serial_node_ = nullptr;
        serial_thread_.join();
      }
      RCLCPP_INFO(logger_, "DiffDrive::on_shutdown()");
    });
}

CallbackReturn DiffDrive::on_init(const HardwareInfo & hardware_info)
{
  if (SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger_, "DiffDrive::on_init()");

  // 读取配置参数
  left_front_wheel_name_ = info_.hardware_parameters["left_front_wheel_name"];
  right_front_wheel_name_ = info_.hardware_parameters["right_front_wheel_name"];
  left_back_wheel_name_ = info_.hardware_parameters["left_back_wheel_name"];
  right_back_wheel_name_ = info_.hardware_parameters["right_back_wheel_name"];
  wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
  wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  max_real_vel_ = std::stod(info_.hardware_parameters["max_real_vel"]);
  RCLCPP_INFO(logger_, "left_front_wheel_name_: %s", left_front_wheel_name_.c_str());
  RCLCPP_INFO(logger_, "right_front_wheel_name_: %s", right_front_wheel_name_.c_str());
  RCLCPP_INFO(logger_, "left_back_wheel_name_: %s", left_back_wheel_name_.c_str());
  RCLCPP_INFO(logger_, "right_back_wheel_name_: %s", right_back_wheel_name_.c_str());
  RCLCPP_INFO(logger_, "wheel_separation_: %f", wheel_separation_);
  RCLCPP_INFO(logger_, "max_real_vel_: %f", max_real_vel_);

  // 初始化输入输出
  left_front_vel_ = std::numeric_limits<double>::quiet_NaN();
  right_front_vel_ = std::numeric_limits<double>::quiet_NaN();
  left_back_vel_ = std::numeric_limits<double>::quiet_NaN();
  right_back_vel_ = std::numeric_limits<double>::quiet_NaN();
  left_front_pos_ = std::numeric_limits<double>::quiet_NaN();
  right_front_pos_ = std::numeric_limits<double>::quiet_NaN();
  left_back_pos_ = std::numeric_limits<double>::quiet_NaN();
  right_back_pos_ = std::numeric_limits<double>::quiet_NaN();
  left_front_cmd_ = std::numeric_limits<double>::quiet_NaN();
  right_front_cmd_ = std::numeric_limits<double>::quiet_NaN();
  left_back_cmd_ = std::numeric_limits<double>::quiet_NaN();
  right_back_cmd_ = std::numeric_limits<double>::quiet_NaN();

  RCLCPP_INFO(logger_, "DiffDrive::on_init() done");

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> DiffDrive::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    StateInterface(
      left_front_wheel_name_, HW_IF_VELOCITY,
      &left_front_vel_));
  state_interfaces.emplace_back(
    StateInterface(
      right_front_wheel_name_, HW_IF_VELOCITY,
      &right_front_vel_));
  state_interfaces.emplace_back(
    StateInterface(
      left_back_wheel_name_, HW_IF_VELOCITY,
      &left_back_vel_));
  state_interfaces.emplace_back(
    StateInterface(
      right_back_wheel_name_, HW_IF_VELOCITY,
      &right_back_vel_));
  state_interfaces.emplace_back(
    StateInterface(
      left_front_wheel_name_, HW_IF_POSITION,
      &left_front_pos_));
  state_interfaces.emplace_back(
    StateInterface(
      right_front_wheel_name_, HW_IF_POSITION,
      &right_front_pos_));
  state_interfaces.emplace_back(
    StateInterface(
      left_back_wheel_name_, HW_IF_POSITION,
      &left_back_pos_));
  state_interfaces.emplace_back(
    StateInterface(
      right_back_wheel_name_, HW_IF_POSITION,
      &right_back_pos_));

  return state_interfaces;
}

std::vector<CommandInterface> DiffDrive::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    CommandInterface(
      left_front_wheel_name_, HW_IF_VELOCITY,
      &left_front_cmd_));
  command_interfaces.emplace_back(
    CommandInterface(
      right_front_wheel_name_, HW_IF_VELOCITY,
      &right_front_cmd_));
  command_interfaces.emplace_back(
    CommandInterface(
      left_back_wheel_name_, HW_IF_VELOCITY,
      &left_back_cmd_));
  command_interfaces.emplace_back(
    CommandInterface(
      right_back_wheel_name_, HW_IF_VELOCITY,
      &right_back_cmd_));

  return command_interfaces;
}

CallbackReturn DiffDrive::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(logger_, "DiffDrive::on_activate()");

  // TODO: 初始化写入串口的 ROS2 Node

  RCLCPP_INFO(logger_, "init input");
  // 初始化输入输出
  if (std::isnan(left_front_vel_)) {
    left_front_vel_ = 0.0;
    right_front_vel_ = 0.0;
    left_back_vel_ = 0.0;
    right_back_vel_ = 0.0;
    left_front_pos_ = 0.0;
    right_front_pos_ = 0.0;
    left_back_pos_ = 0.0;
    right_back_pos_ = 0.0;
    left_front_cmd_ = 0.0;
    right_front_cmd_ = 0.0;
    left_back_cmd_ = 0.0;
    right_back_cmd_ = 0.0;
  }

  serial_thread_ = std::thread(std::bind(&DiffDrive::start_serial_thread, this));

  RCLCPP_INFO(logger_, "DiffDrive::on_activate() done");

  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDrive::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(logger_, "DiffDrive::on_deactivate()");

  // 初始化输入输出
  left_front_vel_ = 0.0;
  right_front_vel_ = 0.0;
  left_back_vel_ = 0.0;
  right_back_vel_ = 0.0;
  left_front_pos_ = 0.0;
  right_front_pos_ = 0.0;
  left_back_pos_ = 0.0;
  right_back_pos_ = 0.0;
  left_front_cmd_ = 0.0;
  right_front_cmd_ = 0.0;
  left_back_cmd_ = 0.0;
  right_back_cmd_ = 0.0;

  // TODO: 速度归零
  geometry_msgs::msg::Twist twist;
  speed_pub_->publish(twist);

  // TOOD: 关闭串口通信

  if (serial_node_ != nullptr) {
    serial_executor_->remove_node(serial_node_);
    serial_executor_ = nullptr;
    serial_node_ = nullptr;
    serial_thread_.join();
  }

  RCLCPP_INFO(logger_, "DiffDrive::on_deactivate() done");

  return CallbackReturn::SUCCESS;
}

return_type DiffDrive::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return return_type::OK;
  }
  // 读取反馈数据
  left_front_vel_ = left_front_cmd_;
  right_front_vel_ = right_front_cmd_;
  left_back_vel_ = left_back_cmd_;
  right_back_vel_ = right_back_cmd_;

  return return_type::OK;
}

return_type DiffDrive::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return return_type::OK;
  }

  if (last_left_front_cmd_ == 0 && last_right_front_cmd_ == 0 &&
    left_front_cmd_ == 0 && right_front_cmd_ == 0 &&
    last_left_back_cmd_ == 0 && last_right_back_cmd_ == 0 &&
    left_back_cmd_ == 0 && right_back_cmd_ == 0)
  {
    return return_type::OK;
  }

  geometry_msgs::msg::Twist twist;

  if (left_front_cmd_ == 0 && right_front_cmd_ == 0 &&
    left_back_cmd_ == 0 && right_back_cmd_ == 0)
  {
    RCLCPP_INFO_STREAM(logger_, "left: " << left_front_cmd_ << ", right: " << right_front_cmd_);
    // TODO: 速度归零

    last_left_front_cmd_ = 0;
    last_right_front_cmd_ = 0;
    last_left_back_cmd_ = 0;
    last_right_back_cmd_ = 0;
    speed_pub_->publish(twist);
    return return_type::OK;
  }

  RCLCPP_INFO_STREAM_THROTTLE(
    logger_, clock_, 1000,
    "left: " << left_front_cmd_ << ", right: " << right_front_cmd_);


  // 计算线速度
  auto linear = (left_front_cmd_ + right_front_cmd_) * wheel_radius_ / 2.0;

  // 计算角速度
  auto angular = (right_front_cmd_ - left_front_cmd_) * wheel_radius_ / (wheel_separation_);

  RCLCPP_INFO_STREAM_THROTTLE(
    logger_, clock_, 1000,
    "linear: " << linear << ", angular: " << angular);

  twist.linear.x = linear;
  twist.angular.z = angular;
  speed_pub_->publish(twist);

  // if (!linear_move(can_socket_, -linear)) {
  //   return return_type::ERROR;
  // }
  // if (!angular_move(can_socket_, angular)) {
  //   return return_type::ERROR;
  // }

  last_left_front_cmd_ = left_front_cmd_;
  last_right_front_cmd_ = right_front_cmd_;

  return return_type::OK;
}

CallbackReturn DiffDrive::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(logger_, "DiffDrive::on_cleanup()");

  if (serial_node_ != nullptr) {
    serial_executor_->remove_node(serial_node_);
    serial_executor_ = nullptr;
    serial_node_ = nullptr;
    serial_thread_.join();
  }

  RCLCPP_INFO(logger_, "DiffDrive::on_cleanup() done");
  return CallbackReturn::SUCCESS;
}

void DiffDrive::start_serial_thread()
{
  RCLCPP_INFO(logger_, "DiffDrive::start_serial_thread()");
  serial_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  serial_node_ = std::make_shared<rclcpp::Node>("serial_node");
  speed_pub_ = serial_node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  serial_executor_->add_node(serial_node_);
  serial_executor_->spin();
  RCLCPP_INFO(logger_, "Stop thread!!!!!!!!!");
}

}  // namespace mm_robot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mm_robot::DiffDrive, hardware_interface::SystemInterface)
