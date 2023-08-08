#include <memory>
#include <iostream>

#include "mm_gps_nav_imu_controller/imu_controller.hpp"


namespace mm_gps_nav_imu_controller
{
/// \brief Fake imu entity
class FakeImuEntity : public aros2_hardware::HardwareEntity<sensor_msgs::msg::Imu>
{
public:
  void on_activate() override
  {
    RCLCPP_INFO(get_node()->get_logger(), "FakeImuEntity on_activate() is called.");
    imu_sub_ = std::make_shared<aros2_core::Subscription<sensor_msgs::msg::Imu>>(
      get_node(),
      "imu",
      rclcpp::SensorDataQoS(),
      [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_msg_ = msg;
      });
    RCLCPP_INFO(get_node()->get_logger(), "FakeImuEntity on_activate() is finished.");
  }

  void on_deactivate() override
  {
    RCLCPP_INFO(get_node()->get_logger(), "FakeImuEntity on_deactivate() is called.");
    imu_sub_.reset();
    RCLCPP_INFO(get_node()->get_logger(), "FakeImuEntity on_deactivate() is finished.");
  }

  sensor_msgs::msg::Imu::SharedPtr get_current_data() override
  {
    return imu_msg_;
  }

  /// \brief Check if the imu is alive
  bool is_alive() override
  {
    if (imu_sub_ == nullptr || imu_msg_ == nullptr) {
      return false;
    }

    auto time_gap = get_node()->get_clock()->now() - imu_msg_->header.stamp;
    auto now = get_node()->get_clock()->now();

    if (time_gap.seconds() < 1) {
      return true;
    } else {
      RCLCPP_WARN_STREAM(
        get_node()->get_logger(),
        "time gap: " << time_gap.seconds() << ", header: " << imu_msg_->header.stamp.sec << "."
                     << imu_msg_->header.stamp.nanosec << ", now: " << now.seconds());
    }
    return false;
  }

private:
  std::shared_ptr<aros2_core::Subscription<sensor_msgs::msg::Imu>> imu_sub_;
  sensor_msgs::msg::Imu::SharedPtr imu_msg_;
};

} // namespace mm_gps_nav_imu_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<mm_gps_nav_imu_controller::ImuController<mm_gps_nav_imu_controller::FakeImuEntity>>();
  auto fake_imu_entity_ = std::make_shared<mm_gps_nav_imu_controller::FakeImuEntity>();
  node->add_hardware_entity(std::move(fake_imu_entity_));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
