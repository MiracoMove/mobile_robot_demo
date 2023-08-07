#include <memory>
#include <iostream>

#include "mm_gps_nav_gps_controller/gps_controller.hpp"

namespace mm_gps_nav_gps_controller
{
/// Fake gps entity
class FakeGpsEntity : public GpsEntity
{
public:
  void on_activate() override
  {
    RCLCPP_INFO(get_node()->get_logger(), "FakeGpsEntity on_activate() is called.");
    gps_sub_ = std::make_shared<aros2_core::Subscription<sensor_msgs::msg::NavSatFix>>(
      get_node(),
      "gps",
      rclcpp::SensorDataQoS(),
      [&](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        gps_msg_ = msg;
      });
    imu_sub_ = std::make_shared<aros2_core::Subscription<sensor_msgs::msg::Imu>>(
      get_node(),
      "imu",
      rclcpp::SensorDataQoS(),
      [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_msg_ = msg;
      });
    RCLCPP_INFO(get_node()->get_logger(), "FakeGpsEntity on_activate() is finished.");
  }

  void on_deactivate() override
  {
    RCLCPP_INFO(get_node()->get_logger(), "FakeGpsEntity on_deactivate() is called.");
    gps_sub_.reset();
    imu_sub_.reset();
    RCLCPP_INFO(get_node()->get_logger(), "FakeGpsEntity on_deactivate() is finished.");
  }

private:
  std::shared_ptr<aros2_core::Subscription<sensor_msgs::msg::NavSatFix>> gps_sub_;
  std::shared_ptr<aros2_core::Subscription<sensor_msgs::msg::Imu>> imu_sub_;
};

} // namespace mm_gps_nav_gps_controller

int main(int argc, char ** argv)
{
  using namespace mm_gps_nav_gps_controller;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsController<FakeGpsEntity>>();
  auto fake_gps_entity = std::make_shared<FakeGpsEntity>();
  node->add_hardware_entity(std::move(fake_gps_entity));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
