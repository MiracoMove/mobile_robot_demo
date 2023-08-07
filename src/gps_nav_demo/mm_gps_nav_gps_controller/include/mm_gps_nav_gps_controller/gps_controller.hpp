#pragma once

#include <memory>

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "aros2_hardware/hardware_controller.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using LifecycleNode = rclcpp_lifecycle::LifecycleNode;

namespace mm_gps_nav_gps_controller
{

struct GpsData
{
  std::shared_ptr<sensor_msgs::msg::NavSatFix> gps_msg;
  std::shared_ptr<sensor_msgs::msg::Imu> imu_msg;
};

class GpsEntity : public aros2_hardware::HardwareEntity<GpsData>
{
public:
  std::shared_ptr<GpsData> get_current_data() override
  {
    auto data = std::make_shared<GpsData>();
    data->gps_msg = gps_msg_;
    data->imu_msg = imu_msg_;
    return data;
  }

  bool is_alive() override
  {
    if (gps_msg_ == nullptr || imu_msg_ == nullptr) {
      return false;
    }
    auto now = get_node()->get_clock()->now();
    auto gps_gap = now - gps_msg_->header.stamp;
    auto imu_gap = now - imu_msg_->header.stamp;
    if (gps_gap.seconds() < 1 && imu_gap.seconds() < 1) {
      return true;
    }
    return false;
  }

protected:
  sensor_msgs::msg::NavSatFix::SharedPtr gps_msg_;
  sensor_msgs::msg::Imu::SharedPtr imu_msg_;

};

template<class EntityT = aros2_hardware::HardwareEntity<GpsData>>
class GpsController : public aros2_hardware::HardwareController<GpsData, EntityT>
{
  using hd_controller = aros2_hardware::HardwareController<GpsData, EntityT>;

public:
  GpsController()
  : hd_controller("mm_gps_nav_gps_controller")
  {
  }

  // 生命周期函数
  virtual CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    // 初始化 gps publisher
    gps_pub_ = std::make_shared<aros2_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>>(
      this->shared_from_this(),
      "~/gps/fix",
      rclcpp::SensorDataQoS());
    // 初始化 imu publisher
    imu_pub_ = std::make_shared<aros2_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>>(
      this->shared_from_this(),
      "~/gps/imu",
      rclcpp::SensorDataQoS());
    return hd_controller::on_configure(state);
  }

  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    // 为 gps publisher 添加定时器
    gps_pub_->add_timer(
      std::bind(&GpsController::publish, this), 10);
    return hd_controller::on_activate(state);
  }

  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    gps_pub_->remove_timer();
    return hd_controller::on_deactivate(state);
  }

  virtual void on_destroy(const rclcpp_lifecycle::State & state) override
  {
    if (gps_pub_ != nullptr) {
      gps_pub_->remove_timer();
    }
    gps_pub_.reset();
    imu_pub_.reset();
    hd_controller::on_destroy(state);
  }

private:
  void publish()
  {
    if (gps_pub_->is_activated() && imu_pub_->is_activated() && this->is_active()) {
      std::shared_ptr<GpsData> data = this->get_hardware_entity()->get_current_data();
      if (data != nullptr) {
        auto gps_msg = data->gps_msg;
        auto imu_msg = data->imu_msg;
        if (gps_msg != nullptr) {
          gps_pub_->publish(std::move(*gps_msg));
        }
        if (imu_msg != nullptr) {
          imu_pub_->publish(std::move(*imu_msg));
        }
      }
    }
  }

  std::shared_ptr<aros2_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>> gps_pub_;
  std::shared_ptr<aros2_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> imu_pub_;
};

} // namespace mm_gps_nav_gps_controller
