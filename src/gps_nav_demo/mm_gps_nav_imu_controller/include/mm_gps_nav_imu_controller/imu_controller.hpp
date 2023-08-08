#pragma once

#include <memory>

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "aros2_hardware/hardware_controller.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


namespace mm_gps_nav_imu_controller
{


template<class EntityT = aros2_hardware::HardwareEntity<sensor_msgs::msg::Imu>>
class ImuController : public aros2_hardware::HardwareController<sensor_msgs::msg::Imu,
    EntityT>
{

  using hd_controller = aros2_hardware::HardwareController<sensor_msgs::msg::Imu, EntityT>;

public:
  ImuController()
  : hd_controller("mm_gps_nav_imu_controller")
  {
  }


  // 生命周期函数
  virtual CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    //初始化 imu publisher
    imu_pub_ = std::make_shared<aros2_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>>(
      this->shared_from_this(),
      "~/imu/fix",
      rclcpp::SensorDataQoS());
    return hd_controller::on_configure(state);
  }

  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    // 初始化 imu subscriber
    gps_imu_sub_ = std::make_shared<aros2_core::Subscription<sensor_msgs::msg::Imu>>(
      this->shared_from_this(),
      "mm_gps_nav_gps_controller/gps/imu", rclcpp::SensorDataQoS(),
      std::bind(&ImuController::gps_imu_sub_callback, this, std::placeholders::_1));
    // 为 imu publisher 添加定时器
    imu_pub_->add_timer(std::bind(&ImuController::publish, this), 30);
    return hd_controller::on_activate(state);
  }

  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    imu_pub_->remove_timer();
    gps_imu_sub_.reset();
    return hd_controller::on_deactivate(state);
  }

  virtual void on_destroy(const rclcpp_lifecycle::State & state) override
  {
    if (imu_pub_ != nullptr) {
      imu_pub_->remove_timer();
    }
    imu_pub_.reset();
    gps_imu_sub_.reset();
    hd_controller::on_destroy(state);
  }

private:
  void publish()
  {
    if (imu_pub_->is_activated() && this->is_active()) {
      // 非低功耗模式,需要检查数据实时性
      std::shared_ptr<sensor_msgs::msg::Imu> imu_msg =
        this->get_hardware_entity()->get_current_data();
      auto gps_imu_msg = gps_imu_msg_;
      if (imu_msg != nullptr && gps_imu_msg != nullptr) {
        auto now = this->get_clock()->now();
        // 检查 imu_msg 时间与当前时间的差值
        if (imu_msg != nullptr) {
          auto time_diff = rclcpp::Time(imu_msg->header.stamp) - now;
          if (time_diff.seconds() > 0.2) {
            RCLCPP_WARN_STREAM(
              this->get_logger(),
              "imu_msg is too old. Time diff is " << time_diff.seconds() << " seconds.");
            return;
          }
        }
        // 检查 gps_imu_msg 时间与当前时间的差值
        if (gps_imu_msg != nullptr) {
          auto time_diff = rclcpp::Time(gps_imu_msg->header.stamp) - now;
          if (time_diff.seconds() > 0.2) {
            RCLCPP_WARN_STREAM(
              this->get_logger(),
              "gps_imu_msg is too old. Time diff is " << time_diff.seconds() << " seconds.");
            return;
          }
        }
        auto merged_imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        merged_imu_msg->header = imu_msg->header;
        merged_imu_msg->orientation = gps_imu_msg->orientation;
        merged_imu_msg->orientation_covariance = gps_imu_msg->orientation_covariance;
        merged_imu_msg->angular_velocity = imu_msg->angular_velocity;
        merged_imu_msg->angular_velocity_covariance = imu_msg->angular_velocity_covariance;
        merged_imu_msg->linear_acceleration = imu_msg->linear_acceleration;
        merged_imu_msg->linear_acceleration_covariance = imu_msg->linear_acceleration_covariance;
        imu_pub_->publish(std::move(*merged_imu_msg));
      }
    }
  }

  void gps_imu_sub_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    gps_imu_msg_ = msg;
  }

  std::shared_ptr<aros2_core::Subscription<sensor_msgs::msg::Imu>> gps_imu_sub_;
  std::shared_ptr<aros2_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> imu_pub_;

  sensor_msgs::msg::Imu::SharedPtr gps_imu_msg_;
};
}   // namespace mm_gps_nav_imu_controller
