//Boost库文件
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <cmath>
#include <nmeaparse/nmea.h>

#include "tf2/LinearMath/Quaternion.h"

#include "mm_gps_nav_gps_controller/gps_controller.hpp"

typedef boost::shared_ptr<boost::asio::serial_port> serial_ptr;

namespace mm_gps_nav_gps_controller
{

class WitGpsEntity : public GpsEntity
{
public:
  void on_activate() override
  {
    RCLCPP_INFO(get_node()->get_logger(), "WitGpsEntity on_activate");
    //提示信息，串口端口号和波特率
    RCLCPP_INFO(
      get_node()->get_logger(), "Tarkbot Robot Set serial %s at %d baud",
      serial_port_.c_str(), serial_port_baud_);

    //初始化串口
    if (openSerialPort()) {
      try {
        //启动串口接收线程
        boost::thread recvSerial_thread(boost::bind(&WitGpsEntity::recvCallback, this));
      } catch (...) {
        RCLCPP_INFO(
          get_node()->get_logger(),
          "Tarkbot Robot can not open recvSerial_thread, Please check the serial port cable ");

        //关闭节点
        throw std::runtime_error("Tarkbot Robot can not open recvSerial_thread");
      }
    } else {
      //关闭节点
      throw std::runtime_error("Tarkbot Robot can not open recvSerial_thread");
    }

    RCLCPP_INFO(get_node()->get_logger(), "Tarkbot Robot is Connected to OpenCTR board ");
  }

  void on_deactivate() override
  {
    RCLCPP_INFO(get_node()->get_logger(), "WitGpsEntity on_deactivate");
    closeSerialPort();
  }

private:
  //串口操作
  bool openSerialPort()
  {
    //检查串口是否已经被打开
    if (serial_ptr_) {
      RCLCPP_INFO(get_node()->get_logger(), "The SerialPort is already opened!\r\n");
      return false;
    }

    //开打串口
    serial_ptr_ = serial_ptr(new boost::asio::serial_port(io_service_));
    serial_ptr_->open(serial_port_, err_code_);

    //串口是否正常打开
    if (err_code_) {
      RCLCPP_INFO(
        get_node()->get_logger(), "Open Port: %s Failed! Aboart!",
        serial_port_.c_str());
      return false;
    }

    //初始化串口参数
    serial_ptr_->set_option(boost::asio::serial_port_base::baud_rate(serial_port_baud_));
    serial_ptr_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_ptr_->set_option(
      boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::
        stop_bits::one));
    serial_ptr_->set_option(
      boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::
        parity::none));
    serial_ptr_->set_option(
      boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base
        ::flow_control::none));

    return true;
  }
  void closeSerialPort()
  {
    //如果串口被打开，则关闭串口
    if (serial_ptr_) {
      serial_ptr_->cancel();
      serial_ptr_->close();
      serial_ptr_.reset();
    }

    //
    io_service_.stop();
    io_service_.reset();
  }

  //串口多线程接收函数
  void recvCallback()
  {
    //接收数据
    uint8_t res;

    // Create a GPS service that will keep track of the fix data.
    nmea::NMEAParser parser;
    nmea::GPSService gps(parser);
    parser.log = false;

    // parser.onSentence += [&](const nmea::NMEASentence & nmea) {
    //     try {
    //       RCLCPP_INFO_STREAM(
    //         get_node()->get_logger(),
    //         nmea.name);
    //     } catch (const std::exception & e) {
    //       RCLCPP_WARN_STREAM(get_node()->get_logger(), e.what());
    //     }
    //   };

    parser.setSentenceHandler(
      "WTRTK", [&](const nmea::NMEASentence & n) {
        try {
          auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
          imu_msg->header.frame_id = "gps_link";
          imu_msg->header.stamp = rclcpp::Clock().now();

          auto roll = std::stod(n.parameters[5]);
          auto pitch = std::stod(n.parameters[4]);
          auto yaw = std::stod(n.parameters[6]);

          // 将RPY转换成四元数
          tf2::Quaternion quat;
          quat.setRPY(roll, pitch, yaw);
          imu_msg->orientation.x = quat.x();
          imu_msg->orientation.y = quat.y();
          imu_msg->orientation.z = quat.z();
          imu_msg->orientation.w = quat.w();

          imu_msg_ = imu_msg;
        } catch (const std::exception & e) {
          RCLCPP_WARN_STREAM(get_node()->get_logger(), e.what());
        }
      });

    gps.onUpdate += [&]() {
        try {

          auto gps_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
          gps_msg->header.frame_id = "gps_link";
          gps_msg->header.stamp = rclcpp::Clock().now();
          gps_msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
          gps_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

          if (!gps.fix.locked()) {
            gps_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            gps_msg->position_covariance[0] = 100000.0;
            gps_msg->position_covariance[4] = 100000.0;
            gps_msg->position_covariance[8] = 100000.0;
            gps_msg_ = gps_msg;
            return;
          }

          gps_msg->position_covariance_type =
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
          if (gps.fix.quality == 1) {
            gps_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
          } else if (gps.fix.quality == 2) {
            gps_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
          } else {
            gps_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
          }

          gps_msg->latitude = gps.fix.latitude;
          gps_msg->longitude = gps.fix.longitude;
          gps_msg->altitude = gps.fix.altitude;

          gps_msg->position_covariance[0] =
            std::pow( (gps.fix.horizontalDilution * gps.fix.horizontalAccuracy()), 2);
          gps_msg->position_covariance[4] = gps_msg->position_covariance[0];
          gps_msg->position_covariance[8] =
            std::pow( (gps.fix.verticalDilution * gps.fix.verticalAccuracy()), 2);

          gps_msg_ = gps_msg;
        } catch (const std::exception & e) {
          RCLCPP_WARN_STREAM(get_node()->get_logger(), e.what());
        }
      };

    while (1) {
      try {
        //读取串口数据
        boost::asio::read(*serial_ptr_.get(), boost::asio::buffer(&res, 1), err_code_);
        parser.readByte(res);
      } catch (const std::exception & e) {
        RCLCPP_WARN_STREAM(get_node()->get_logger(), e.what());
      }
    }
  }

  //串口指针
  std::string serial_port_ = "/dev/ttyUSB0";
  int serial_port_baud_ = 230400;
  boost::shared_ptr<boost::asio::serial_port> serial_ptr_;
  boost::system::error_code err_code_;
  boost::asio::io_service io_service_;
};

} // namespace mm_gps_nav_gps_controller

int main(int argc, char ** argv)
{
  using namespace mm_gps_nav_gps_controller;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsController<WitGpsEntity>>();
  auto wit_gps_entity = std::make_shared<WitGpsEntity>();
  node->add_hardware_entity(std::move(wit_gps_entity));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
