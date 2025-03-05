#ifndef ATHENA_MOTOR_DRIVER_ATHENA_MOTOR_DRIVER_H
  #define ATHENA_MOTOR_DRIVER_ATHENA_MOTOR_DRIVER_H

#include "athena_motor_driver/controllers/controller_base.h"

#include <athena_motor_interface/msg/full_motor_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <hector_ros2_utils/node.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>

namespace athena_motor_driver
{

class AthenaMotorDriver : public hector::Node
{
  struct InternalData;

public:
  explicit AthenaMotorDriver( const rclcpp::NodeOptions &options );

  ~AthenaMotorDriver() override;

private:
  void setupSerial();

  void closeSerial();

  void setupController( const std::string &controller_type );

  void update();

  template<typename T>
  void sendCommand( const T &command );

  std::vector<std::tuple<std::string, std::function<void( const rclcpp::Parameter & )>>>
      auto_reconfigurable_params_;

  rclcpp::SubscriptionBase::SharedPtr twist_sub_;
  rclcpp::Publisher<athena_motor_interface::msg::FullMotorStatus>::SharedPtr status_pub_;
  athena_motor_interface::msg::FullMotorStatus status_msg_;

  std::string controller_type_ = "diff_drive";
  ControllerBase::SharedPtr controller_;

  geometry_msgs::msg::Twist::ConstSharedPtr twist_msg_;
  rclcpp::Time last_twist_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<InternalData> data_;
  // Serial communication
  std::unique_ptr<LibSerial::SerialPort> serial_;
  LibSerial::DataBuffer buffer_;
  std::string port_name_ = "/dev/ttyACM0";
  int baud_rate_ = 115200;
  double wheel_radius_ = 0.07;
};

} // namespace athena_motor_driver

#endif // ATHENA_MOTOR_DRIVER_ATHENA_MOTOR_DRIVER_H
