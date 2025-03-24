//
// Created by stefan on 05.03.25.
//

#ifndef ATHENA_MOTOR_DRIVER_DIFF_DRIVE_CONTROLLER_HPP
#define ATHENA_MOTOR_DRIVER_DIFF_DRIVE_CONTROLLER_HPP

#include "athena_motor_driver/controllers/controller_base.h"
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>

namespace athena_motor_driver
{

class DiffDriveController : public ControllerBase
{
public:
  explicit DiffDriveController( const rclcpp::Node::SharedPtr &node );

  MotorCommand computeMotorCommand( double linear, double angular ) override;

private:
  std::vector<hector::ParameterSubscription> parameters_;
  double wheel_radius_ = 0.075;
  double wheel_separation_ = 0.4;
};
} // namespace athena_motor_driver

#endif // ATHENA_MOTOR_DRIVER_DIFF_DRIVE_CONTROLLER_HPP
