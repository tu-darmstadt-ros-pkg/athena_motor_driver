//
// Created by stefan on 05.03.25.
//

#ifndef ATHENA_MOTOR_DRIVER_CONTROLLER_BASE_H
#define ATHENA_MOTOR_DRIVER_CONTROLLER_BASE_H

#include <rclcpp/rclcpp.hpp>
#include <athena_motor_interface/athena_motor_interfaces.h>

namespace athena_motor_driver
{
class ControllerBase
{
public:
  using SharedPtr = std::shared_ptr<ControllerBase>;

  virtual ~ControllerBase() = default;

  virtual MotorCommand computeMotorCommand( double linear, double angular ) = 0;
};
}

#endif // ATHENA_MOTOR_DRIVER_CONTROLLER_BASE_H
