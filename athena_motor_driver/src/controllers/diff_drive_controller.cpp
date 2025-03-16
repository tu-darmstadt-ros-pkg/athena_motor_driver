//
// Created by stefan on 05.03.25.
//

#include "athena_motor_driver/controllers/diff_drive_controller.hpp"
#include <rclcpp/wait_for_message.hpp>

namespace athena_motor_driver
{

DiffDriveController::DiffDriveController( const rclcpp::Node::SharedPtr &node )
{
  parameters_.push_back( hector::createReconfigurableParameter(
      node, "wheel_separation", wheel_separation_, "Distance between the wheels" ) );
  parameters_.push_back( hector::createReconfigurableParameter( node, "wheel_radius", wheel_radius_,
                                                                "Radius of the wheels" ) );
  RCLCPP_DEBUG_STREAM( node->get_logger(), "DiffDriveController initialized with wheel separation "
                                               << wheel_separation_ << " and wheel radius "
                                               << wheel_radius_ );
}

MotorCommand DiffDriveController::computeMotorCommand( double linear, double angular )
{
  double left_velocity = linear - angular * wheel_separation_ / 2;
  double right_velocity = linear + angular * wheel_separation_ / 2;
  // Velocity is now in m/s, convert to rad/s using v = w * r  -->  w = v / r
  left_velocity /= wheel_radius_;
  right_velocity /= wheel_radius_;
  return { float( left_velocity ), float( right_velocity ) };
}
} // namespace athena_motor_driver
