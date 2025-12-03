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
      node, "wheel_separation", std::ref( wheel_separation_ ), "Distance between the wheels" ) );
  parameters_.push_back( hector::createReconfigurableParameter(
      node, "wheel_radius", std::ref( wheel_radius_ ), "Radius of the wheels" ) );
  parameters_.push_back( hector::createReconfigurableParameter(
      node, "rotational_amplification", std::ref( rotational_amplification_ ),
      "Amplification factor for rotational control component" ) );
  RCLCPP_DEBUG_STREAM( node->get_logger(), "DiffDriveController initialized with wheel separation "
                                               << wheel_separation_ << ", wheel radius "
                                               << wheel_radius_ << ", and rotational amplification "
                                               << rotational_amplification_ );
}

MotorCommand DiffDriveController::computeMotorCommand( double linear, double angular )
{
  // Apply rotational amplification to the angular component
  double amplified_angular = angular * rotational_amplification_;
  double left_velocity = linear - amplified_angular * wheel_separation_ / 2;
  double right_velocity = linear + amplified_angular * wheel_separation_ / 2;
  // Velocity is now in m/s, convert to rad/s using v = w * r  -->  w = v / r
  left_velocity /= wheel_radius_;
  right_velocity /= wheel_radius_;
  return MotorCommand::Velocity( float( left_velocity ), float( right_velocity ) );
}
} // namespace athena_motor_driver
