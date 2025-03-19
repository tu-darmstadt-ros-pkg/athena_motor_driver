#include "motor_controller.h"
#include "motor_comm.h"

MotorController::MotorController()
    : left_pid_controller_( 0.1, 0.01, 0.0, -30, 30, 0.1 ),
      right_pid_controller_( 0.1, 0.01, 0.0, -30, 30, 0.1 )
{
}

MotorController::~MotorController() = default;

void MotorController::init( std::shared_ptr<MotorComm> front_comm,
                            std::shared_ptr<MotorComm> back_comm )
{
  front_motor_comm_ = front_comm;
  back_motor_comm_ = back_comm;
}

void MotorController::setVelocities( float left_velocity, float right_velocity )
{
  target_left_velocity_ = left_velocity;
  target_right_velocity_ = -right_velocity; // Invert right velocity due to motor orientation
}

void MotorController::stop()
{
  target_left_velocity_ = 0;
  target_right_velocity_ = 0;
}

MotorStatus toMotorStatus( const MotorCommStatus &status )
{
  MotorStatus result;
  result.valid = status.valid;
  switch ( status.mode ) {
  case MotorMode::BRAKE:
    result.mode = MotorStatus::Mode::BRAKE;
    break;
  case MotorMode::FOC:
    result.mode = MotorStatus::Mode::FOC;
    break;
  case MotorMode::CALIBRATE:
    result.mode = MotorStatus::Mode::CALIBRATE;
    break;
  default:
    result.mode = MotorStatus::Mode::INVALID;
  }
  result.temperature = status.temperature;
  result.error = MotorStatus::Error( status.error_code );
  result.torque = status.torque;
  result.velocity = status.velocity;
  result.position = status.position;
  result.acceleration = status.acceleration;
  return result;
}

bool updateStatus( MotorStatus &left, MotorStatus &right, elapsedMillis age_left,
                   elapsedMillis age_right, MotorCommStatus &status )
{
  if ( !status.valid )
    return false;
  if ( status.motor_id == 0 ) {
    left = toMotorStatus( status );
    age_left = 0;
  } else if ( status.motor_id == 1 ) {
    right = toMotorStatus( status );
    age_right = 0;
  } else {
    return false;
  }
  return true;
}

namespace
{
int sign( float value )
{
  if ( value > 0 ) {
    return 1;
  } else if ( value < 0 ) {
    return -1;
  }
  return 0;
}

MotorCommCommand computeMotorCommand( float velocity )
{
  MotorCommCommand command;
  if ( std::abs( velocity ) < 1E-4 ) {
    command.mode = MotorMode::FOC;
    command.velocity = 0;
    command.k_w = 50;
  } else {
    command.mode = MotorMode::FOC;
    command.velocity = velocity;
    if ( std::abs( velocity ) > 20 ) {
      command.k_w = 1;
    } else if ( std::abs( velocity ) > 10 ) {
      command.k_w = 4;
    } else {
      command.k_w = 15;
    }
  }
  command.torque = 0;
  command.position = 0;
  return command;
}
} // namespace

void MotorController::update()
{
  long elapsed_millis = time_since_last_command_;
  time_since_last_command_ = 0;
  // Limit acceleration
  // Really simple ramp up and faster ramp down for breaking
  float acceleration = MAX_DECELERATION;
  if ( std::signbit( target_left_velocity_ ) == std::signbit( left_velocity_ ) &&
       std::abs( target_left_velocity_ ) > std::abs( left_velocity_ ) ) {
    acceleration = MAX_ACCELERATION;
  }
  if ( std::signbit( target_right_velocity_ ) == std::signbit( right_velocity_ ) &&
       std::abs( target_right_velocity_ ) > std::abs( right_velocity_ ) ) {
    acceleration = MAX_ACCELERATION;
  }

  if ( std::abs( target_left_velocity_ - left_velocity_ ) < acceleration * elapsed_millis / 1000.0 ) {
    left_velocity_ = target_left_velocity_;
  } else {
    left_velocity_ +=
        sign( target_left_velocity_ - left_velocity_ ) * acceleration * elapsed_millis / 1000.0;
  }
  if ( std::abs( target_right_velocity_ - right_velocity_ ) < acceleration * elapsed_millis / 1000.0 ) {
    right_velocity_ = target_right_velocity_;
  } else {
    right_velocity_ +=
        sign( target_right_velocity_ - right_velocity_ ) * acceleration * elapsed_millis / 1000.0;
  }

  MotorCommCommand left_command;
  left_command.motor_id = 0;
  MotorCommCommand right_command;
  right_command.motor_id = 1;
  if ( status_age.front_left > 1000 || status_age.front_right > 1000 ||
       status_age.rear_left > 1000 || status_age.rear_right > 1000 ) {
    // If we haven't received any status in a while, stop
    left_velocity_ = 0;
    right_velocity_ = 0;
    left_command.mode = MotorMode::BRAKE;
    right_command.mode = MotorMode::BRAKE;
  } else {
    float current_left_velocity =
        ( motor_status_.front_left.velocity + motor_status_.rear_left.velocity ) / 2;
    float current_right_velocity =
        ( motor_status_.front_right.velocity + motor_status_.rear_right.velocity ) / 2;
    left_command.mode = MotorMode::FOC;
    left_command.torque = left_pid_controller_.computeTorque( left_velocity_, current_left_velocity );
    right_command.mode = MotorMode::FOC;
    right_command.torque =
        right_pid_controller_.computeTorque( right_velocity_, current_right_velocity );
  }

  std::vector<MotorCommStatus> status( 2 );
  front_motor_comm_->sendReceive( { left_command, right_command }, status );
  has_new_status_ |= updateStatus( motor_status_.front_left, motor_status_.front_right,
                                   status_age.front_left, status_age.front_right, status[0] );
  has_new_status_ |= updateStatus( motor_status_.front_left, motor_status_.front_right,
                                   status_age.front_left, status_age.front_right, status[1] );

  back_motor_comm_->sendReceive( { left_command, right_command }, status );
  has_new_status_ |= updateStatus( motor_status_.rear_left, motor_status_.rear_right,
                                   status_age.rear_left, status_age.rear_right, status[0] );
  has_new_status_ |= updateStatus( motor_status_.rear_left, motor_status_.rear_right,
                                   status_age.rear_left, status_age.rear_right, status[1] );
}
