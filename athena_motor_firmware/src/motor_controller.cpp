#include "motor_controller.h"
#include "motor_comm.h"

// Initialize the PID controllers with 0, as they will be set by driver
MotorController::MotorControlData::MotorControlData()
    : control_mode( ControlMode::POSITION ), position_filter(), velocity_filter(),
      velocity_pid_controller( 0, 0, 0, -MOTOR_TORQUE_LIMIT, MOTOR_TORQUE_LIMIT, MAX_TORQUE_CHANGE ),
      position_pid_controller( 0, 0, 0, -MOTOR_TORQUE_LIMIT, MOTOR_TORQUE_LIMIT, MAX_TORQUE_CHANGE ),
      position( 0 )
{
}

MotorController::MotorController() { }

MotorController::~MotorController() = default;

void MotorController::init( std::shared_ptr<MotorComm> front_comm,
                            std::shared_ptr<MotorComm> back_comm )
{
  front_motor_comm_ = front_comm;
  rear_motor_comm_ = back_comm;
}

void MotorController::setCommand( const MotorCommand &command ) { command_ = command; }

void MotorController::setPositionPIDGains( const PIDGains &left_pid_gains,
                                           const PIDGains &right_pid_gains )
{
  control_data_left_.position_pid_controller.setGains( left_pid_gains.k_p, left_pid_gains.k_i,
                                                       left_pid_gains.k_d );
  control_data_right_.position_pid_controller.setGains( right_pid_gains.k_p, right_pid_gains.k_i,
                                                        right_pid_gains.k_d );
}

void MotorController::setVelocityPIDGains( const PIDGains &left_pid_gains,
                                           const PIDGains &right_pid_gains )
{
  control_data_left_.velocity_pid_controller.setGains( left_pid_gains.k_p, left_pid_gains.k_i,
                                                       left_pid_gains.k_d );
  control_data_right_.velocity_pid_controller.setGains( right_pid_gains.k_p, right_pid_gains.k_i,
                                                        right_pid_gains.k_d );
}

void MotorController::stop()
{
  command_ = MotorCommand();
  target_velocity_.left = 0;
  target_velocity_.right = 0;
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
  result.velocity_high = status.velocity_high;
  result.velocity_low = status.velocity_low;
  result.position = status.position;
  result.acceleration = status.acceleration;
  return result;
}

MotorController::Torque MotorController::computeTorque()
{
  if ( !initialized_position_ ) {
    return { 0, 0 }; // Do not issue any torque commands until position is initialized
  }
  if ( command_.mode == MotorCommand::MotorMode::TORQUE ) {
    return { command_.left, -command_.right };
  } else if ( command_.mode == MotorCommand::MotorMode::BRAKE ) {
    return { 0, 0 };
  }
  target_velocity_.left = command_.left;
  target_velocity_.right = -command_.right;
  long elapsed_micros = time_since_last_command_;
  time_since_last_command_ = 0;
  // Limit acceleration
  // Really simple ramp up and faster ramp down for breaking
  float acceleration = MAX_DECELERATION;
  if ( std::signbit( target_velocity_.left ) == std::signbit( velocity_.left ) &&
       std::abs( target_velocity_.left ) > std::abs( velocity_.left ) ) {
    acceleration = MAX_ACCELERATION;
  }
  if ( std::signbit( target_velocity_.right ) == std::signbit( velocity_.right ) &&
       std::abs( target_velocity_.right ) > std::abs( velocity_.right ) ) {
    acceleration = MAX_ACCELERATION;
  }

  const float max_velocity_change = acceleration * elapsed_micros / 1E6f;

  if ( const float velocity_change = target_velocity_.left - velocity_.left;
       std::abs( velocity_change ) < max_velocity_change ) {
    velocity_.left = target_velocity_.left;
  } else {
    velocity_.left += std::copysign( max_velocity_change, velocity_change );
  }
  if ( const float velocity_change = target_velocity_.right - velocity_.right;
       std::abs( velocity_change ) < max_velocity_change ) {
    velocity_.right = target_velocity_.right;
  } else {
    velocity_.right += std::copysign( max_velocity_change, velocity_change );
  }
  if ( disable_acceleration_limiting_ ) {
    // If acceleration limits are disabled, we just set the target velocity directly
    // This is useful for tuning the PID controller but should not be used in normal operation
    velocity_.left = target_velocity_.left;
    velocity_.right = target_velocity_.right;
  }

  // For both motors, we use a separate PID controller for position and velocity control
  // If the velocity is close to zero, we use position control, otherwise we use velocity control
  float left_torque = 0;
  if ( std::abs( velocity_.left ) < 0.1f ) {
    if ( control_data_left_.control_mode != MotorControlData::ControlMode::POSITION ) {
      control_data_left_.control_mode = MotorControlData::ControlMode::POSITION;
      control_data_left_.position_filter.reset(); // Also reset filter so current position is 0
      control_data_left_.position_pid_controller.reset();
      control_data_left_.position = control_data_left_.position_filter.getFiltered();
    }
    const float measured_position = control_data_left_.position_filter.getFiltered();
    left_torque = control_data_left_.position_pid_controller.computeTorque(
        control_data_left_.position, measured_position );
  } else {
    if ( control_data_left_.control_mode != MotorControlData::ControlMode::VELOCITY ) {
      control_data_left_.control_mode = MotorControlData::ControlMode::VELOCITY;
      control_data_left_.velocity_pid_controller.reset();
    }
    const float measured_velocity = control_data_left_.velocity_filter.getFiltered();
    left_torque = control_data_left_.velocity_pid_controller.computeTorque( velocity_.left,
                                                                            measured_velocity );
  }

  float right_torque = 0;
  if ( std::abs( velocity_.right ) < 0.1f ) {
    if ( control_data_right_.control_mode != MotorControlData::ControlMode::POSITION ) {
      control_data_right_.control_mode = MotorControlData::ControlMode::POSITION;
      control_data_right_.position_pid_controller.reset();
      control_data_right_.position = control_data_right_.position_filter.getFiltered();
    }
    const float measured_position = control_data_right_.position_filter.getFiltered();
    right_torque = control_data_right_.position_pid_controller.computeTorque(
        control_data_right_.position, measured_position );
  } else {
    if ( control_data_right_.control_mode != MotorControlData::ControlMode::VELOCITY ) {
      control_data_right_.control_mode = MotorControlData::ControlMode::VELOCITY;
      control_data_right_.velocity_pid_controller.reset();
    }
    const float measured_velocity = control_data_right_.velocity_filter.getFiltered();
    right_torque = control_data_right_.velocity_pid_controller.computeTorque( velocity_.right,
                                                                              measured_velocity );
  }
  return { left_torque, right_torque };
}

const FullMotorStatus &MotorController::update()
{
  constexpr int STATUS_TIMEOUT_MS = 50;
  debug_data_.error = MotorDebugData::Error::NO_ERROR;

  MotorCommCommand left_command;
  left_command.motor_id = 0;
  MotorCommCommand right_command;
  right_command.motor_id = 1;
  const bool left_working =
      status_age_.front_left < STATUS_TIMEOUT_MS || status_age_.rear_left < STATUS_TIMEOUT_MS;
  const bool right_working =
      status_age_.front_right < STATUS_TIMEOUT_MS || status_age_.rear_right < STATUS_TIMEOUT_MS;
  if ( !left_working || !right_working ) {
    // At least one motor on each side needs to be working, otherwise we stop
    velocity_.left = 0;
    velocity_.right = 0;
    left_command.mode = MotorMode::BRAKE;
    right_command.mode = MotorMode::BRAKE;
    // Reset all PID controllers and position filter so it will not try to jump back to a position
    // when power is restored
    control_data_left_.velocity_pid_controller.reset();
    control_data_right_.velocity_pid_controller.reset();
    initialized_position_ = false;
    debug_data_.error = MotorDebugData::Error::NO_MOTOR_STATUS;
  } else {
    Torque torque = computeTorque();
    left_command.mode = std::abs( torque.left ) > MIN_TORQUE ? MotorMode::FOC : MotorMode::BRAKE;
    left_command.torque = torque.left;
    right_command.mode = std::abs( torque.right ) > MIN_TORQUE ? MotorMode::FOC : MotorMode::BRAKE;
    right_command.torque = torque.right;
    if ( std::abs( left_command.torque ) > MOTOR_TORQUE_LIMIT ) {
      left_command.torque = std::copysign( MOTOR_TORQUE_LIMIT, left_command.torque );
    }
    if ( std::abs( right_command.torque ) > MOTOR_TORQUE_LIMIT ) {
      right_command.torque = std::copysign( MOTOR_TORQUE_LIMIT, right_command.torque );
    }
  }

  const bool front_working =
      status_age_.front_left < STATUS_TIMEOUT_MS || status_age_.front_right < STATUS_TIMEOUT_MS;
  const bool rear_working =
      status_age_.rear_left < STATUS_TIMEOUT_MS || status_age_.rear_right < STATUS_TIMEOUT_MS;
  constexpr int MAX_RESET_SKIP_COUNT = 10;

  MotorCommStatus left_status;
  MotorCommStatus right_status;
  if ( front_working || ++reset_skip_count_front_ > MAX_RESET_SKIP_COUNT ) {
    // When communication fails, try to skip commands for a few cycles so if motor comm is misaligned it has time to recover
    reset_skip_count_front_ = 0;
    front_motor_comm_->sendReceive( left_command, right_command, left_status, right_status );
  } else {
    front_motor_comm_->resetComm();
  }
  motor_status_.front_left = toMotorStatus( left_status );
  motor_status_.front_left.valid &= left_status.motor_id == 0;
  motor_status_.front_right = toMotorStatus( right_status );
  motor_status_.front_right.valid &= right_status.motor_id == 1;
  status_debug_.front_left_valid.addValue( motor_status_.front_left.valid ? 1 : 0 );
  status_debug_.front_right_valid.addValue( motor_status_.front_right.valid ? 1 : 0 );

  left_status = {};
  right_status = {};
  if ( rear_working || ++reset_skip_count_rear_ > MAX_RESET_SKIP_COUNT ) {
    reset_skip_count_rear_ = 0;
    rear_motor_comm_->sendReceive( left_command, right_command, left_status, right_status );
  } else {
    rear_motor_comm_->resetComm();
  }
  motor_status_.rear_left = toMotorStatus( left_status );
  motor_status_.rear_left.valid &= left_status.motor_id == 0;
  motor_status_.rear_right = toMotorStatus( right_status );
  motor_status_.rear_right.valid &= right_status.motor_id == 1;
  status_debug_.rear_left_valid.addValue( motor_status_.rear_left.valid ? 1 : 0 );
  status_debug_.rear_right_valid.addValue( motor_status_.rear_right.valid ? 1 : 0 );

  if ( !initialized_position_ ) {
    control_data_left_.position_filter.reset();
    control_data_right_.position_filter.reset();
    // If at least one motor on each side is valid, we can initialize the position
    if ( ( motor_status_.front_left.valid || motor_status_.front_right.valid ) &&
         ( motor_status_.rear_left.valid || motor_status_.rear_right.valid ) ) {
      initialized_position_ = true;
      control_data_left_.position = control_data_left_.position_filter.getFiltered();
      control_data_right_.position = control_data_right_.position_filter.getFiltered();
    }
  }
  control_data_left_.position_filter.addMeasurements( motor_status_.front_left,
                                                      motor_status_.rear_left );
  control_data_right_.position_filter.addMeasurements( motor_status_.front_right,
                                                       motor_status_.rear_right );
  control_data_left_.velocity_filter.addMeasurements( motor_status_.front_left,
                                                      motor_status_.rear_left );
  control_data_right_.velocity_filter.addMeasurements( motor_status_.front_right,
                                                       motor_status_.rear_right );
  motor_status_.velocity_left = control_data_left_.velocity_filter.getFiltered();
  motor_status_.velocity_right = control_data_right_.velocity_filter.getFiltered();

  if ( motor_status_.front_left.valid )
    status_age_.front_left = 0;
  if ( motor_status_.front_right.valid )
    status_age_.front_right = 0;
  if ( motor_status_.rear_left.valid )
    status_age_.rear_left = 0;
  if ( motor_status_.rear_right.valid )
    status_age_.rear_right = 0;

  motor_status_.front_left.target_torque =
      left_command.mode == MotorMode::FOC ? left_command.torque : 0;
  motor_status_.rear_left.target_torque = motor_status_.front_left.target_torque;
  motor_status_.front_left.target_velocity = velocity_.left;
  motor_status_.rear_left.target_velocity = velocity_.left;

  motor_status_.front_right.target_torque =
      right_command.mode == MotorMode::FOC ? right_command.torque : 0;
  motor_status_.rear_right.target_torque = motor_status_.front_right.target_torque;
  motor_status_.front_right.target_velocity = velocity_.right;
  motor_status_.rear_right.target_velocity = velocity_.right;

  motor_status_.front_left.age_ms = status_age_.front_left;
  motor_status_.front_right.age_ms = status_age_.front_right;
  motor_status_.rear_left.age_ms = status_age_.rear_left;
  motor_status_.rear_right.age_ms = status_age_.rear_right;

  status_debug_.ages.push( elapsedMillis() );
  long status_age_ms = status_debug_.ages.front();
  debug_data_.status.freq_front_left =
      status_debug_.front_left_valid.getSum() * 1000 / status_age_ms; // Msg/ms = Msg/s * 1000ms/s
  debug_data_.status.freq_front_right =
      status_debug_.front_right_valid.getSum() * 1000 / status_age_ms;
  debug_data_.status.freq_rear_left = status_debug_.rear_left_valid.getSum() * 1000 / status_age_ms;
  debug_data_.status.freq_rear_right = status_debug_.rear_right_valid.getSum() * 1000 / status_age_ms;
  debug_data_.left_velocity_pid = control_data_left_.velocity_pid_controller.debugData();
  debug_data_.right_velocity_pid = control_data_right_.velocity_pid_controller.debugData();
  debug_data_.left_position_pid = control_data_left_.position_pid_controller.debugData();
  debug_data_.right_position_pid = control_data_right_.position_pid_controller.debugData();
  return motor_status_;
}
