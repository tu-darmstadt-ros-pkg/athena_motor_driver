#include <Arduino.h>

#include "athena_motor_interface/athena_motor_interfaces.h"
#include "math/mean_filter.h"
#include "math/ring_buffer.h"
#include "pid_controller.h"
#include "position_measurement_filter.hpp"
#include "velocity_measurement_filter.hpp"
#include <memory>

class MotorComm;

// In m/s^2. To reach 1U/s=2*pi rad/s in 0.5 seconds, acceleration would be 2m/s^2
const float MAX_ACCELERATION = 8.0;
const float MAX_DECELERATION = 24.0;

const float MIN_TORQUE = 0.5f;         // Min torque to issue movement command rather than brake
const float MOTOR_TORQUE_LIMIT = 30.0; // Nm
// With 500 Hz loop frequency, this is a max change rate of 200Nm/s meaning 150ms for 0 to full torque
const float MAX_TORQUE_CHANGE = 0.4;

class MotorController
{
public:
  MotorController();

  ~MotorController();

  void init( std::shared_ptr<MotorComm> front_comm, std::shared_ptr<MotorComm> back_comm );

  void setCommand( const MotorCommand &command );

  void setPositionPIDGains( const PIDGains &left_pid_gains, const PIDGains &right_pid_gains );

  void setVelocityPIDGains( const PIDGains &left_pid_gains, const PIDGains &right_pid_gains );

  void setDisableAccelerationLimiting( bool disable ) { disable_acceleration_limiting_ = disable; }

  void stop();

  const FullMotorStatus &update();

  MotorComm &frontComm() { return *front_motor_comm_; }

  MotorComm &backComm() { return *rear_motor_comm_; }

  MotorError::Error getError()
  {
    // Removed status age check since the age is transmitted now.
    // This could be used in the future if another error is needed.
    return MotorError::Error::NO_ERROR;
  }

  const MotorDebugData &debugData() const { return debug_data_; }

private:
  struct Torque {
    float left = 0;
    float right = 0;

    Torque( float left, float right ) : left( left ), right( right ) { }

    Torque() = default;
  };

  Torque computeTorque();

  struct Velocity {
    float left = 0;
    float right = 0;
  };

  MotorCommand command_;
  Velocity target_velocity_;
  Velocity velocity_;
  elapsedMicros time_since_last_command_ = 0;

  struct CommStatusDebug {
    RingBuffer<elapsedMillis, 50> ages;
    MeanFilter<uint8_t, 50> front_left_valid;
    MeanFilter<uint8_t, 50> front_right_valid;
    MeanFilter<uint8_t, 50> rear_left_valid;
    MeanFilter<uint8_t, 50> rear_right_valid;
  } status_debug_;

  MotorDebugData debug_data_;

  struct MotorControlData {
    enum class ControlMode { POSITION, VELOCITY };
    ControlMode control_mode = ControlMode::POSITION;
    PositionMeasurementFilter position_filter;
    VelocityMeasurementFilter velocity_filter;
    PIDController velocity_pid_controller;
    PIDController position_pid_controller;
    float position;

    MotorControlData();
  };

  MotorControlData control_data_left_;
  MotorControlData control_data_right_;
  FullMotorStatus motor_status_;

  struct {
    elapsedMillis front_left;
    elapsedMillis front_right;
    elapsedMillis rear_left;
    elapsedMillis rear_right;
  } status_age_;

  std::shared_ptr<MotorComm> front_motor_comm_;
  std::shared_ptr<MotorComm> rear_motor_comm_;
  int reset_skip_count_front_ = 0; // If motor comm fails try to skip communication for a few times
  int reset_skip_count_rear_ = 0;
  bool disable_acceleration_limiting_ = false; // For tuning PID controller
  bool initialized_position_ = false;
};
