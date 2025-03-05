#include <Arduino.h>

#include "../../athena_motor_interface/athena_motor_interfaces.h"
#include <memory>

class MotorComm;

// In m/s^2. To reach 1U/s=2*pi rad/s in 0.5 seconds, acceleration would be 2m/s^2
const float MAX_ACCELERATION = 8.0;
const float MAX_DECELERATION = 30.0;

class MotorController
{
public:
  MotorController();

  ~MotorController();

  void init( std::shared_ptr<MotorComm> front_comm, std::shared_ptr<MotorComm> back_comm );

  void setVelocities( float left_velocity, float right_velocity );

  void stop();

  void update();

  MotorComm &frontComm() { return *front_motor_comm_; }

  MotorComm &backComm() { return *back_motor_comm_; }

  const FullMotorStatus &status()
  {
    has_new_status_ = false;
    return motor_status_;
  }

  bool hasNewStatus() { return has_new_status_; }

private:
  float target_left_velocity_ = 0;
  float target_right_velocity_ = 0;

  float left_velocity_ = 0;
  float right_velocity_ = 0;

  elapsedMillis time_since_last_command_ = 0;
  bool has_new_status_ = false;

  FullMotorStatus motor_status_;
  std::shared_ptr<MotorComm> front_motor_comm_;
  std::shared_ptr<MotorComm> back_motor_comm_;
};
