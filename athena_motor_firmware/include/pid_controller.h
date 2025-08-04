#ifndef ATHENA_MOTOR_FIRMWARE_PID_CONTROLLER_H
#define ATHENA_MOTOR_FIRMWARE_PID_CONTROLLER_H

#include "athena_motor_interface/athena_motor_interfaces.h"
#include <elapsedMillis.h>

class PIDController
{
public:
  PIDController( float kp, float ki, float kd, float min_output, float max_output,
                 float max_output_change );
  void setGains( float kp, float ki, float kd );
  void setOutputLimits( float min_output, float max_output );
  void reset();

  //! Compute the torque required to reach the goal velocity
  float computeTorque( float goal, float current );

  const PIDDebugData &debugData() const { return debug_data_; }

private:
  PIDDebugData debug_data_;
  elapsedMicros elapsed_;
  float kp_;
  float ki_;
  float kd_;
  float max_output_;
  float min_output_;
  float max_output_change_;
  float last_input_ = 0;
  float last_output_ = 0;
  float integral_ = 0;
  float last_error_ = 0;
  bool first_compute_;
};

#endif // ATHENA_MOTOR_FIRMWARE_PID_CONTROLLER_H