#include "pid_controller.h"

PIDController::PIDController( float kp, float ki, float kd, float min_output, float max_output,
                              float max_output_change )
    : kp_( kp ), ki_( ki ), kd_( kd ), max_output_( max_output ), min_output_( min_output ),
      max_output_change_( max_output_change ), last_input_( 0 ), integral_( 0 ), last_error_( 0 ),
      first_compute_( true )
{
}

void PIDController::setGains( float kp, float ki, float kd )
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setOutputLimits( float min_output, float max_output )
{
  min_output_ = min_output;
  max_output_ = max_output;
}

void PIDController::reset()
{
  last_input_ = 0;
  integral_ = 0;
  last_error_ = 0;
  first_compute_ = true;
}

float PIDController::computeTorque( float goal, float current )
{
  float dt = elapsed_ / 1E6;
  elapsed_ = 0;
  if ( first_compute_ ) {
    last_input_ = current;
    dt = 0;
    first_compute_ = false;
  }

  float error = goal - current;
  float p_term = kp_ * error;

  integral_ += ki_ * error * dt;

  float d_term = 0;
  if ( dt > 0 ) {
    d_term = kd_ * ( error - last_error_ ) / dt;
  }

  float output = p_term + integral_ + d_term;
  output = constrain( output, min_output_, max_output_ );
  output = constrain( output, last_output_ - max_output_change_, last_output_ + max_output_change_ );

  last_input_ = current;
  last_error_ = error;
  last_output_ = output;

  return output;
}
