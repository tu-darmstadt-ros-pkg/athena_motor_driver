#include "pid_controller.h"

PIDController::PIDController( float kp, float ki, float kd, float min_output, float max_output,
                              float max_output_change )
    : kp_( kp ), ki_( ki ), kd_( kd ), max_output_( max_output ), min_output_( min_output ),
      max_output_change_( max_output_change ), first_compute_( true )
{
}

void PIDController::setGains( float kp, float ki, float kd )
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  integral_ = 0; // Reset integral to avoid sudden jumps when changing gains
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
  float dt = float( elapsed_ ) / 1E6f;
  elapsed_ = 0;
  if ( first_compute_ ) {
    last_input_ = current;
    dt = 0;
    first_compute_ = false;
  }

  const float error = goal - current;
  integral_ += error * dt;
  const float derivative = dt <= 0 ? 0 : ( error - last_error_ ) / dt;

  float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  debug_data_.raw_output = output;
  if ( std::abs( goal ) > 0.1 ) {
    output += ( std::signbit( goal ) ? -1 : 1 ) * 0.3f; // Feedforward term to avoid deadband
  }
  output = constrain( output, min_output_, max_output_ );
  output = constrain( output, last_output_ - max_output_change_, last_output_ + max_output_change_ );

  last_input_ = current;
  last_error_ = error;
  last_output_ = output;

  debug_data_.goal = goal;
  debug_data_.current = current;
  debug_data_.dt = dt;
  debug_data_.error = error;
  debug_data_.derivative = derivative;
  debug_data_.integral = integral_;
  debug_data_.output = output;

  return output;
}
