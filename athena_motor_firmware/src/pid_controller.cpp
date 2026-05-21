#include "pid_controller.h"
#include "config.h"

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

void PIDController::setFeedForwardGains( float k_v, float k_s, float ramp_width )
{
  feed_forward_k_v_ = k_v;
  feed_forward_k_s_ = k_s;
  feed_forward_ramp_width_ = ramp_width;
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

  float p_term = kp_ * error;

  // Calculate derivative term on measurement to reduce derivative kick from setpoint changes
  const float derivative = dt <= 0 ? 0 : ( last_input_ - current ) / dt;
  float d_term = kd_ * derivative;

  // Anti-windup: Only integrate if not saturated, or if integrating reduces the saturation
  float pre_integral_output = p_term + d_term + feed_forward_k_v_ * goal;
  if ( pre_integral_output < max_output_ && pre_integral_output > min_output_ ) {
    integral_ += error * dt;
  } else if ( pre_integral_output >= max_output_ && error < 0 ) {
    integral_ += error * dt; // Allow winding down
  } else if ( pre_integral_output <= min_output_ && error > 0 ) {
    integral_ += error * dt; // Allow winding down
  }

  float i_term = ki_ * integral_;

  float output = p_term + i_term + d_term;
  debug_data_.raw_output = output;

  // Feed-forward control
  float feed_forward = 0.0f;
  if ( std::abs( goal ) > FEED_FORWARD_DEAD_ZONE ) {
    feed_forward = goal * feed_forward_k_v_;

    float ramp_factor = ( feed_forward_ramp_width_ > 0.0f )
                            ? std::min( 1.0f, std::abs( goal ) / feed_forward_ramp_width_ )
                            : 0.0f;
    feed_forward += std::copysign( feed_forward_k_s_ * ramp_factor, goal );

    output += feed_forward;
  }

  const float max_output_change = max_output_change_ * dt;
  output = constrain( output, last_output_ - max_output_change, last_output_ + max_output_change );
  output = constrain( output, min_output_, max_output_ );

  last_input_ = current;
  last_error_ = error;
  last_output_ = output;

  debug_data_.goal = goal;
  debug_data_.current = current;
  debug_data_.dt = dt;
  debug_data_.error = error;
  debug_data_.derivative = derivative;
  debug_data_.integral = integral_;
  debug_data_.feed_forward = feed_forward;
  debug_data_.output = output;

  return output;
}
