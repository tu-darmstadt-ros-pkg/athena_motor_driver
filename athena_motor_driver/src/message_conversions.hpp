//
// Created by stefan on 16.07.25.
//

#ifndef ATHENA_MOTOR_DRIVER_MESSAGE_CONVERSIONS_HPP
#define ATHENA_MOTOR_DRIVER_MESSAGE_CONVERSIONS_HPP

#include <athena_motor_interface/athena_motor_interfaces.h>
#include <athena_motor_interface/msg/debug_data.hpp>

inline athena_motor_interface::msg::PIDDebugData toMsg( const PIDDebugData &pid_debug )
{
  athena_motor_interface::msg::PIDDebugData msg;
  msg.goal = pid_debug.goal;
  msg.current = pid_debug.current;
  msg.dt = pid_debug.dt;
  msg.error = pid_debug.error;
  msg.derivative = pid_debug.derivative;
  msg.integral = pid_debug.integral;
  msg.raw_output = pid_debug.raw_output;
  msg.output = pid_debug.output;
  return msg;
}

inline athena_motor_interface::msg::DebugData toMsg( const MotorDebugData &debug_data )
{
  athena_motor_interface::msg::DebugData msg;
  msg.left_velocity_pid = toMsg( debug_data.left_velocity_pid );
  msg.right_velocity_pid = toMsg( debug_data.right_velocity_pid );
  msg.left_position_pid = toMsg( debug_data.left_position_pid );
  msg.right_position_pid = toMsg( debug_data.right_position_pid );
  msg.status.freq_front_left = debug_data.status.freq_front_left;
  msg.status.freq_front_right = debug_data.status.freq_front_right;
  msg.status.freq_rear_left = debug_data.status.freq_rear_left;
  msg.status.freq_rear_right = debug_data.status.freq_rear_right;
  msg.average_loop_time_us = debug_data.average_loop_time_us;
  msg.error_code = static_cast<uint8_t>( debug_data.error );
  return msg;
}

#endif // ATHENA_MOTOR_DRIVER_MESSAGE_CONVERSIONS_HPP
