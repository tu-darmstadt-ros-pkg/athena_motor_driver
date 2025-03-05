#include <Arduino.h>

#define MAIN_LOOP_DELAY_IN_US 1000

#include "host_comm.h"
#include "motor_comm.h"
#include "motor_controller.h"
#include "status_led.h"

const int MAX_TIME_SINCE_LAST_COMMAND_IN_MILLISECONDS = 200;
const int MAX_CYCLES_SINCE_LAST_COMMAND =
    MAX_TIME_SINCE_LAST_COMMAND_IN_MILLISECONDS * 1000 / MAIN_LOOP_DELAY_IN_US;

HostComm host_comm;
int cycles_since_last_command = 0;
MotorController motor_controller;
IntervalTimer motor_timer;

FullMotorStatus motor_status;

void motorControlLoop();

void setup()
{
  pinMode( LED_BUILTIN, OUTPUT );
  digitalWrite( LED_BUILTIN, HIGH );
  // Set UART_CLK_SEL bit to 0 to enable higher baud rates
  CCM_CSCDR1 = ( CCM_CSCDR1 & ~CCM_CSCDR1_UART_CLK_SEL );
  host_comm.init();
  auto front_comm = std::make_shared<MotorComm>( &Serial1, 2 );
  auto back_comm = std::make_shared<MotorComm>( &Serial2, 9 );
  motor_controller.init( front_comm, back_comm );
  // motor_timer.priority( 0 );
  // motor_timer.begin( motorControlLoop, 1000 );

  digitalWrite( LED_BUILTIN, LOW );
}

void motorControlLoop()
{
  motor_controller.update();
  if ( motor_controller.hasNewStatus() ) {
    FullMotorStatus new_status = motor_controller.status();
    if ( new_status.front_left.valid )
      motor_status.front_left = new_status.front_left;
    if ( new_status.front_right.valid )
      motor_status.front_right = new_status.front_right;
    if ( new_status.rear_left.valid )
      motor_status.rear_left = new_status.rear_left;
    if ( new_status.rear_right.valid )
      motor_status.rear_right = new_status.rear_right;
    host_comm.sendStatus( motor_status );
  }
}

void loop()
{
  delayMicroseconds( MAIN_LOOP_DELAY_IN_US );
  if ( host_comm.hasCommand() ) {
    switch ( host_comm.getType() ) {
    case CommandType::MOTOR_COMMAND: {
      MotorCommand command = host_comm.getMotorCommand();
      motor_controller.setVelocities( command.left_velocity, command.right_velocity );
      cycles_since_last_command = 0;
      status_led.speed = StatusLED::FAST;
      break;
    }
    case CommandType::NONE:
      // Do nothing
      break;
    }
  }
  motorControlLoop();
  status_led.update();

  if ( ++cycles_since_last_command >= MAX_CYCLES_SINCE_LAST_COMMAND ) {
    cycles_since_last_command = MAX_CYCLES_SINCE_LAST_COMMAND; // Avoid overflow
    motor_controller.stop();
    status_led.speed = StatusLED::SLOW;
    return;
  }
}
