#pragma once

#include <Arduino.h>

// LED
struct StatusLED {
  enum Speed { SLOW = 2000 * 1000 / MAIN_LOOP_DELAY_IN_US, FAST = 200 * 1000 / MAIN_LOOP_DELAY_IN_US };
  
  StatusLED( int pin ) : led_pin_( pin ) { pinMode( pin, OUTPUT ); }

  void update() {
    if ( ++led_counter >= speed ) {
      led_counter = 0;
      last_led_state = !last_led_state;
      digitalWrite( led_pin_, last_led_state ? LOW : HIGH );
    }
  }

  int led_pin_;
  int led_counter = 0;
  bool last_led_state = false;
  Speed speed = SLOW;
} status_led( LED_BUILTIN );
