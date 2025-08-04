#pragma once

#include <Arduino.h>

// LED
struct StatusLED {
  enum Speed { SLOW = 1000, FAST = 200 };

  StatusLED( int pin ) : led_pin_( pin ) { pinMode( pin, OUTPUT ); }

  void update()
  {
    if ( last_toggle >= speed ) {
      last_toggle = 0;
      last_led_state = !last_led_state;
      digitalWrite( led_pin_, last_led_state ? LOW : HIGH );
    }
  }

  int led_pin_;
  int led_counter = 0;
  elapsedMillis last_toggle;
  bool last_led_state = false;
  Speed speed = SLOW;
} status_led( LED_BUILTIN );
