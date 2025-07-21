#pragma once

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {

// The motor class is a simple interface designed to assist in motor control
// You may choose to impliment additional functionality in the future such as dual motor or speed control 
class Motor {
public:
    Motor(uint8_t pwm_pin, uint8_t in2) :  pwm_pin(pwm_pin), dir_pin(in2) {
       pinMode(pwm_pin, OUTPUT);
       pinMode(dir_pin, OUTPUT);
    }

    // Sets pwm directly
    void setPWM(int16_t pwm) {
        analogWrite(pwm_pin, min(abs(pwm), 255));
        if (isFlipped) {
          pwm = -pwm;
        }
        digitalWrite(dir_pin, (pwm < 0 ? LOW : HIGH));
    }

    // Takes a percentage speed
    void setSpeed(int16_t speed) {
      setPWM(map(constrain(speed, -100, 100), -100, 100, -255, 255));
    }

    // Flips the direction of the motor (based on mounting), returns whether it is or isnt flipped (for debugging)
    bool flip() {
      return (isFlipped = !isFlipped);
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
    bool isFlipped = false;
};

}
