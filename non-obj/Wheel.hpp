#pragma once

#include <Arduino.h>

#include "math.h"
#include "PIDController.hpp"
#include "Motor.hpp"
#include "Encoder.hpp"

namespace mtrn3100 {
  class Wheel {
    public:
        Wheel(mtrn3100::Motor* motor, mtrn3100::Encoder* encoder) : motor(motor), encoder(encoder) {
        }

        float getDistanceMoved() {
          return (distanceMoved  = encoder->getRotation() * wheel_radius);
        }

        void setSpeed(float speed) {
          motor->setSpeed(speed);
        }

    private:
        const mtrn3100::Motor* motor;
        const mtrn3100::Encoder* encoder;
        float distanceMoved = 0;
        const float wheel_radius = 16; //In millis
  };
}
