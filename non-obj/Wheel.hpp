#pragma once

#include <Arduino.h>

#include "math.h"
#include "Motor.hpp"
#include "Encoder.hpp"
#include "Constants.h"

namespace mtrn3100 {
  class Wheel {
    public:
        Wheel(mtrn3100::Motor* motor, mtrn3100::Encoder* encoder) : motor(motor), encoder(encoder) {
        }

        float getDistanceMoved() {
          return (distanceMoved  = encoder->getRotation() * WHEEL_RADIUS);
        }

        void setSpeed(float speed) {
          motor->setSpeed(speed);
        }

    private:
        const mtrn3100::Motor* motor;
        const mtrn3100::Encoder* encoder;
        float distanceMoved = 0;
  };
}
