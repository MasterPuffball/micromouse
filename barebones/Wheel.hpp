#pragma once

#include <Arduino.h>

#include "math.h"
#include "PIDController.hpp"
#include "Motor.hpp"
#include "Encoder.hpp"

namespace mtrn3100 {
  class Wheel {
    public:
        Wheel(mtrn3100::PIDController controller, mtrn3100::Motor motor, mtrn3100::Encoder encoder) :  controller(controller), motor(motor), encoder(encoder) {
        }

        void moveDistanceMillis(int16_t dist) {
          controller.zeroAndSetTarget(getDistanceMoved(), dist);

          if (!isFinishedMove()) {
            float pos = getDistanceMoved();
            float intendedSignal = controller.compute(pos);
            Serial.println(String("Intended: ") + intendedSignal);
            Serial.println(String("Pos: ") + encoder.getRotationDegrees());

            motor.setSpeed(intendedSignal);
            if (controller.getError() < tolerance) {
              countWithinTolerance++;
            }
            else {
              countWithinTolerance = 0;
            }
          }
        }

        bool isFinishedMove() {
          if (countWithinTolerance >= acceptableCounts) {
            countWithinTolerance = 0;
            return true;
          }
          return false;
        }

        float getDistanceMoved() {
          return (distanceMoved  = (encoder.getRotationDegrees() * wheel_diam)/360);
        }

    private:
        const mtrn3100::PIDController controller;
        const mtrn3100::Motor motor;
        const mtrn3100::Encoder encoder;
        float distanceMoved = 0;
        const float wheel_diam = 32; //In millis
        const int acceptableCounts = 10;
        const float tolerance = 0.1;
        int countWithinTolerance = 0;
  };
}
