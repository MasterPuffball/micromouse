#pragma once

#include <Arduino.h>

#include "math.h"
#include "PIDController.hpp"
#include "Motor.hpp"
#include "Encoder.hpp"

namespace mtrn3100 {
  class Wheel {
    public:
        Wheel(mtrn3100::PIDController* controller, mtrn3100::Motor* motor, mtrn3100::Encoder* encoder, float coef) :  controller(controller), motor(motor), encoder(encoder), gearbox_coef(coef) {
        }

        void setTarget(int16_t dist) {
          controller->zeroAndSetTarget(getDistanceMoved(), dist);
          isFinished = false;
          countWithinTolerance = 0;
        }

        float compute(float pos) {
          return controller->compute(pos);
        }

        float getMotorSignal(float signal, float speed) {
          return constrain(signal, -100, 100) * gearbox_coef * speed;
        }

        float getError() {
          return controller->getError();
        }

        bool isFinishedMove() {
          if (abs(getError()) <= tolerance) {
            countWithinTolerance++;
          }
          else {
            countWithinTolerance = 0;
          }

          if (countWithinTolerance >= acceptableCounts || isFinished) {
            motor->setSpeed(0);
            isFinished = true;
          }

          return isFinished;
        }

        float getDistanceMoved() {
          return (distanceMoved  = encoder->getRotation() * wheel_radius);
        }

        void setSpeed(float speed) {
          motor->setSpeed(speed);
        }

    private:
        const mtrn3100::PIDController* controller;
        const mtrn3100::Motor* motor;
        const mtrn3100::Encoder* encoder;
        float distanceMoved = 0;
        const float wheel_radius = 16; //In millis
        float gearbox_coef;
        const int acceptableCounts = 1;
        const float tolerance = 5;
        bool isFinished = false;
        const float minSignal = 5.06;
        int countWithinTolerance = 0;
  };
}
