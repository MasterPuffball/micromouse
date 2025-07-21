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
        }

        float compute(float pos) {
          return controller->compute(pos);
        }

        float getMotorSignal(float signal, float speed) {
          return min(signal,100) * gearbox_coef * speed;
        }

        float getError() {
          return controller->getError();
        }

        void updateTolerance() {
          if (abs(controller->getError()) <= tolerance) {
            countWithinTolerance++;
          }
          else {
            countWithinTolerance = 0;
          }
        }

        bool isFinishedMove() {
          if (countWithinTolerance >= acceptableCounts) {
            countWithinTolerance = 0;
            motor->setSpeed(0);
            return true;
          }
          return false;
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
        const int acceptableCounts = 200;
        const float tolerance = 5;
        const float minSignal = 5;
        int countWithinTolerance = 0;
        bool hasStartedMove = false;
  };
}
