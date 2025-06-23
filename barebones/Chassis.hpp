#pragma once

#include <Arduino.h>

#include "Wheel.hpp"
#include "math.h"

namespace mtrn3100 {

  // The motor class is a simple interface designed to assist in motor control
  // You may choose to impliment additional functionality in the future such as dual motor or speed control 
  class Chassis {
    public:
        Chassis(mtrn3100::Wheel* left, mtrn3100::Wheel* right) :  left(left), right(right) {
        }

        void moveForwardDistance(uint16_t dist) {
          left->setTarget(dist);
          right->setTarget(dist);

          while (!left->isFinishedMove() && !right->isFinishedMove()) {
            left->moveDistanceMillis(dist, 0.5);
            right->moveDistanceMillis(dist, 0.5);
          }

          left->setSpeed(0);
          right->setSpeed(0);
        }

    private:
        const mtrn3100::Wheel* left;
        const mtrn3100::Wheel* right;
  };
}
