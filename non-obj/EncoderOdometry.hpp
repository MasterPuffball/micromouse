#pragma once

#include <Arduino.h>
#include "Constants.h"
#include "Encoder.hpp"

namespace mtrn3100 {
  class EncoderOdometry {
    public:
      EncoderOdometry(mtrn3100::Encoder* leftEncoder, mtrn3100::Encoder* rightEncoder) : leftEncoder(leftEncoder), rightEncoder(rightEncoder) {}

      void begin() {
        reset();
      }

      void reset() {
        leftZero = leftEncoder->getCount();
        rightZero = rightEncoder->getCount();

        x = 0.0f;
        y = 0.0f;
        theta = 0.0f;
        prevLeftTicks = 0;
        prevRightTicks = 0;
        leftTicks = 0;
        rightTicks = 0;
        lastUpdateTime = millis();
      }

      void update() {
        leftTicks = leftEncoder->getCount() - leftZero;
        rightTicks = rightEncoder->getCount() - rightZero;

        long dLeft = leftTicks - prevLeftTicks;
        long dRight = rightTicks - prevRightTicks;

        prevLeftTicks = leftTicks;
        prevRightTicks = rightTicks;

        float dL_mm = dLeft * MM_PER_TICK;
        float dR_mm = dRight * MM_PER_TICK;

        float dCenter = (dL_mm + dR_mm) / 2.0f;
        float dTheta  = (dR_mm - dL_mm) / AXLE_LENGTH;

        theta += dTheta;
        if (theta > PI) {
          theta -= TWO_PI;
        }  
        else if (theta <= -PI) {
          theta += TWO_PI;
        }

        x += dCenter * cos(theta);
        y += dCenter * sin(theta);

        lastUpdateTime = millis();
      }

      float EncoderOdometry::getX() {
        update();
        return x;
      }

      float EncoderOdometry::getY() {
        update();
        return y; 
      }

      float EncoderOdometry::getTheta(){
        update();
        return theta * (180.0f / PI);
      }

      float getLeftSpeedMMs() const {
          static unsigned long lastTime = 0;
          static long lastTicks = 0;

          unsigned long now = millis();
          float dt = (now - lastTime) / 1000.0f;

          if (dt <= 0.0f) return 0.0f;

          long currentTicks = leftTicks;
          float speed = (currentTicks - lastTicks) * MM_PER_TICK / dt;

          lastTicks = currentTicks;
          lastTime = now;

          return speed;
      }

      float getRightSpeedMMs() const {
          static unsigned long lastTime = 0;
          static long lastTicks = 0;

          unsigned long now = millis();
          float dt = (now - lastTime) / 1000.0f;

          if (dt <= 0.0f) return 0.0f;

          long currentTicks = rightTicks;
          float speed = (currentTicks - lastTicks) * MM_PER_TICK / dt;

          lastTicks = currentTicks;
          lastTime = now;

          return speed;
      }

    private:
      float x = 0.0f;
      float y = 0.0f;
      float theta = 0.0f;

      long prevLeftTicks = 0;
      long prevRightTicks = 0;

      long leftTicks = 0;
      long rightTicks = 0;

      long leftZero = 0;
      long rightZero = 0;

      unsigned long lastUpdateTime = 0;

      const mtrn3100::Encoder* leftEncoder;
      const mtrn3100::Encoder* rightEncoder;
  };
} // namespace mtrn3100