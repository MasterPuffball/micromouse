#pragma once

#include <math.h>
#include "Constants.h"

namespace mtrn3100 {
class PIDController {
public:
    PIDController(float kp, float ki, float kd, float isDir = false) : kp(kp), ki(ki), kd(kd), isDir(isDir) {}

    // Compute the output signal required from the current/actual value.
    // Outputs positive if wants to move forward
    float compute(float input) {
      uint32_t curr_time = micros();
      float dt = static_cast<float>(curr_time - prev_time) / 1e6;
      prev_time = curr_time;

      // Outputs positive if wants to turn left
      if (isDir) {
        error = getDirError(input);
      }
      else {
        error = setpoint - (input - zero_ref);
      }
      
      integral += constrain(error, -INCREMENT_CLAMP, INCREMENT_CLAMP) * dt;
      integral = constrain(integral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

      derivative = 0;
      if (fabs(error) >  DERIVATIVE_DEADZONE) {
        derivative = (error - prev_error) / dt;
      }
      
      output = kp * error + ki * integral + kd * derivative;

      prev_error = error;

      recordError(error);

      return output;
    }

    float getDirError(float angle) {
      float normalized = fmod(angle - setpoint, 360.0f);

      if (normalized > 180) {
        normalized -= 360.0;
      }

      if (normalized < -180) {
        normalized += 360.0;
      }

      return normalized;
    }

    void tune(float p, float i, float d) {
      kp = p;
      ki = i;
      kd = d;
    }

    // Function used to return the last calculated error. 
    // The error is the difference between the desired position and current position. 
    float getError() {
      return error;
    }

    float getDerivative() {
      return derivative;
    }

    float getIntegral() {
      return integral;
    }

    float getP() {
      return kp;
    }

    float getI() {
      return ki;
    }

    float getD() {
      return kd;
    }

    float getTarget() {
      return setpoint;
    }

    float getZero() {
      return zero_ref;
    }

    int getErrorSampleCount() {
      return errorIndex;
    }

    float getErrorAtIndex(int idx) {
      if (idx < 0 || idx >= NUM_STEADY) return 0.0f;
      return errorHistory[idx];
    }

    // This must be called before trying to achieve a setpoint.
    // The first argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
      prev_time = micros();
      zero_ref = zero;
      setpoint = target;

      if (isDir) {
        error = getDirError(zero);
      } else {
        error = target - zero;
      }

      integral = 0;
      derivative = 0;

      clearErrorHistory();
    }

    bool isWithin(float tolerance) {
      return isSteady() && abs(error) < tolerance;
    }

    bool isSteady() const {
      if (errorIndex < NUM_STEADY) {
        return false; // not enough samples yet
      }

      float minError = errorHistory[0];
      float maxError = errorHistory[0];

      for (int i = 0; i < NUM_STEADY; i++) {
        if (errorHistory[i] < minError) {
          minError = errorHistory[i];
        }
        if (errorHistory[i] > maxError) {
          maxError = errorHistory[i];
        }
      }

      return (maxError - minError) <= STEADY_TOLERANCE;
    }

    void recordError(float newError) {
      errorHistory[errorIndex % NUM_STEADY] = newError;
      errorIndex++;
    }

    void clearErrorHistory() {
      errorIndex = 0;

      for (int i = 0; i < NUM_STEADY; i++) {
        errorHistory[i] = 0;
      }
    }

public:
    uint32_t prev_time = micros();

private:
    float kp, ki, kd;
    bool isDir;
    float error, derivative, integral, output;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;
    
    float errorHistory[NUM_STEADY] = {0};
    int errorIndex = 0;
};

}  // namespace mtrn3100
