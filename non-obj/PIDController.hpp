#pragma once

#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Compute the output signal required from the current/actual value.
    // Outputs positive if wants to move forward
    float compute(float input) {
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        error = setpoint - (input - zero_ref);

        integral += min(error,30)*dt;
        derivative = (error - prev_error) / dt;
        output = kp * error + ki * integral + kd * derivative;

        prev_error = error;
        // Serial.println(String("Input is: ") + input);
        // Serial.println(String("Current error: ") + kp*error);
        // Serial.println(String("Current integral: ") + ki*integral);
        // Serial.println(String("Current differential: ") + kd*derivative);

        return output;
    }


    // Outputs positive if wants to turn left
    float computeDir(float input) {
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        error = getDirError(input);
        
        integral += min(error,30)*dt;
        derivative = (error - prev_error) / dt;
        output = kp * error + ki * integral + kd * derivative;

        prev_error = error;
        // Serial.println(String("Input is: ") + input);
        // Serial.println(String("Current Dir error: ") + kp*error);
        // Serial.println(String("Current Dir integral: ") + ki*integral);
        // Serial.println(String("Current Dir differential: ") + kd*derivative);

        return output;
    }

    float getDirError(float angle) {
        float normalized = fmod(angle - setpoint, 360.0f);

        if (normalized > 180) {
            normalized -= 360.0;
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

    // This must be called before trying to achieve a setpoint.
    // The first argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
      prev_time = micros();
      zero_ref = zero;
      setpoint = target;

      error = target-zero; 
      integral = 0;
      derivative = 0;
    }

public:
    uint32_t prev_time, curr_time = micros();
    float dt;

private:
    float kp, ki, kd;
    float error, derivative, integral, output;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;
};

}  // namespace mtrn3100
