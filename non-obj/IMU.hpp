#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

#define AVERAGE_AMOUNT 5

namespace mtrn3100 {

struct IMUData {
  float x;
  float y;
  float z;
};

class IMU {
public:
    IMU(TwoWire& wire = Wire) : mpu(wire) {}

    bool begin() {
      Serial.println("Beginning IMU");
      if (mpu.begin() != 0) {
        Serial.println(F("IMU init failed"));
        return false;
      }
    
      Serial.println(F("Calculating offsets, do not move MPU6050"));
      mpu.calcOffsets(); // gyro and accelero
      Serial.println("Done!\n");

      return true;
    }

    IMUData read() {
      IMUData data;
      mpu.update();
      
      data.x = normalizeAngle(mpu.getAngleX());
      data.y = normalizeAngle(mpu.getAngleY());
      data.z = normalizeAngle(mpu.getAngleZ());
      x = data.x;
      y = data.y;
      z = data.z;

      return data;
    }

    float getDirection() {
      float direction = 0;

      for (int i = 0; i < AVERAGE_AMOUNT; i++) {
        direction += read().z;
      }
      
      return direction/AVERAGE_AMOUNT;
    }

    float normalizeAngle(float angle) {
      float normalized = fmod(angle, 360.0f);

      return (normalized < 0 ? (normalized + 360.0f) : normalized);
    }

    void printCurrentData() {
      read();
      
      Serial.print("X : ");
      Serial.print(x);
      Serial.print("\tY : ");
      Serial.print(y);
      Serial.print("\tZ : ");
      Serial.println(z);
    }

  private:
    MPU6050 mpu;
    bool initialized = true;
    float x;
    float y;
    float z;
};
} // namespace mtrn3100