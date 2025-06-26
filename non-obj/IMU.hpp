#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

namespace mtrn3100 {

struct IMUData {
  float x;
  float y;
  float z;
};

class IMU {
public:
    IMU(TwoWire& wire = Wire) : imu(wire) {}

    bool begin() {
      if (!imu.begin()) {
        Serial.println(F("IMU init failed"));
        return false;
      }

      imu.calcOffsets();

      return true;
    }

    void update() {
      imu.update();
      x = imu.getAngleX();
      y = imu.getAngleY();
      z = imu.getAngleZ();
    }

    IMUData read() {
      IMUData data;
      data.x = normalizeAngle(imu.getAngleX());
      data.y = normalizeAngle(imu.getAngleY());
      data.z = normalizeAngle(imu.getAngleZ());
      return data;
    }

    IMUData updateRead() {
      update();
      return read();
    }

    float normalizeAngle(float angle) {
      float normalized = fmod(angle, 360.0f);

      return (normalized < 0 ? (normalized + 360.0f) : normalized);
    }

    void printCurrentData() {
      IMUData data = updateRead();
      
      Serial.print("X : ");
      Serial.print(data.x);
      Serial.print("\tY : ");
      Serial.print(data.y);
      Serial.print("\tZ : ");
      Serial.println(data.z);
    }

  private:
    MPU6050 imu;
    bool initialized = true;
    float x;
    float y;
    float z;
};
} // namespace mtrn3100