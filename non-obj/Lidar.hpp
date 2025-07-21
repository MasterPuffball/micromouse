#pragma once

#include <VL6180X.h>
#include <Wire.h>

int leftSensorPin = A0; // ENABLE PIN FOR SENSOR POSITION A2
int frontSensorPin = A1; // ENABLE PIN FOR SENSOR POSITION A3
int rightSensorPin = A2; // ENABLE PIN FOR SENSOR POSITION A4

VL6180X leftSensor;
VL6180X frontSensor;
VL6180X rightSensor;


void initLidar() {
    Wire.begin();
    Serial.begin(9600);

    // SET UP ENABLE PINS AND DISABLE SENSORS
    pinMode(leftSensorPin, OUTPUT);
    pinMode(frontSensorPin, OUTPUT);
    pinMode(rightSensorPin, OUTPUT);
    digitalWrite(leftSensorPin, LOW);
    digitalWrite(frontSensorPin, LOW);
    digitalWrite(rightSensorPin, LOW);

    // ENABLE LEFT SENSOR
    digitalWrite(leftSensorPin, HIGH);
    delay(50);
    leftSensor.init();
    leftSensor.configureDefault();
    leftSensor.setTimeout(250);
    leftSensor.setAddress(0x54);
    delay(50);
  
    // ENABLE FRONT SENSOR
    digitalWrite(frontSensorPin, HIGH);
    delay(50);
    frontSensor.init();
    frontSensor.configureDefault();
    frontSensor.setTimeout(250);
    frontSensor.setAddress(0x56);

    // ENABLE RIGHT SENSOR
    digitalWrite(rightSensorPin, HIGH);
    delay(50);
    rightSensor.init();
    rightSensor.configureDefault();
    rightSensor.setTimeout(250);
    rightSensor.setAddress(0x58);
}

int getLeftDist() {
  int reading = leftSensor.readRangeSingleMillimeters();
  Serial.println(String("Left: ") + reading);
  return reading;
}

int getFrontDist() {
  int reading = frontSensor.readRangeSingleMillimeters();
  Serial.println(String("Front: ") + reading);
  return reading;
}

int getRightDist() {
  int reading = rightSensor.readRangeSingleMillimeters();
  Serial.println(String("right: ") + reading);
  return reading;
}

// void lidarLoop() {
//   Serial.print(leftSensor.readRangeSingleMillimeters());
//   Serial.print(" | ");
//   Serial.print(frontSensor.readRangeSingleMillimeters());
//   Serial.print(" | ");
//   Serial.print(rightSensor.readRangeSingleMillimeters());
//   Serial.println();
  
//   if (leftSensor.timeoutOccurred()) { Serial.print("Left Sensor TIMEOUT"); }
//   if (frontSensor.timeoutOccurred()) { Serial.print("Front Sensor TIMEOUT"); }
//   if (rightSensor.timeoutOccurred()) { Serial.print("Right Sensor TIMEOUT"); }
//   delay(100);
// }