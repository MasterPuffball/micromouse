#pragma once

#include <VL6180X.h>
#include <Wire.h>

int leftSensorPin = A0; // ENABLE PIN FOR SENSOR POSITION A2
int frontSensorPin = A1; // ENABLE PIN FOR SENSOR POSITION A3
int rightSensorPin = A2; // ENABLE PIN FOR SENSOR POSITION A4

VL6180X leftSensor;
VL6180X frontSensor;
VL6180X rightSensor;


void lidarSetup() {
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
  // Serial.print("Left: ");
  // Serial.print(leftSensor.readRangeSingleMillimeters());
  // Serial.print(" | ");
  return leftSensor.readRangeSingleMillimeters();
}

int getFrontDist() {
  // Serial.print("Front: ");
  // Serial.print(frontSensor.readRangeSingleMillimeters());
  // Serial.print(" | ");
  return frontSensor.readRangeSingleMillimeters();
}

int getRightDist() {
  // Serial.print("Right: ");
  // Serial.print(rightSensor.readRangeSingleMillimeters());
  // Serial.println();
  return rightSensor.readRangeSingleMillimeters();
}