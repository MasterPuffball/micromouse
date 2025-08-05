#pragma once

#include <VL6180X.h>
#include <Wire.h>

namespace mtrn3100 {
class Lidar {
public:
    Lidar(uint8_t pin, uint8_t address) : init_pin(pin), address(address) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        delay(50);
        sensor.init();
        sensor.configureDefault();
        sensor.setTimeout(250);
        sensor.setAddress(address);
        delay(50);
    }

    int get_dist() {
        return sensor.readRangeSingleMillimeters();
    }

private:
    VL6180X sensor;
    const uint8_t init_pin;
    const uint8_t address;
};
}

