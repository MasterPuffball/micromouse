#pragma once

#include <Arduino.h>
#include "Constants.h"

namespace mtrn3100 {

class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2, int num);
    void readEncoder();
    float getRotation();
    float getRotationDegrees();
    long getCount();
    void flip();

private:
    static void readEncoderISR1();
    static void readEncoderISR2();

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile long count = 0;
    bool isFlipped = false;

private:
    static Encoder* instance1;
    static Encoder* instance2;
};

}  // namespace mtrn3100
