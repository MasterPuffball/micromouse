#pragma once

#include <Arduino.h>

namespace mtrn3100 {


// The encoder class is a simple interface which counts and stores an encoders count.
// Encoder pin 1 is attached to the interupt on the arduino and used to trigger the count.
// Encoder pin 2 is attached to any digital pin and used to derrive rotation direction.
// The count is stored as a volatile variable due to the high frequency updates. 
class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2, int num) : encoder1_pin(enc1), encoder2_pin(enc2) {
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);
        
        if (num == 0) {
          instance1 = this;  // Store the instance pointer
          attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR1, RISING);
        }
        if (num == 1) {
          instance2 = this;  // Store the instance pointer
          attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR2, RISING);
        }
    }


    // Encoder function used to update the encoder
    void readEncoder() {
        noInterrupts();

        // NOTE: DO NOT PLACE SERIAL PRINT STATEMENTS IN THIS FUNCTION
        // NOTE: DO NOT CALL THIS FUNCTION MANUALLY IT WILL ONLY WORK IF CALLED BY THE INTERRUPT
        // TODO: Increase or Decrease the count by one based on the reading on encoder pin 2
        int read = digitalRead(encoder2_pin);
        
        if (read == LOW) {
          count += (isFlipped == true ? -1 : 1);
        }
        else if (read == HIGH) {
          count -= (isFlipped == true ? -1 : 1);
        }
      
        interrupts();
    }

    // Helper function which to convert encouder count to radians
    float getRotation() {
        return count*((2*PI)/counts_per_revolution);
    }

    float getRotationDegrees() {
        return ((count*360)/counts_per_revolution);
    }

    void flip() {
      isFlipped = !isFlipped;
    }

private:
    static void readEncoderISR1() {
        if (instance1 != nullptr) {
            instance1->readEncoder();
        }
    }
    static void readEncoderISR2() {
        if (instance2 != nullptr) {
            instance2->readEncoder();
        }
    }

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction;
    float position = 0;
    uint16_t counts_per_revolution = 700;
    volatile long count = 0;
    uint32_t prev_time;
    bool read = false;
    bool isFlipped = false;

private:
    static Encoder* instance1;
    static Encoder* instance2;
};

Encoder* Encoder::instance1 = nullptr;
Encoder* Encoder::instance2 = nullptr;

}  // namespace mtrn3100
