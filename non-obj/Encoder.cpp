#include "Encoder.hpp"

namespace mtrn3100 {

Encoder* Encoder::instance1 = nullptr;
Encoder* Encoder::instance2 = nullptr;

Encoder::Encoder(uint8_t enc1, uint8_t enc2, int num) : encoder1_pin(enc1), encoder2_pin(enc2) {
    pinMode(encoder1_pin, INPUT_PULLUP);
    pinMode(encoder2_pin, INPUT_PULLUP);
    
    if (num == 0) {
      instance1 = this;
      attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR1, RISING);
    }
    if (num == 1) {
      instance2 = this;
      attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR2, RISING);
    }
}

void Encoder::readEncoder() {
    noInterrupts();
    int read = digitalRead(encoder2_pin);
    
    if (read == LOW) {
      count += (isFlipped == true ? -1 : 1);
    }
    else if (read == HIGH) {
      count -= (isFlipped == true ? -1 : 1);
    }
  
    interrupts();
}

void Encoder::readEncoderISR1() {
    if (instance1 != nullptr) {
        instance1->readEncoder();
    }
}

void Encoder::readEncoderISR2() {
    if (instance2 != nullptr) {
        instance2->readEncoder();
    }
}

float Encoder::getRotation() {
    return count*((2*PI)/TICKS_PER_REV);
}

float Encoder::getRotationDegrees() {
    return ((count*360)/TICKS_PER_REV);
}

long Encoder::getCount() {
    long res;
    noInterrupts();
    res = count;
    interrupts();
    return res;
}

void Encoder::flip() {
    isFlipped = !isFlipped;
}

} // namespace mtrn3100
