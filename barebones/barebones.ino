#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Wheel.hpp"
#include "Chassis.hpp"

bool screen_initialised = true;
int curTime = 0;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // I2C adress
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define KP1 3
#define KI1 0.2
#define KD1 0
#define KP2 3
#define KI2 0.2
#define KD2 0
mtrn3100::PIDController left_controller(KP1, KI1, KD1);
mtrn3100::PIDController right_controller(KP2, KI2, KD2);

#define MOT2PWM 9
#define MOT2DIR 10
#define MOT1PWM 11
#define MOT1DIR 12
mtrn3100::Motor left_motor(MOT1PWM, MOT1DIR);
mtrn3100::Motor right_motor(MOT2PWM, MOT2DIR);

#define MOT1ENCA 2 // PIN 2 is an interupt
#define MOT2ENCA 3
#define MOT1ENCB 7 // PIN 7 is an interupt
#define MOT2ENCB 8
mtrn3100::Encoder left_encoder(MOT1ENCA, MOT1ENCB, 0);
mtrn3100::Encoder right_encoder(MOT2ENCA, MOT2ENCB, 1);

// Initialise each wheel
mtrn3100::Wheel left_wheel(&left_controller, &left_motor, &left_encoder);
mtrn3100::Wheel right_wheel(&right_controller, &right_motor, &right_encoder);

mtrn3100::Chassis chassis(&left_wheel, &right_wheel);

void initScreen() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    screen_initialised = false;
  }

  // Clear the buffer
  display.clearDisplay();
}

void drawString(String string) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(string);

  display.display();
}

void initWheels() {
  left_motor.flip();
  // right_motor.flip();
  // left_encoder.flip();
  right_encoder.flip();
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  initScreen();
  initWheels();
}

void loop() {
  chassis.moveForwardDistance(200);

  while (true) {}
}

