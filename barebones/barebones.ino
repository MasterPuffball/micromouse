#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DRV8835MotorShield.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // I2C adress

bool screen_initialised = true;
int curTime = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DRV8835MotorShield motors;

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

void initMotors() {
  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipM1(true);
  //motors.flipM2(true);
}

void setup() {
  Serial.begin(9600);

  initScreen();
  initMotors();
}

void loop() {
  if (millis() - curTime > 100){
    curTime = millis();
    drawString(String(curTime));
  }

  for (int speed = 0; speed <= 400; speed++)
  {
    motors.setM1Speed(speed);
    motors.setM2Speed(speed);
    delay(2);
  }

  for (int speed = 400; speed >= 0; speed--)
  {
    motors.setM1Speed(speed);
    motors.setM2Speed(speed);
    delay(2);
  }
}

