#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Wheel.hpp"

#include "IMU.hpp"

#include "Lidar.hpp"

bool screen_initialised = true;
bool imu_initialised = true;
int curTime = 0;

// Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // I2C adress
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Controllers
#define KP1 1.1
#define KI1 0
#define KD1 0.1
mtrn3100::PIDController left_controller(KP1, KI1, KD1);
#define KP2 1.1
#define KI2 0
#define KD2 0.1
mtrn3100::PIDController right_controller(KP2, KI2, KD2);

// Encoder direction controller
#define KP3 1.5
#define KI3 0
#define KD3 0.1
mtrn3100::PIDController diff_controller(KP3, KI3, KD3);

// True direction controller
#define KP4 1.5
#define KI4 0.3
#define KD4 0.1
mtrn3100::PIDController direction_controller(KP4, KI4, KD4);

// Motors
#define MOT2PWM 9
#define MOT2DIR 10
#define MOT1PWM 11
#define MOT1DIR 12
mtrn3100::Motor left_motor(MOT1PWM, MOT1DIR);
mtrn3100::Motor right_motor(MOT2PWM, MOT2DIR);

// Encoders
#define MOT1ENCA 2 // PIN 2 is an interupt
#define MOT2ENCA 3
#define MOT1ENCB 7 // PIN 7 is an interupt
#define MOT2ENCB 8
mtrn3100::Encoder left_encoder(MOT1ENCA, MOT1ENCB, 0);
mtrn3100::Encoder right_encoder(MOT2ENCA, MOT2ENCB, 1);

// Initialise each wheel
#define RIGHT_COEF 0.91
#define LEFT_COEF 0.95
mtrn3100::Wheel left_wheel(&left_motor, &left_encoder);
mtrn3100::Wheel right_wheel(&right_motor, &right_encoder);

// Initialise the IMU
mtrn3100::IMU imu(Wire);

// Tuning Prams
#define ANGLE_TOLERANCE 5
#define DIST_TOLERANCE 5
#define DIFF_TOLERANCE 3
#define DIRECTION_BIAS_STRENGTH 0.75
#define DIFF_BIAS_STRENGTH 0


void initScreen() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    screen_initialised = false;
  }
  // Clear the buffer
  display.clearDisplay();
}

void initWheels() {
  left_motor.flip();
  // right_motor.flip();
  // left_encoder.flip();
  right_encoder.flip();
}

void initIMU() {
  if(!imu.begin()) {
    imu_initialised = false;
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(1000);
  initScreen();
  initWheels();
  initIMU();
  delay(500);
  lidarSetup();
}

void drawString(String string) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(string);

  display.display();
}

void moveForwardDistance(uint16_t dist, float speed) {
  // Set to stay in the current direction
  float startDirection = imu.getDirection();
  direction_controller.zeroAndSetTarget(startDirection, startDirection);

  // Set the wheels to go forward dist
  left_controller.zeroAndSetTarget(left_wheel.getDistanceMoved(), dist);
  right_controller.zeroAndSetTarget(right_wheel.getDistanceMoved(), dist);

  while (!abs(left_controller.getError()) < DIST_TOLERANCE || !abs(right_controller.getError()) < DIST_TOLERANCE || !abs(direction_controller.getError()) < ANGLE_TOLERANCE) {
    
    float directionalAdjustment = direction_controller.computeDir(imu.getDirection());
    float leftSignal = left_controller.compute(left_wheel.getDistanceMoved());
    float rightSignal = right_controller.compute(right_wheel.getDistanceMoved());

    float leftMotorSignal = (constrain(leftSignal, -100, 100) + (directionalAdjustment * DIRECTION_BIAS_STRENGTH)) * speed;
    float rightMotorSignal = (constrain(rightSignal, -100, 100) - (directionalAdjustment * DIRECTION_BIAS_STRENGTH)) * speed;

    left_wheel.setSpeed(leftMotorSignal);
    right_wheel.setSpeed(rightMotorSignal);
  }

  left_wheel.setSpeed(0);
  right_wheel.setSpeed(0);
}

void turnToAngle(float angle, float speed) {
  // Set to move to the target angle
  float startDirection = imu.getDirection();
  direction_controller.zeroAndSetTarget(startDirection, angle);

  float leftZero = left_wheel.getDistanceMoved();
  float rightZero = right_wheel.getDistanceMoved(); 
  diff_controller.zeroAndSetTarget(0, 0);

  while (!abs(diff_controller.getError()) < DIFF_TOLERANCE || !abs(direction_controller.getError()) < ANGLE_TOLERANCE) {
    float directionalAdjustment = direction_controller.computeDir(imu.getDirection());

    float leftDiff = left_wheel.getDistanceMoved() - leftZero;
    float rightDiff = -(right_wheel.getDistanceMoved() - rightZero);
    // +ve = left forward /-ve means send left wheel back
    float diffAdjustment = diff_controller.compute(leftDiff - rightDiff);

    float leftMotorSignal = (constrain(directionalAdjustment, -100, 100) + (diffAdjustment * DIFF_BIAS_STRENGTH)) * speed;
    float rightMotorSignal = (constrain(-directionalAdjustment, -100, 100) - (diffAdjustment * DIFF_BIAS_STRENGTH)) * speed;

    left_wheel.setSpeed(leftMotorSignal);
    right_wheel.setSpeed(rightMotorSignal);
    delay(100);
  }

  left_wheel.setSpeed(0);
  right_wheel.setSpeed(0);
}

void loop() {
  
  turnToAngle(0,0.5);

  //delay(100);
  // getLeftDist();
  // getFrontDist();
  // getRightDist();
  // turnRight90();
  // turnLeft90();
 
  //executeMovementString("lfrfflfr");
  //delay(1000);
}

void moveForwardOneCell() {
  moveForwardDistance(180.0, 0.5);
}

void turnLeft90() {
  turnToAngle(imu.getDirection() - 90, 0.5);
}

void turnRight90() {
  turnToAngle(imu.getDirection() + 90, 0.5);
}

void executeMovementString(char* cmdString) {
  for (int i = 0; cmdString[i] != '\0'; ++i) {
    switch (cmdString[i]) {
      case 'f': moveForwardOneCell(); break;
      case 'l': turnLeft90(); break;
      case 'r': turnRight90(); break;
    }
    delay(100);
  }
}
