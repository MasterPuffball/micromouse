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
#define KP1 1.5
#define KI1 1.2
#define KD1 0
mtrn3100::PIDController left_controller(KP1, KI1, KD1);
#define KP2 1.5
#define KI2 1.2
#define KD2 0
mtrn3100::PIDController right_controller(KP2, KI2, KD2);
#define KP3 1.5
#define KI3 1.2
#define KD3 0
mtrn3100::PIDController IMU_controller(KP3, KI3, KD3)

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
mtrn3100::Wheel left_wheel(&left_controller, &left_motor, &left_encoder, LEFT_COEF);
mtrn3100::Wheel right_wheel(&right_controller, &right_motor, &right_encoder, RIGHT_COEF);

// Initialise the IMU
mtrn3100::IMU imu(Wire);

#define AXLE_LENGTH 40.0; //in Millis
#define ANGLE_TOLERANCE 1;

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

void drawString(String string) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(string);

  display.display();
}

void moveWheelToTarget(mtrn3100::Wheel wheel, float speed) {
  if (!wheel.isFinishedMove()) {
    float pos = wheel.getDistanceMoved();
    float intendedSignal = wheel.compute(pos);
    float motorSignal = wheel.getMotorSignal(intendedSignal, speed);

    Serial.println(String("Intended: ") + intendedSignal);
    Serial.println(String("Actual: ") + motorSignal);
    Serial.println(String("Pos: ") + wheel.getError());

    wheel.setSpeed(motorSignal);
    wheel.updateTolerance();
  }
}

void moveForwardDistance(uint16_t dist) {
  left_wheel.setTarget(dist);
  right_wheel.setTarget(dist);

  while (!left_wheel.isFinishedMove() && !right_wheel.isFinishedMove()) {
    moveWheelToTarget(left_wheel, 0.5);
    moveWheelToTarget(right_wheel, 0.5);
  }

  left_wheel.setSpeed(0);
  right_wheel.setSpeed(0);
}

void computeTurnDistTo(float angle) {
  float normalized = fmod(angle - imu.read().z, 360.0f);

  if (normalized > 180) {
    normalized -= 360.0;
  }

  return AXLE_LENGTH * normalized * (PI/180);
}

void turnToAngle(float angle, float speed) {
  float dir = imu.normalizeAngle(angle);
  float dist = computeTurnDistTo(dir);

  left_wheel.setTarget(dist);
  right_wheel.setTarget(-dist);

  while (!left_wheel.isFinishedMove() && !right_wheel.isFinishedMove()) {
    moveWheelToTarget(left_wheel, speed);
    moveWheelToTarget(right_wheel, speed);
  }

  left_wheel.setSpeed(0);
  right_wheel.setSpeed(0);
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

void loop() {
  snapToAngle(90, 0.5);
  // moveForwardDistance(220);
  //imu.printCurrentData();

  //delay(100);
  //moveForwardDistance(220);
  // getLeftDist();
  // getFrontDist();
  // getRightDist();

  //turnLeft90();
  //turnRight90();
  //executeMovementString("lfrfflfr");
  //delay(1000);
}

bool withinAngleTolerance(float target) {
  float angle = imu.updateRead().z;
  return (angle <= target + ANGLE_TOLERANCE) && (angle >= target - ANGLE_TOLERANCE);
}

void snapToAngle(float angle, float speed) {
  if (!withinAngleTolerance(angle)) {
    turnToAngle(angle, speed);
  }
}

void moveForwardOneCell() {
  moveForwardDistance(180.0);
}

void turnLeft90() {
  turnToAngle(imu.read().z - 90, 0.5);
}

void turnRight90() {
  turnToAngle(imu.read().z + 90, 0.5);
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
