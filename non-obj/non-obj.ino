#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <U8g2lib.h>

#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Wheel.hpp"
#include "IMU.hpp"
#include "Lidar.hpp"

int curTime = 0;
int setCursorFirst = 10;
int setCursorSecond = 7;

struct Robot {
  U8G2_SSD1306_128X64_NONAME_1_HW_I2C display{U8G2_R0, U8X8_PIN_NONE};

  // Controllers
  static constexpr float KP1 = 1.1;
  static constexpr float KI1 = 0.2;
  static constexpr float KD1 = 0.1;
  mtrn3100::PIDController left_controller{KP1, KI1, KD1};

  static constexpr float KP2 = 1.1;
  static constexpr float KI2 = 0.2;
  static constexpr float KD2 = 0.1;
  mtrn3100::PIDController right_controller{KP2, KI2, KD2};

  // Encoder direction controller
  static constexpr float KP3 = 1.5;
  static constexpr float KI3 = 0.1;
  static constexpr float KD3 = 0.1;
  mtrn3100::PIDController diff_controller{KP3, KI3, KD3};

  // True direction controller
  static constexpr float KP4 = 1.5;
  static constexpr float KI4 = 0.05;
  static constexpr float KD4 = 0.1;
  mtrn3100::PIDController direction_controller{KP4, KI4, KD4};

  // Distance Controller
  static constexpr float KP5 = 1.5;
  static constexpr float KI5 = 0.3;
  static constexpr float KD5 = 0.1;
  mtrn3100::PIDController distance_controller{KP5, KI5, KD5};

  // Motors
  static constexpr int MOT2PWM = 9;
  static constexpr int MOT2DIR = 10;
  static constexpr int MOT1PWM = 11;
  static constexpr int MOT1DIR = 12;
  mtrn3100::Motor left_motor{MOT1PWM, MOT1DIR};
  mtrn3100::Motor right_motor{MOT2PWM, MOT2DIR};

  // Encoders
  static constexpr int MOT1ENCA = 2; // PIN 2 is an interupt
  static constexpr int MOT2ENCA = 3;
  static constexpr int MOT1ENCB = 7; // PIN 7 is an interupt
  static constexpr int MOT2ENCB = 8;
  mtrn3100::Encoder left_encoder{MOT1ENCA, MOT1ENCB, 0};
  mtrn3100::Encoder right_encoder{MOT2ENCA, MOT2ENCB, 1};

  // Wheels
  static constexpr float RIGHT_COEF = 0.91;
  static constexpr float LEFT_COEF = 0.95;
  mtrn3100::Wheel left_wheel{&left_motor, &left_encoder};
  mtrn3100::Wheel right_wheel{&right_motor, &right_encoder};

  // IMU
  mtrn3100::IMU imu{};

  // Lidars
  static constexpr int leftSensorPin = A0;
  static constexpr int frontSensorPin = A1;
  static constexpr int rightSensorPin = A2;
  static constexpr int leftSensorAddress = 0x54;
  static constexpr int frontSensorAddress = 0x56;
  static constexpr int rightSensorAddress = 0x58;
  mtrn3100::Lidar left_lidar{leftSensorPin, leftSensorAddress};
  mtrn3100::Lidar front_lidar{frontSensorPin, frontSensorAddress};
  mtrn3100::Lidar right_lidar{rightSensorPin, rightSensorAddress};

  // Tuning Prams
  static constexpr float ANGLE_TOLERANCE = 5;
  static constexpr float DIST_TOLERANCE = 5;
  static constexpr float DIFF_TOLERANCE = 3;
  static constexpr float SLOPE_TOLERANCE = 0.02;
  static constexpr float DIRECTION_BIAS_STRENGTH = 0.75;
  static constexpr float DIFF_BIAS_STRENGTH = 0;
  static constexpr int MAX_DURATION = 10000; // in millis

  // Wall following constants
  static constexpr float WALL_DIST = 100;

  Robot() {
  }

  void begin() {
    Serial.println("Beginning Robot");
    imu.begin();
    Serial.println("IMU Setup Complete");
    initWheels();
    Serial.println("Wheel Setup Complete");
    delay(100);
    initScreen();
    Serial.println("Display Setup Complete");
    
    delay(500);
    Serial.println("Finished Setup");
    drawString("Finished Setup");
   
    delay(250);
  }
  
  void loop() {
    // moveForwardOneCell();
    
    // turnToAngle(0,0.5);
    // maintainDistance(100, 0.5); 
    turnLeft90();
    // turnToAngle(90,0.4);
    // maintainDistance(100, 0.5);

    //delay(100);
    // getLeftDist();
    // getFrontDist();
    // getRightDist();
    // turnRight90();

    Serial.println(imu.getDirection());
    drawFloat(imu.getDirection());

    // executeMovementString("lfrfflfr");
    // executeMovementString("ffllfrfr");
    // turnToAngle(-90, 0.3);
    // executeMovementString("r");

  }

  void initScreen() {
    display.begin();
    display.clearBuffer(); // Clear the internal memory
    display.setFont(u8g2_font_6x12_mf); // Choose a suitable font
    display.setCursor(setCursorFirst, setCursorSecond);
    display.print("Staring!!!"); // Write a string to the display
    display.sendBuffer(); // Transfer internal memory to the display
  }

  void initWheels() {
    left_motor.flip();
    // right_motor.flip();
    // left_encoder.flip();
    right_encoder.flip();
  }

  void drawFloat(float num) {
    String message = String(num, 1);
    drawString(message.c_str());
  }

  void drawString(const char* message) {
    display.clearBuffer(); // Clear the internal memory
    display.setCursor(setCursorFirst, setCursorSecond); // Set the cursor to the start position
    display.print(message); // Print the message
    display.sendBuffer(); // Transfer internal memory to the display
  }

  bool directionSteady() {
    if (abs(direction_controller.getError()) < ANGLE_TOLERANCE && abs(direction_controller.getDerivative() < SLOPE_TOLERANCE)) {
      return true;
    }
    Serial.println("DIRECTION NOT STEADY");
    return false;
  }

  bool leftPosSteady() {
    if (abs(left_controller.getError()) < DIST_TOLERANCE && abs(left_controller.getDerivative()) < SLOPE_TOLERANCE) {
      return true;
    }
    Serial.println("LEFT NOT STEADY");
    return false;
  }

  bool rightPosSteady() {
    if (abs(right_controller.getError()) < DIST_TOLERANCE && abs(right_controller.getDerivative()) < SLOPE_TOLERANCE) {
      return true;
    }
    Serial.println("RIGHT NOT STEADY");
    return false;
  }

  void moveForwardDistance(uint16_t dist, float speed) {
    // Set to stay in the current direction
    float startDirection = imu.getDirection();
    direction_controller.zeroAndSetTarget(startDirection, startDirection);

    // Set the wheels to go forward dist
    left_controller.zeroAndSetTarget(left_wheel.getDistanceMoved(), dist);
    right_controller.zeroAndSetTarget(right_wheel.getDistanceMoved(), dist);

    long startTime = millis();

    while (true) {
      float directionalAdjustment = direction_controller.computeDir(imu.getDirection());
      float leftSignal = left_controller.compute(left_wheel.getDistanceMoved());
      float rightSignal = right_controller.compute(right_wheel.getDistanceMoved());

      float leftMotorSignal = (constrain(leftSignal, -100, 100) + (directionalAdjustment * DIRECTION_BIAS_STRENGTH)) * speed;
      float rightMotorSignal = (constrain(rightSignal, -100, 100) - (directionalAdjustment * DIRECTION_BIAS_STRENGTH)) * speed;

      left_wheel.setSpeed(leftMotorSignal);
      right_wheel.setSpeed(rightMotorSignal);

      if ((directionSteady() && leftPosSteady() && rightPosSteady()) || millis() - startTime > MAX_DURATION) {
        break;
      }
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

    long startTime = millis();

    while (true) {
      float direction = imu.getDirection();
      float directionalAdjustment = direction_controller.computeDir(direction);
      Serial.println(direction);
      drawFloat(direction);

      float leftDiff = left_wheel.getDistanceMoved() - leftZero;
      float rightDiff = -(right_wheel.getDistanceMoved() - rightZero);
      // +ve = left forward /-ve means send left wheel back
      float diffAdjustment = diff_controller.compute(leftDiff - rightDiff);

      float leftMotorSignal = (constrain(directionalAdjustment, -100, 100) + (diffAdjustment * DIFF_BIAS_STRENGTH)) * speed;
      float rightMotorSignal = (constrain(-directionalAdjustment, -100, 100) - (diffAdjustment * DIFF_BIAS_STRENGTH)) * speed;

      left_wheel.setSpeed(leftMotorSignal);
      right_wheel.setSpeed(rightMotorSignal);
      
      if (directionSteady() || millis() - startTime > MAX_DURATION) {
        break;
      }
    }

    left_wheel.setSpeed(0);
    right_wheel.setSpeed(0);
  }

  void maintainDistance(float distance, float speed) {
    // Set to stay in the current direction
    float startDirection = imu.getDirection();
    direction_controller.zeroAndSetTarget(startDirection, startDirection);

    // Set the wheels to go forward dist
    distance_controller.zeroAndSetTarget(0, distance);

    while (true) {
      float directionalAdjustment = direction_controller.computeDir(imu.getDirection());
      float distanceAdjustment = distance_controller.compute(front_lidar.get_dist());

      float leftMotorSignal = (constrain(-distanceAdjustment, -100, 100) + (directionalAdjustment * DIRECTION_BIAS_STRENGTH)) * speed;
      float rightMotorSignal = (constrain(-distanceAdjustment, -100, 100) - (directionalAdjustment * DIRECTION_BIAS_STRENGTH)) * speed;

      left_wheel.setSpeed(leftMotorSignal);
      right_wheel.setSpeed(rightMotorSignal);
    }
  }

  void moveForwardOneCell() {
    moveForwardDistance(180.0, 0.3);
  }

  void turnLeft90() {
    turnToAngle(imu.normalizeAngle(imu.getDirection() + 90), 0.3);
    delay(200);
  }

  void turnRight90() {
    turnToAngle(imu.normalizeAngle(imu.getDirection() - 90), 0.3);
    delay(200);
  }

  void executeMovementString(char* cmdString) {
    for (int i = 0; cmdString[i] != '\0'; ++i) {
      switch (cmdString[i]) {
        case 'f': moveForwardOneCell(); break;
        case 'l': turnLeft90(); break;
        case 'r': turnRight90(); break;
      }
      delay(500);
    }
  }
};

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Beginning Setup");
  delay(50);
  Robot robot{};

  robot.begin();
  
  delay(100);

  while (true) {
    robot.loop();
  }
}

void loop() {}
