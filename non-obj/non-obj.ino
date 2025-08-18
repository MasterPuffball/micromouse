#include <SPI.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_GFX.h>

#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Wheel.hpp"
#include "IMU.hpp"
#include "Lidar.hpp"
#include "Map.hpp"
#include "EncoderOdometry.hpp"
#include "Constants.h"
#include "MapRenderer.hpp"

int curTime = 0;
int setCursorFirst = 10;
int setCursorSecond = 7;
float general_speed = 0.45;

struct Robot {
  U8G2_SSD1306_128X64_NONAME_1_HW_I2C display{U8G2_R0, U8X8_PIN_NONE};
  // mtrn3100::Map map{};
  // MapRenderer mapRenderer{display};
  // Controllers
  static constexpr float KP1 = 2.3;
  static constexpr float KI1 = 0.2;
  static constexpr float KD1 = 0.1;
  mtrn3100::PIDController left_controller{KP1, KI1, KD1};

  static constexpr float KP2 = 2.3;
  static constexpr float KI2 = 0.2;
  static constexpr float KD2 = 0.1;
  mtrn3100::PIDController right_controller{KP2, KI2, KD2};

  // Encoder direction controller
  static constexpr float KP3 = 1.5;
  static constexpr float KI3 = 0.1;
  static constexpr float KD3 = 0.1;
  mtrn3100::PIDController diff_controller{KP3, KI3, KD3};

  // True direction controller
  static constexpr float KP4 = 2.3;
  static constexpr float KI4 = 0.8;
  static constexpr float KD4 = 0.08;
  mtrn3100::PIDController direction_controller{KP4, KI4, KD4, true};

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
  mtrn3100::EncoderOdometry odom{&left_encoder, &right_encoder};

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

  int robotX = STARTING_X;
  int robotY = STARTING_Y;
  int robotOrientation = STARTING_ORIENTATION;

  Robot() {
  }

  void begin() {
    Serial.println("Beginning Robot");

    initScreen();
    Serial.println("Display Setup Complete");
    drawString("Display Setup Complete");

    delay(50);
  
    imu.begin();
    Serial.println("IMU Setup Complete");
    drawString("IMU Setup Complete");

    odom.begin();

    delay(50);

    initWheels();
    Serial.println("Wheel Setup Complete");
    drawString("Wheel Setup Complete");
        
    delay(50);
    Serial.println("Finished Setup");
    drawString("Finished Setup");
  }
  
  void loop() {
    // moveForwardOneCell();
    
    // display.firstPage();
    // do {
    //   mapRenderer.drawCompletion();
    // } while (display.nextPage());

    // exploreMap();
    // mapRenderer.drawMap();
	  // while (true) {}

    // turnToAngle(0,0.5);
    // maintainDistance(100, 0.5); 
    // turnLeft90(); 
    executeMovementString("frflflfrf");
    // executeMovement('l');
    // turnToAngle(90,0.4);
    // maintainDistance(100, 0.5);
    // direction_controller.tune(KP4, KI4, i);
    // Serial.println(imu.getDirection());
    delay(100);
    // getLeftDist();
    // getFrontDist();
    // getRightDist();
    // turnRight90();
    // testPrintLines();
    
    // drawTelemetry(direction_controller);
    // executeMovementString("lfrfflfr");
    // executeMovementString("ffllfrfr");
    // turnToAngle(-90, 0.3);
    // executeMovementString("r");

  }

  void initScreen() {
    display.begin();
    display.clearBuffer(); // Clear the internal memory
    display.setFont(u8g2_font_4x6_tr); // Choose a suitable font
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

  void drawTelemetry(mtrn3100::PIDController controller) {
    display.firstPage();
    do {
      display.setFont(u8g2_font_4x6_tr); // very small font, 4px tall

      int lineHeight = 6; // 4px font + ~2px spacing
      int startY = 6;     // top margin

      drawTextValueLine("IMU:",        imu.getDirection(),         0, startY + lineHeight * 0);
      drawTextValueLine("Error:",      controller.getError(),      0, startY + lineHeight * 1);
      drawTextValueLine("Derivative:", controller.getDerivative(), 0, startY + lineHeight * 2);
      drawTextValueLine("Integral:",   controller.getIntegral(),   0, startY + lineHeight * 3);
      drawTextValueLine("Target:",     controller.getTarget(),     0, startY + lineHeight * 4);
      drawTextValueLine("Zero:",       controller.getZero(),       0, startY + lineHeight * 5);
      drawTextValueLine("Odom θ:",     odom.getTheta(),            0, startY + lineHeight * 6);

    } while (display.nextPage());
  }

  void drawTextValueLine(const char* message, float num, int x, int y) {
    display.setCursor(x, y);
    display.print(message);
    drawFloatNoClear(num, x + 64, y);
  }

  void drawStringNoClear(const char* message, int x, int y) {
    display.setCursor(x, y);
    display.print(message);
  }

  void drawFloatNoClear(float num, int x, int y) {
    char buf[16];
    dtostrf(num, 0, 1, buf);
    drawStringNoClear(buf, x, y);
  }

  void drawFloat(float num, const int x = setCursorFirst, const int y = setCursorSecond) {
    char buf[16];
    dtostrf(num, 0, 1, buf);
    drawString(buf, x, y);
  }

  void drawString(const char* message, const int x = setCursorFirst, const int y = setCursorSecond) {
    display.clearBuffer(); // Clear the internal memory
    display.setCursor(x, y); // Set the cursor to the start position
    display.print(message); // Print the message
    display.sendBuffer(); // Transfer internal memory to the display
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
      drawTelemetry(left_controller);
      float directionalAdjustment = direction_controller.compute(imu.getDirection());
      float leftSignal = left_controller.compute(left_wheel.getDistanceMoved());
      float rightSignal = right_controller.compute(right_wheel.getDistanceMoved());

      float leftMotorSignal = (constrain(leftSignal, -100, 100) + (directionalAdjustment * DIRECTION_BIAS_STRENGTH)) * speed;
      float rightMotorSignal = (constrain(rightSignal, -100, 100) - (directionalAdjustment * DIRECTION_BIAS_STRENGTH)) * speed;

      left_wheel.setSpeed(leftMotorSignal);
      right_wheel.setSpeed(rightMotorSignal);

      if ((direction_controller.isWithin(10) && left_controller.isWithin(DIST_TOLERANCE) && right_controller.isWithin(DIST_TOLERANCE)) || millis() - startTime > MAX_DURATION) {
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
      drawTelemetry(direction_controller);
      
      float direction = imu.getDirection();
      float directionalAdjustment = direction_controller.compute(direction);
      // Serial.println(direction);
      // drawFloat(direction);

      float leftDiff = left_wheel.getDistanceMoved() - leftZero;
      float rightDiff = -(right_wheel.getDistanceMoved() - rightZero);
      // +ve = left forward /-ve means send left wheel back
      float diffAdjustment = diff_controller.compute(leftDiff - rightDiff);

      float leftMotorSignal = (constrain(directionalAdjustment, -100, 100) + (diffAdjustment * DIFF_BIAS_STRENGTH)) * speed;
      float rightMotorSignal = (constrain(-directionalAdjustment, -100, 100) - (diffAdjustment * DIFF_BIAS_STRENGTH)) * speed;

      left_wheel.setSpeed(leftMotorSignal);
      right_wheel.setSpeed(rightMotorSignal);
      
      if (direction_controller.isWithin(ANGLE_TOLERANCE) || millis() - startTime > MAX_DURATION) {
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
      float directionalAdjustment = direction_controller.compute(imu.getDirection());
      float distanceAdjustment = distance_controller.compute(front_lidar.get_dist());

      float leftMotorSignal = (constrain(-distanceAdjustment, -100, 100) + (directionalAdjustment * DIRECTION_BIAS_STRENGTH)) * speed;
      float rightMotorSignal = (constrain(-distanceAdjustment, -100, 100) - (directionalAdjustment * DIRECTION_BIAS_STRENGTH)) * speed;

      left_wheel.setSpeed(leftMotorSignal);
      right_wheel.setSpeed(rightMotorSignal);
    }
  }

  // // Basically just a DFS (note, must start with back against wall)
  // void exploreMap() {
  //   int cmdNumber = 0;
  //   char cmds[MAX_MOVEMENTS] = {};
  //   bool isBacktracking = false;
  //   bool adjToVisit = false;
    
  //   // Needs to start with butt against wall
  //   map.setWall(robotX,robotY,(robotOrientation + 2) % 4);
    
  //   while (!(cmdNumber == 0 && !adjToVisit)) {
  //     visitCurrentCell();
  //     // mapRenderer.drawMap();

  //     bool visitLeft = shouldTravelTo(-1);
  //     bool visitForward = shouldTravelTo(0);
  //     bool visitRight = shouldTravelTo(1);
  //     adjToVisit = visitLeft || visitForward || visitRight;

  //     // if not backtracking and shouldn't -> (prioritise left -> right-> forward) go there and save inverse instruction to cmds[cmdNumber] and cmdNumber++ 
  //     if (!isBacktracking) {
  //       if (adjToVisit) {
  //         if (visitLeft) {
  //           cmds[cmdNumber] = 'r';
  //           executeMovement('l');
  //         }
  //         else if (visitForward) {
  //           cmds[cmdNumber] = 'f';
  //           executeMovement('f');
  //         }
  //         else if (visitRight) {
  //           cmds[cmdNumber] = 'l';
  //           executeMovement('r');
  //         }
  //         cmdNumber++;
  //       }
  //       // If "not backtracking" and should (ie dead end) -> turn around, set to is backtracking
  //       else {
  //         executeMovement('u');
  //         isBacktracking = true;
  //       }
  //     }
  //     else {
  //       // if "backtracking" and shouldn't -> (do one more cmd (only if its a turn), turn around) = turn opposite way to cmd, now not backtracking 
  //       if (adjToVisit) {
  //         isBacktracking = false;
  //         cmdNumber--;
  //         if (cmds[cmdNumber] == 'l') {
  //           executeMovement('r');
  //         }
  //         else if (cmds[cmdNumber] == 'r') {
  //           executeMovement('l');
  //         }
  //         else {
  //           executeMovement('u');
  //         }
  //       }
  //       // if "backtracking" and should -> do latest command, subtract cmdNumber 
  //       else {
  //         cmdNumber--;
  //         executeMovement(cmds[cmdNumber]);
  //       }
  //     }
  //   }

  //   executeMovement('u');
  // }

  // void scanWalls() {
  //   if (front_lidar.get_dist() < IS_WALL_DIST) {
  //     map.setWall(robotX,robotY,robotOrientation);
  //   }
  //   if (left_lidar.get_dist() < IS_WALL_DIST) {
  //     map.setWall(robotX, robotY, (robotOrientation + 3) % 4);
  //   }
  //   if (right_lidar.get_dist() < IS_WALL_DIST) {
  //     map.setWall(robotX, robotY, (robotOrientation + 1) % 4); 
  //   }
  // }

  // void visitCurrentCell() {
  //   scanWalls();
  //   map.visitCell(robotX, robotY);
  // }

  // bool shouldTravelTo(int direction) {
  //   int absoluteDirection = (robotOrientation + (direction + 4)) % 4;
  //   return !map.cellVisited(robotX, robotY, absoluteDirection) && !map.wallExists(robotX, robotY, absoluteDirection);
  // }

  void moveForwardOneCell() {
    moveForwardDistance(180.0, general_speed);
  }

  // Right = positive angle here
  void turnToRelativeAngle(float angle) {
	  turnToAngle(imu.normalizeAngle(imu.getDirection() - angle), general_speed);
  }

  void executeMovementString(char* cmdString) {
    for (int i = 0; cmdString[i] != '\0'; ++i) {
      executeMovement(cmdString[i]);
    }
  }

  void executeMovement(char movement) {
    switch (movement) {
      case 'f': 
        moveForwardOneCell();
        switch (robotOrientation) {
          case UP: robotY -= 1; break;
          case RIGHT: robotX += 1; break;
          case DOWN: robotY += 1; break;
          case LEFT: robotX -= 1; break;
        }
        break;
      case 'l': 
        turnToRelativeAngle(-90); 
        robotOrientation = (robotOrientation + 3) % 4;
        break;
      case 'r': 
        turnToRelativeAngle(90); 
        robotOrientation = (robotOrientation + 1) % 4;
        break;
      case 'u': // U-turn
        turnToRelativeAngle(180);
        robotOrientation = (robotOrientation + 2) % 4;		
        break;
    }
    
    left_wheel.setSpeed(0);
    right_wheel.setSpeed(0);x
    delay(50);
  }
};

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Beginning Setup");
  delay(50);
  Robot robot{};

  constexpr auto a{sizeof(Robot)};
  constexpr auto b{sizeof(mtrn3100::Map)};
  constexpr auto c{sizeof(mtrn3100::PIDController)};

  robot.begin();
  
  delay(100);
  
  while (true) {
    robot.loop();
  }
}

void loop() {}