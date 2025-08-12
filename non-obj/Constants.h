// Constants.h
#ifndef CONSTANTS_H
#define CONSTANTS_H

// Controls Params
#define ANGLE_TOLERANCE 1
#define DIST_TOLERANCE 5
#define DIFF_TOLERANCE 3
#define SLOPE_TOLERANCE 0.02
#define DIRECTION_BIAS_STRENGTH 0.75
#define DIFF_BIAS_STRENGTH 0
#define MAX_DURATION 100000 // in millis

// Wall following constants
#define WALL_DIST 100

// PID 
#define DERIVATIVE_DEADZONE 5
#define INCREMENT_CLAMP 10
#define INTEGRAL_CLAMP 20
#define NUM_STEADY 5
#define STEADY_TOLERANCE 2

// IMU
#define IMU_AVERAGE_AMOUNT 5

// Encoder Odometry
#define WHEEL_DIAMETER 32.4f //In millimeters
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)
#define AXLE_LENGTH 84 //In millimeters
#define TICKS_PER_REV 700
#define WHEEL_CIRCUM (WHEEL_DIAMETER * 3.14159)
#define MM_PER_TICK (WHEEL_CIRCUM / TICKS_PER_REV)
#define ODOM_POLL_TIME 1 //In Milliseconds

#endif  // CONSTANTS_H
