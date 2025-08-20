// Constants.h
#ifndef CONSTANTS_H
#define CONSTANTS_H

// Controls Params
#define ANGLE_TOLERANCE 1
#define DIST_TOLERANCE 5
#define DIFF_TOLERANCE 3
#define SLOPE_TOLERANCE 0.02
#define DIRECTION_BIAS_STRENGTH 0.75
#define WALL_BIAS_STRENGTH 0.3
#define DIFF_BIAS_STRENGTH 0
#define MAX_DURATION 5000 // in millis
#define NUM_TIMEOUT 5 
#define NUM_DO 3

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

// Map Constants
#define MAX_MOVEMENTS 100
#define MAP_HEIGHT 9
#define MAP_LENGTH 9
#define MAX_NODES (MAP_HEIGHT*MAP_LENGTH)
#define UP 0
#define RIGHT 1
#define DOWN 2
#define LEFT 3
#define IS_WALL_DIST 150

// Map Draw Constants
#define CURSOR_START_X 10
#define CURSOR_START_Y 5
#define CELL_SIZE (SCREEN_HEIGHT - (CURSOR_START_Y*2))/MAP_HEIGHT
#define SCREEN_HEIGHT 64
#define SCREEN_LENGTH 128
#define NON_EXISTENT_CELLS 12

// Compact Map Storage Constants
#define NUM_MAP_BITS (MAP_HEIGHT*MAP_LENGTH)
#define NUM_MAP_BYTES ((NUM_MAP_BITS + 7) / 8)

// Robot starting orientation
#define STARTING_ORIENTATION (DOWN)
#define STARTING_X 2
#define STARTING_Y 0




#endif  // CONSTANTS_H
