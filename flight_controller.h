#ifndef _MOTOR_DRIVER_H
#define _MOTOR_DRIVER_H



// ***************** Define the pin connection *******

#define LF_ESC_PIN 3    // Left Front
#define LR_ESC_PIN 11   // Left Rear
#define RF_ESC_PIN 10   // Right Front
#define RR_ESC_PIN 9    // Right Rear

#define MIN_ESC_DRIVE  0
#define MAX_ESC_DRIVE  180

//short for minimum/maximum ESC PWM in milli seconds
#define MIN_ESC_PWM_IN_MS    900
#define MAX_ESC_PWM_IN_MS    2000
 
#define FLOAT_ESC_PWM_IN_MS  1400   
//the value for which the drone is assumed to stay flying in one place


// *** We need to adjust each of the motors, so that it gives more or less equal thrust
#define LF_ESC_INDEX 0
#define LR_ESC_INDEX 1
#define RF_ESC_INDEX 2
#define RR_ESC_INDEX 3

#define MAX_TILT_ANGLE 
// Current value of esc drive
/*extern unsigned int esc_drive_float[4];
extern unsigned int esc_drive_reg[4]; // Current value of esc drive 

// Funtions 
extern void setup_ESCs ();
extern void esc_set_drive_to_float(int motor);
extern void esc_set_drive(int motor, int drive);
extern void map_to_esc_drive(int motor, int drive, int min_drive, int max_drive);
inline int  esc_get_drive(int motor) { return esc_drive_reg[motor]; }
*/

#endif


#ifndef _FLIGHT_CONTROLLER_H
#define _FLIGHT_CONTROLLER_H

#include "common_utils.h"
//#include "gyro_data.h"
#include "rc_comm.h"
#include "motor_driver.h"

// This is X formation frame
#define LF_MOTOR 0    // Left front
#define LR_MOTOR 1    // Left rear
#define RF_MOTOR 2    // Right front
#define RR_MOTOR 3    // Right rear

// The MPU is assumed to be placed in XY plane, which is horizontal, and Y axis is front, X is towards 

// **************** PID controller parameters
// Following function will provide the calculated drive model
extern flight_controller fc;

#endif


/*
(0) (2)     x
  \ /     z ↑
   X       \|
  / \       +----→ y
(1) (3)

      Z(into the plane)
      |
  LF  |   RF
______X_______Y
      |
  LR  |   RR
      |
The MPU6050 must be oriented as following:

X axis : roll which mean bend left or bend right
       : bend left: speed up RF and RR
       : bend right: speed up LF and LL 
Y axis : pitch which means move forward or backward
       : forward: LF and RF < LR and RR
       : backward: LF and RF > LR and RR 
Z axis : yaw: rotate left or right
       : rotate left: RF and LR > RR and LF
       : rotate right: RF and LR < RR and LF
Up-Down: increase or decrese speed of all motors by same amount      

Angles vs Tilt correlation
      Z(into the plane)
      |
  LF  |   RF
______X_______Y
      |
  LR  |   RR
      |

forward tilt : -X
backward tilt: +X
left tilt: -Y
right tilt: +y
clockwise rotate: -Z
*/

// The model will be set to as follows
// 0 - not at all rotating
// 25 - Just enough to float
// 100 - to the maximum rotation
// This will allow us to have some headroom to slow down in one side to fall, while good enough 
// Resolution for flying.