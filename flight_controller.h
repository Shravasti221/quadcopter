#ifndef _FLIGHT_CONTROLLER_H
#define _FLIGHT_CONTROLLER_H
#include "Arduino.h"
#include "common_utils.h"
#include "rc_comm.h"
#include "gyro.h"
#include "motor.h"

#define _CONSTRAINED  0
// ***************** Define the pin connection *******

#define LF_ESC_PIN 3    // Left Front
#define LR_ESC_PIN 11   // Left Rear
#define RF_ESC_PIN 10   // Right Front
#define RR_ESC_PIN 9    // Right Rear


#define MAX_TILT_ANGLE 5

// This is X formation frame
#define LF_MOTOR 0    // Left front
#define LR_MOTOR 1    // Left rear
#define RF_MOTOR 2    // Right front
#define RR_MOTOR 3    // Right rear

class flight_controller{
  // The model will map the value from 0 to 100 (kind of %)
  float lastCtlLoopTime;
  gyroscope gyro;
  int lb, ub; //for speed of motors
  float motor_throttle; // current speed of all the motors
  float x_tilt, y_tilt, z_tilt;//target_tilt from RC
  float Xangle, Yangle, Zangle;//updated values from gyro;
  static int mod2;
  public:
  motor motors[4]; //the individual motors;
  /*
    motors[0] = &LF_ESC;
    motors[1] = &LR_ESC;
    motors[2] = &RF_ESC;
    motors[3] = &RR_ESC;
  */
 public:
  flight_controller();
  void calculate_flight_targets();
  void reset2float();
  void reset2throttle();
  void lost_control();
  void get_gyro_values();
  void set_model_drives();
  void drive();
  void print_vals();
  void run_flight_controller();

};

#endif

// The MPU is assumed to be placed in XY plane, which is horizontal, and Y axis is front, X is towards 

// **************** PID controller parameters
// Following function will provide the calculated drive model
/*
flight controller class:
motor motors[4] : an array for holding the 4 motor objects
contructor: 
  1.sets esc pin
  #if DDEBUG mode switched on
  2.Print and blink
  3.Tests every motor : LF -> LR -> RF -> RR
calculate flight targets() has to update the target values based on the data in vars in rc.cpp
reset_angles(): set all motors to float
heartbeat(): Idt I have used yet

gyro.h
print_vals;
check the updated MPU 6050 values

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
// 40 - Just enough to float
// 100 - to the maximum rotation
// This will allow us to have some headroom to slow down in one side to fall, while good enough 
// Resolution for flying.
