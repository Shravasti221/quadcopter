#ifndef _RC_COMM_H

#define _RC_COMM_H


// Define the RC channel connections MODE 2
#define RC_CH1   2    // Ailron - Right Joystick, sidewise
#define RC_CH2   4    // Elevator - Right Joystick, front back
#define RC_CH3   5    // Throttle - Left Joystick, front back, can hold 
#define RC_CH4   6    // Rudder - Left Joystick, sidewise
#define RC_CH5   7    // Aux - CH5 - VRA - Left knob at front
#define RC_CH6  12    // Aux - CH6 - VRB - Right knob at front

/* expected behavior from the RC control: (Mode 2)
 *  Left joy stick: Up down movement - can hold - should determine the hight of the drone (Channel 4)
 *                  left right movement - Roll in current position  (Channel 7)
 *  Right Joystick: Up down movement - move forward or backward     (Channel 3)
 *                  left right movement - Move left or right in current plane (Channel 2)
 *  Top two knob - Left is channel 8, right is channel 12
 */
// Now channel assignments to actual value
#define RC_THROTTLE  RC_CH3
#define RC_RUDDER    RC_CH4
#define RC_ELEVATOR  RC_CH2
#define RC_AILERON   RC_CH1

// This two is not very convenient
#define RC_THRTL_Y RC_CH5
#define RC_PITCH   RC_CH6

// Some hashdefs to make the code more managable
#define MIN_RC_ELEVATOR  950
#define MAX_RC_ELEVATOR  1950

#define MIN_RC_THROTTLE  950
#define MAX_RC_THROTTLE  1950

#define MIN_RC_ROLL  950
#define MAX_RC_ROLL  1950

#define MIN_RC_PITCH  950
#define MAX_RC_PITCH  1950

// *********************  Get RC data  ***************
#define RC_MIN_THROTTLE     900
#define RC_NEUTRAL_THROTTLE 1425    // this is the duration in microseconds of neutral throttle on an electric RC Car
#define RC_MAX_THROTTLE     1950

// Define the delay between two RC reads
#define RC_READ_INTERVAL  125000    // 8 HZ

// Take 5 second before declare the communication lost, currently we are 
#define INIT_RC_HEARTBIT_LEFT  (5 * 1000000 / RC_READ_INTERVAL)

#define numMaxPinNum 13

// The setup_rc should be in blocking mode unless the RC is active
extern void setup_rc();

extern int update_rc_data();  // returns 1 if there was any update, else return 0
extern int is_rc_alive();     // Returns true if we have any update in any of the RC value in recent past

// Some utility functions 
int rc_aileron();
int rc_elevator();
int rc_rudder();
int rc_throttle();
void calibrate_rc();
void RC_print_vals(); 

#endif
