#ifndef _COMMON_UTILS_H
#define _COMMON_UTILS_H

#define DEBUG 1

// Enables adjusting from MPU
#define ENABLE_MPU  1      // Enables reading
#define ADJUST_FROM_MPU 1  // Enables adjustment

// Get the RC commands to determine a target
#define ENABLE_RC   1

// Actually drive the motors
#define ENABLE_ESC 0

extern void setup_indicator();
extern void indicate_glow();
extern void indicate_off();
extern void indicate_blink (int count, int on_time, int off_time);

#endif
