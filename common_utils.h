#ifndef _COMMON_UTILS_H
#define _COMMON_UTILS_H

#define DEBUG 1

extern void setup_indicator();
extern void indicate_glow();
extern void indicate_off();
extern void indicate_blink (int count, int on_time, int off_time);

#endif
