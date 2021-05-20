#include "Arduino.h"
#include "common_utils.h"

void setup_indicator() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void indicate_glow() {
  digitalWrite(LED_BUILTIN, HIGH);
}

void indicate_off() {
  digitalWrite(LED_BUILTIN, LOW);
}

void indicate_blink (int count, int on_time, int off_time) {
  indicate_off();
  delay(250);
  for (int i=0; i < count; i++) {
    indicate_glow();
    delay(on_time);
    indicate_off();
    delay(off_time);
  }
}
