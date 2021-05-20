#include "Arduino.h"
// Enabled all interrupt
#define EI_ARDUINO_INTERRUPTED_PIN
#include "EnableInterrupt.h"

#include "common_utils.h"
#include "rc_comm.h"

/* RC receiver has a fail safe mode where the receiver will hosld th elast received value when 
 *  the connection to the transmitter is lost. We need to detect this by observing no change 
 *  of value for consecutive X number of times, and then go into failsafe mode. 
 */
 
volatile uint8_t externalInterruptFlag=0;
volatile uint8_t pinChangeInterruptFlag=0;

// volatile, we set this in the Interrupt and read it in loop so it must be declared volatile
volatile int     nRcChannelData[numMaxPinNum] = {0,0,0,0,0,0,0,0,0,0,0,0,0} ;

// set in the interrupt and read in the loop
volatile boolean bNewThrottleSignal[numMaxPinNum] = {false, false, false, false, false, false, false, false, false, false,false, false, false}; 
volatile unsigned long ulStartPeriod[numMaxPinNum] = {0,0,0,0,0,0,0,0,0,0,0,0,0} ; // set in the interrupt

volatile uint8_t lastPinChanged = 0;
volatile uint8_t rcHeartbitLeft = INIT_RC_HEARTBIT_LEFT ;

volatile unsigned long int lastRCInterruptTime = 0;
unsigned long int lastRCReadTime = 0;

int  nlastRcChannelData[numMaxPinNum] = {0,0,0,0,0,0,0,0,0,0,0,0,0} ;
int  min_rc_value[numMaxPinNum];
int  max_rc_value[numMaxPinNum];
int  center_rc_value[numMaxPinNum];



void rx_calcInput();
void loop_printRCData();

int wait_for_receiver();
void wait_sticks_zero();
void register_min_max();
void  check_to_continue(int channel, int move=150);
void calibrate_rc();

void setup_rc () {
  //// Serial.println("Setting up Remote contoller \n");
  // tell the Arduino we want the function calcInput to be called whenever INT0 (digital pin 2) changes from HIGH to LOW or LOW to HIGH
  // catching these changes will allow us to calculate how long the input pulse is
  enableInterrupt (RC_CH1, rx_calcInput, CHANGE);
  enableInterrupt (RC_CH2, rx_calcInput, CHANGE);
  enableInterrupt (RC_CH3, rx_calcInput, CHANGE);
  enableInterrupt (RC_CH4, rx_calcInput, CHANGE);
  enableInterrupt (RC_CH5, rx_calcInput, CHANGE);
  enableInterrupt (RC_CH6, rx_calcInput, CHANGE);

  calibrate_rc();
}

// Returns the value of data
int rc_aileron() {
  return nRcChannelData[RC_AILERON];
}

int rc_elevator()
{
  return nRcChannelData[RC_ELEVATOR];
}
int rc_rudder() {
  return nRcChannelData[RC_RUDDER];
}
int rc_throttle() {
  return nRcChannelData[RC_THROTTLE];
}

// This is the interrupt handler, so keep it as much short as possible
void rx_calcInput() {
  unsigned long ulLocalTime = micros();
    
  lastPinChanged = arduinoInterruptedPin;
  lastRCInterruptTime = ulLocalTime;
  
  // if the pin is high, its the start of an interrupt
  if (arduinoPinState > 0) { 
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its 
    // easy to understand and works very well
    ulStartPeriod[lastPinChanged] = ulLocalTime;
  } else {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the 
    // start time ulStartPeriod from the current time returned by micros()
    if (ulStartPeriod[lastPinChanged] > 0) {
      nRcChannelData[lastPinChanged] = (int)(ulLocalTime - ulStartPeriod[lastPinChanged]);
      bNewThrottleSignal[lastPinChanged] = true;
      ulStartPeriod[lastPinChanged] = 0;
    }
  }
}

// ****************************  Other functions *****************************

int update_rc_data () {
  int rcChanged = 0;
  for (int i = 0; i < numMaxPinNum; i++) {
    // It can be withing some tolerance too, as the determnation is based on pulse width
    if (nlastRcChannelData[i] != nRcChannelData[i]) {
       rcChanged = 1;
       nlastRcChannelData[i] = nRcChannelData[i];
    }
  }
  if (rcChanged) { 
     rcHeartbitLeft = INIT_RC_HEARTBIT_LEFT;
  } else {
    if (rcHeartbitLeft != 0) {
      rcHeartbitLeft--;
    }
  }

#if 0
  Serial.print("A = "); Serial.print(rc_aileron()); Serial.print(", E = "); Serial.print(rc_elevator()); 
  Serial.print(", R = "); Serial.print(rc_rudder()); Serial.print(", T = "); Serial.print(rc_throttle()); 
#endif

  return rcChanged;
}

// This function simply returns if we are getting change in the values of control - expecting always some change
int is_rc_alive() {

#if DEBUG
  static int alert = 1;
#endif
  
#if DEBUG
  if (rcHeartbitLeft > 0) {
    if (alert) {
      Serial.println(" ******* RC just became active **************");
      alert = 0;
    }
  } else {
    if (!alert) {
      Serial.println(" ******** NO RC - going into failsafe mode *******");
      alert = 1;
    }
  }
#endif

  return (rcHeartbitLeft > 0);
}

void loop_printRCData() {

#if DEBUG  
  // if a new throttle signal has been measured, lets print the value to serial, if not our code could carry on with some other processing 
  Serial.print("Ailron: "); Serial.print(rc_aileron()); 
  Serial.print(", Elevator: "); Serial.print(rc_elevator()); 
  Serial.print(", Rudder: "); Serial.print(rc_rudder());
  Serial.print(", Throttle: "); Serial.print(rc_throttle()); 
#endif

}


void calibrate_rc() {

  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("Transmitter setup"));
  Serial.println(F("==================================================="));
  delay(1000);
  Serial.print(F("Checking for valid receiver signals."));
  //Wait 10 seconds until all receiver inputs are valid
  if (wait_for_receiver()) {
    Serial.println("Coild not detect the receiver - exiting the program");
    exit(0);
  }
  
  Serial.println(F(""));
  
  delay(2000);
  Serial.println(F("Place all sticks and subtrims in the center position within 10 seconds."));
  for(int i = 9; i > 0; i--) {
    delay(1000);
    Serial.print(i);
    Serial.print(" ");
  }
  Serial.println(" ");
  
  //Store the central stick positions
  center_rc_value[RC_THROTTLE] = rc_throttle();
  center_rc_value[RC_RUDDER]   = rc_rudder();
  center_rc_value[RC_ELEVATOR] = rc_elevator();
  center_rc_value[RC_AILERON]  = rc_aileron();
  
  Serial.println(F(""));
  Serial.println(F("Center positions stored."));
  Serial.print(F("Center Throttle = ")); Serial.println(rc_throttle());
  Serial.print(F("Center Rudder = "));   Serial.println(rc_rudder());
  Serial.print(F("Center Elevator = ")); Serial.println(rc_elevator());
  Serial.print(F("Center Aileron = "));  Serial.println(rc_aileron());
  Serial.println(F(""));
  Serial.println(F(""));
  
  Serial.println(F("Move the throttle stick to full throttle and back to center"));  
  check_to_continue(RC_THROTTLE, 400);
    
  Serial.println(F(""));
  Serial.println(F(""));

  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("Gently move all the sticks simultaneously to their extends"));
  Serial.println(F("When ready put the sticks back in their center positions"));

  //Register the min and max values of the receiver channels
  register_min_max();
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("High, low and center values found during setup"));
  Serial.print(F("Throttle: ")); Serial.print(min_rc_value[RC_THROTTLE]); Serial.print(F(" - "));
  Serial.print(center_rc_value[RC_THROTTLE]); Serial.print(F(" - ")); Serial.println(max_rc_value[RC_THROTTLE]);
  
  Serial.print(F("Rudder: ")); Serial.print(min_rc_value[RC_RUDDER]); Serial.print(F(" - "));
  Serial.print(center_rc_value[RC_RUDDER]); Serial.print(F(" - ")); Serial.println(max_rc_value[RC_RUDDER]);
  
  Serial.print(F("Elevator: ")); Serial.print(min_rc_value[RC_ELEVATOR]); Serial.print(F(" - "));
  Serial.print(center_rc_value[RC_ELEVATOR]); Serial.print(F(" - ")); Serial.println(max_rc_value[RC_ELEVATOR]);
  
  Serial.print(F("Aileron: ")); Serial.print(min_rc_value[RC_AILERON]); Serial.print(F(" - "));
  Serial.print(center_rc_value[RC_AILERON]); Serial.print(F(" - ")); Serial.println(max_rc_value[RC_AILERON]);

  Serial.println(F("Move stick 'nose up' and back to center to continue"));
  check_to_continue(RC_ELEVATOR, 100);   
}

void check_to_continue(int channel, int min_move){
  byte continue_byte = 0;
  while (continue_byte == 0){
    if (nRcChannelData[channel] > center_rc_value[channel] + min_move) 
        continue_byte = 1;
    delay(100);
  }
  Serial.println("Detected movement in RC .... move to center now");
  wait_sticks_zero();
}

//Check if the transmitter sticks are in the neutral position - must be all of them
void wait_sticks_zero(){
  byte zero = 0;
  while(zero < 15){
    zero = 0;
    if (rc_throttle() < center_rc_value[RC_THROTTLE] + 20 && rc_throttle() > center_rc_value[RC_THROTTLE] - 20) zero |= 0b00000001;
    if (rc_rudder()   < center_rc_value[RC_RUDDER] + 20   && rc_rudder()   > center_rc_value[RC_RUDDER] - 20)   zero |= 0b00000010;
    if (rc_elevator() < center_rc_value[RC_ELEVATOR] + 20 && rc_elevator() > center_rc_value[RC_ELEVATOR] - 20) zero |= 0b00000100;
    if (rc_aileron()  < center_rc_value[RC_AILERON] + 20  && rc_aileron()  > center_rc_value[RC_AILERON] - 20)  zero |= 0b00001000;
    delay(100);
  }
}

//Checck if the receiver values are valid within 10 seconds
int wait_for_receiver(){
  byte zero = 0;
  long int timer = millis() + 10000;
  while (timer > millis() && zero < 15) {
    if (nRcChannelData[RC_THROTTLE] < 2100 && nRcChannelData[RC_THROTTLE] > 900) zero |= 0b00000001;
    if (nRcChannelData[RC_RUDDER]   < 2100 && nRcChannelData[RC_RUDDER]   > 900) zero |= 0b00000010;
    if (nRcChannelData[RC_ELEVATOR] < 2100 && nRcChannelData[RC_ELEVATOR] > 900) zero |= 0b00000100;
    if (nRcChannelData[RC_AILERON]  < 2100 && nRcChannelData[RC_AILERON]  > 900) zero |= 0b00001000;
    delay(500);
    Serial.print(F("."));
  }
  
  if(zero == 0) {
    Serial.println(F("."));
    Serial.println(F("No valid receiver signals found!!! (ERROR 1)"));
    return 1;
  } else {
    Serial.println(F(" OK"));
    return 0;
  }
}

//Register the min and max receiver values and exit when the sticks are back in the neutral position
void register_min_max(){
  byte zero = 0;
  min_rc_value[RC_THROTTLE] = rc_throttle();
  min_rc_value[RC_RUDDER] = rc_rudder();
  min_rc_value[RC_ELEVATOR] = rc_elevator();
  min_rc_value[RC_AILERON] = rc_aileron();

  Serial.println(F("Make sure throttle stick is at central position ...."));
  
  while (rc_throttle() < center_rc_value[RC_THROTTLE] + 50 && rc_throttle() > center_rc_value[RC_THROTTLE] - 50)
     delay(250);
  Serial.println(F("Measuring endpoints...."));
  delay(1000);
  
  while(zero < 15){
    if(min_rc_value[RC_THROTTLE] < center_rc_value[RC_THROTTLE] - 350 && max_rc_value[RC_THROTTLE] > center_rc_value[RC_THROTTLE] + 350) zero |= 0b00000001;
    if(min_rc_value[RC_RUDDER]   < center_rc_value[RC_RUDDER] - 350   && max_rc_value[RC_RUDDER]   > center_rc_value[RC_RUDDER]   + 350) zero |= 0b00000010;
    if(min_rc_value[RC_ELEVATOR] < center_rc_value[RC_ELEVATOR] - 350 && max_rc_value[RC_ELEVATOR] > center_rc_value[RC_ELEVATOR] + 350) zero |= 0b00000100;
    if(min_rc_value[RC_AILERON]  < center_rc_value[RC_AILERON] - 350  && max_rc_value[RC_AILERON]  > center_rc_value[RC_AILERON]  + 350) zero |= 0b00001000;
    
    if(rc_throttle() < min_rc_value[RC_THROTTLE]) min_rc_value[RC_THROTTLE] = rc_throttle();
    if(rc_throttle() > max_rc_value[RC_THROTTLE]) max_rc_value[RC_THROTTLE] = rc_throttle();

    if(rc_rudder() < min_rc_value[RC_RUDDER]) min_rc_value[RC_RUDDER] = rc_rudder();
    if(rc_rudder() > max_rc_value[RC_RUDDER]) max_rc_value[RC_RUDDER] = rc_rudder();

    if(rc_elevator() < min_rc_value[RC_ELEVATOR]) min_rc_value[RC_ELEVATOR] = rc_elevator();
    if(rc_elevator() > max_rc_value[RC_ELEVATOR]) max_rc_value[RC_ELEVATOR] = rc_elevator();

    if(rc_aileron() < min_rc_value[RC_AILERON]) min_rc_value[RC_AILERON] = rc_aileron();
    if(rc_aileron() > max_rc_value[RC_AILERON]) max_rc_value[RC_AILERON] = rc_aileron();

    delay(100);
  }
}
