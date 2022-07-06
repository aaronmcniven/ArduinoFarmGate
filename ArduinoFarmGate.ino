/* ArduinoFarmGate - Aaron McNiven */
/* ATmega328P */

#include <EEPROM.h>

#include "SimpleTimer.h"
#include "SimpleActuator.h"

#include "ADS1X15.h"

/* Pins: */

#define PIN_RELAY_LOCK          4         /* Lock relay. */
#define PIN_RELAY_OPEN          2         /* H Bridge open relay. */
#define PIN_RELAY_CLOSE         3         /* H Bridge close relay. */

#define PIN_TRIP_LED            7         /* Actuator over-current LED output. Signal from ACS711. */

#define PIN_ACS711_FAULT        8         /* Signal from ACS711. Normally high. Pulled low under fault condition. */

#define PIN_STAT_BIT_OUT_0      A0        /* 4 Bit output to MKR board for status. */
#define PIN_STAT_BIT_OUT_1      A1        /* 4 Bit output to MKR board for status. */
#define PIN_STAT_BIT_OUT_2      A2        /* 4 Bit output to MKR board for status. */
#define PIN_STAT_BIT_OUT_3      A3        /* 4 Bit output to MKR board for status. */

#define PIN_TRIG_1              11        /* Spare. */
#define PIN_TRIG_2              12        /* Spare. */
#define PIN_TRIG_3              13        /* Spare. */

#define PIN_IN_OPENCLOSE        5         /* Pull high to trigger normal open and close cycle. */
#define PIN_IN_OPENKO           6         /* Pull high to trigger open and keep open. */
#define PIN_IN_LOCK             10        /* Pull high to toggle lock. */

#define PIN_IN_ENDSTOP          9         /* Endstop input. Pull high to trigger stop event. */

#define openWaitTimeMs          60000     /* Automatically close after this time. */
#define openCloseTimeoutMs      45000     /* Trigger fault after this time with no stop event. */

/* Timers: */

SimpleTimer openWaitTime;
SimpleTimer openCloseTimeout;
SimpleTimer stallTimer;

/* Objects: */

SimpleActuator actuator;

/* ADS1115 ADC */

/* Input 0 is current sense output from ACS711. */
/* Input 1 is trip threshold from potentiometer. */
/* Input 2 and 3 is unused. */

ADS1115 ADS(0x48);
int16_t tripADC = 0;
bool eeprom_adc = false;

/* States: */

enum State {  /* Stat bit output: */
  FAULT,      /* 0 - 0000 */
  OPENING,    /* 1 - 0001 */
  OPENINGKO,  /* 2 - 0010 */
  OPEN,       /* 3 - 0011 */
  OPENKO,     /* 4 - 0100 */
  CLOSING,    /* 5 - 0101 */
  CLOSED,     /* 6 - 0110 */
  STOPPED,    /* 7 - 0111 */
  UNK         /* 8 - 1000 */
};

int state = State::UNK;
int memState = State::UNK;

/* Functions */

bool setStatBits() { /* Returns true if values are different from previous function call. */
  
  if(state == 0) { /* Fault / Boot */
    digitalWrite(PIN_STAT_BIT_OUT_0, false);
    digitalWrite(PIN_STAT_BIT_OUT_1, false);
    digitalWrite(PIN_STAT_BIT_OUT_2, false);
    digitalWrite(PIN_STAT_BIT_OUT_3, false);
  }

  if(state == 1) { /* Opening */
    digitalWrite(PIN_STAT_BIT_OUT_0, false);
    digitalWrite(PIN_STAT_BIT_OUT_1, false);
    digitalWrite(PIN_STAT_BIT_OUT_2, false);
    digitalWrite(PIN_STAT_BIT_OUT_3, true);
  }

  if(state == 2) { /* Opening - Keep Open */
    digitalWrite(PIN_STAT_BIT_OUT_0, false);
    digitalWrite(PIN_STAT_BIT_OUT_1, false);
    digitalWrite(PIN_STAT_BIT_OUT_2, true);
    digitalWrite(PIN_STAT_BIT_OUT_3, false);
  }

  if(state == 3) { /* Open */
    digitalWrite(PIN_STAT_BIT_OUT_0, false);
    digitalWrite(PIN_STAT_BIT_OUT_1, false);
    digitalWrite(PIN_STAT_BIT_OUT_2, true);
    digitalWrite(PIN_STAT_BIT_OUT_3, true);
  }

  if(state == 4) { /* Keep Open */
    digitalWrite(PIN_STAT_BIT_OUT_0, false);
    digitalWrite(PIN_STAT_BIT_OUT_1, true);
    digitalWrite(PIN_STAT_BIT_OUT_2, false);
    digitalWrite(PIN_STAT_BIT_OUT_3, false);
  }

  if(state == 5) { /* Closing */
    digitalWrite(PIN_STAT_BIT_OUT_0, false);
    digitalWrite(PIN_STAT_BIT_OUT_1, true);
    digitalWrite(PIN_STAT_BIT_OUT_2, false);
    digitalWrite(PIN_STAT_BIT_OUT_3, true);
  }

  if(state == 6) { /* Closed */
    digitalWrite(PIN_STAT_BIT_OUT_0, false);
    digitalWrite(PIN_STAT_BIT_OUT_1, true);
    digitalWrite(PIN_STAT_BIT_OUT_2, true);
    digitalWrite(PIN_STAT_BIT_OUT_3, false);
  }

  if(state == 7) { /* Stopped */
    digitalWrite(PIN_STAT_BIT_OUT_0, false);
    digitalWrite(PIN_STAT_BIT_OUT_1, true);
    digitalWrite(PIN_STAT_BIT_OUT_2, true);
    digitalWrite(PIN_STAT_BIT_OUT_3, true);
  }

  if(state == 8) { /* Unknown */
    digitalWrite(PIN_STAT_BIT_OUT_0, true);
    digitalWrite(PIN_STAT_BIT_OUT_1, false);
    digitalWrite(PIN_STAT_BIT_OUT_2, false);
    digitalWrite(PIN_STAT_BIT_OUT_3, false);
  }

  if(memState == state) {
    return false;
  }
    
  memState = state;
  return true;
}

bool stallDetect() {

  if(state == State::UNK) {
    return false;
  }

  /* Trip if tripADC is exceeded for timer duration. */

  int16_t readADC = ADS.readADC(0);

  if(readADC >= tripADC) {
    digitalWrite(PIN_TRIP_LED, HIGH);
  } else {
    digitalWrite(PIN_TRIP_LED, LOW);
  }

  if(readADC >= tripADC) {
    
    if(state == State::CLOSING || state == State::OPENING || state == State::OPENINGKO) {
      stallTimer.startTimer();
    }

    if(stallTimer.expired(false)) {
      stallTimer.stopTimer();
      return true;
    }
    
  } else {
    stallTimer.stopTimer();
  }

  return false;
}

void pulseTripLED(int count) {
  for(int i = 0; i < count; ++i) {
    digitalWrite(PIN_TRIP_LED, HIGH);
    delay(300);
    digitalWrite(PIN_TRIP_LED, LOW);
    delay(300);
  }
}

void setup() {

  Serial.begin(9600);

  ADS.begin();

  ADS.setGain(1);
  ADS.setMode(0);

  ADS.requestADC(0);
  ADS.requestADC(1);

  delay(500);

  /* Read trip potentiometer. */
  tripADC = ADS.readADC(1);

  pinMode(PIN_RELAY_LOCK, OUTPUT);
  pinMode(PIN_RELAY_OPEN, OUTPUT);
  pinMode(PIN_RELAY_CLOSE, OUTPUT);

  pinMode(PIN_TRIP_LED, OUTPUT);
  pinMode(PIN_ACS711_FAULT, INPUT);

  pinMode(PIN_STAT_BIT_OUT_0, OUTPUT);
  pinMode(PIN_STAT_BIT_OUT_1, OUTPUT);
  pinMode(PIN_STAT_BIT_OUT_2, OUTPUT);
  pinMode(PIN_STAT_BIT_OUT_3, OUTPUT);

  pinMode(PIN_IN_OPENCLOSE, INPUT);
  pinMode(PIN_IN_OPENKO, INPUT);
  pinMode(PIN_IN_LOCK, INPUT);

  setStatBits();

  actuator.setPins(PIN_RELAY_OPEN, PIN_RELAY_CLOSE, PIN_RELAY_LOCK);

  openWaitTime.setDuration(openWaitTimeMs);
  openCloseTimeout.setDuration(openCloseTimeoutMs);
  stallTimer.setDuration(250);
}

void loop() {

  /* Possible short circuit: */
  
  if(!digitalRead(PIN_ACS711_FAULT)) {
    actuator.stop(true);
  }

  if(actuator.disabled()) {
    
    state = State::FAULT;
    
    digitalWrite(PIN_TRIP_LED, LOW);
    delay(1500);
    digitalWrite(PIN_TRIP_LED, HIGH);
    delay(75);
    digitalWrite(PIN_TRIP_LED, LOW);
    delay(75);
    digitalWrite(PIN_TRIP_LED, HIGH);
    delay(75);
    digitalWrite(PIN_TRIP_LED, LOW);
    delay(75);
    digitalWrite(PIN_TRIP_LED, HIGH);
    delay(75);
    digitalWrite(PIN_TRIP_LED, LOW);
    delay(75);
    digitalWrite(PIN_TRIP_LED, HIGH);
    delay(75);
    digitalWrite(PIN_TRIP_LED, LOW);
    delay(1500);
  }

  setStatBits();
 
  /* Update timers: */

  openWaitTime.updateTimer();
  openCloseTimeout.updateTimer();
  stallTimer.updateTimer();
   
  /* Handle input buttons: */

  if(!digitalRead(PIN_IN_LOCK)) {
    
    if(digitalRead(PIN_IN_OPENCLOSE)) {

      if(state == State::CLOSED) {
        
          state = State::OPENING;
          actuator.beginOpen();
          openCloseTimeout.restartTimer();
      }
      
      if(state == State::OPEN || state == State::OPENKO || state == State::UNK || state == State::FAULT || state == State::STOPPED) {
        
          state = State::CLOSING;
          actuator.beginClose();
          openCloseTimeout.restartTimer();
      }
    }

    if(digitalRead(PIN_IN_OPENKO)) {
      
      if(state == State::CLOSED || state == State::UNK || state == State::FAULT || state == State::STOPPED) {
        
          state = State::OPENINGKO;
          actuator.beginOpen();
          openCloseTimeout.restartTimer();
      }
    }
  }

  /* Handle actuator stop events: */
  
  if(state == State::CLOSING || state == State::OPENING || state == State::OPENINGKO) {

    /* If openCloseTimeout expires while the actuator is moving, trigger fault: */

    openWaitTime.stopTimer();
    
    if(openCloseTimeout.expired(false)) {
      actuator.stop();
      state = State::FAULT;
    }

    /* If PIN_IN_OPENCLOSE goes high while the actuator is moving, stop: */

    if(digitalRead(PIN_IN_OPENCLOSE) && openCloseTimeout.getElapsed() > 1500) {
      actuator.stop();
      state = State::STOPPED;
      delay(2500);
    }
  }

  /* In an unknown state, allow trip sense LED to function: */

  if(state == State::UNK) {
    
    int16_t ADS_Stall = ADS.readADC(0);

    if(!eeprom_adc) {
      tripADC = ADS.readADC(1);
    }
    
    if(ADS_Stall >= tripADC) {
      digitalWrite(PIN_TRIP_LED, HIGH);
    } else {
      digitalWrite(PIN_TRIP_LED, LOW);
    }
  }

  /* Disable trip sense LED if actuator is not moving and not in an unknown state: */

  if(state == State::CLOSED || state == State::OPEN || state == State::OPENKO || state == State::STOPPED || state == State::FAULT) {
    digitalWrite(PIN_TRIP_LED, LOW);
  }

  /* Normal stop events: */

  if(state == State::CLOSING && (digitalRead(PIN_IN_ENDSTOP) || stallDetect()) && openCloseTimeout.getElapsed() > 100) {
    state = State::CLOSED;
    actuator.stop();
    pulseTripLED(2);
  }

  if(state == State::OPENING && (digitalRead(PIN_IN_ENDSTOP) || stallDetect()) && openCloseTimeout.getElapsed() > 100) {
    state = State::OPEN;
    actuator.stop();
    openWaitTime.restartTimer();
    pulseTripLED(3);
  }

  if(state == State::OPENINGKO && (digitalRead(PIN_IN_ENDSTOP) || stallDetect()) && openCloseTimeout.getElapsed() > 100) {
    state = State::OPENKO;
    actuator.stop();
    pulseTripLED(4);
  }

  if(state == State::OPEN && openWaitTime.expired(false)) {
    state = State::CLOSING;
    actuator.beginClose();
    openCloseTimeout.restartTimer();
  }
}
