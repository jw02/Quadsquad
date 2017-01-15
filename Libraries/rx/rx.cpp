// includes
#include <RCArduinoFastLib.h>
#include <PinChangeInt.h>
#include <rx.h>

// Assign your channel in pins
#define ROLL_IN_PIN 10
#define PITCH_IN_PIN 11
#define THROTTLE_IN_PIN 12
#define YAW_IN_PIN 13
#define AUTO_HOV_IN_PIN 14

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define PITCH_FLAG 2
#define ROLL_FLAG 4
#define YAW_FLAG 8

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile int16_t unThrottleInShared;
volatile int16_t unPitchInShared;
volatile int16_t unRollInShared;
volatile int16_t unYawInShared;
volatile uint16_t unHoverInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint16_t unThrottleInStart;
uint16_t unPitchInStart;
uint16_t unRollInStart;
uint16_t unYawInStart;
uint16_t unHoverInStart;


// simple interrupt service routine
//
void calcThrottle() {
  if(PCintPort::pinState)
  {
    unThrottleInStart = TCNT1;
  }
  else
  {
    unThrottleInShared = (TCNT1 - unThrottleInStart)>>1;
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
  //Serial.println(unThrottleInShared);
}

// simple interrupt service routine
void calcPitch()
{
  if(PCintPort::pinState)
  {
    unPitchInStart = TCNT1;
  }
  else
  {
    unPitchInShared = (TCNT1 - unPitchInStart)>>1;
    bUpdateFlagsShared |= PITCH_FLAG;
  }
  //Serial.println(unPitchInShared);
}

// simple interrupt service routine
void calcRoll()
{
  if(PCintPort::pinState)
  {
    unRollInStart = TCNT1;
  }
  else
  {
    unRollInShared = (TCNT1 - unRollInStart)>>1;
    bUpdateFlagsShared |= ROLL_FLAG;
  }
  //Serial.println(unRollInShared);
}

void calcYaw()
{
  if(PCintPort::pinState)
  {
    unYawInStart = TCNT1;
  }
  else
  {
    unYawInShared = (TCNT1 - unYawInStart)>>1;
    bUpdateFlagsShared |= YAW_FLAG;
  }
  //Serial.println(unYawInShared);
}

// simple interrupt service routine
void hoverMode()
{
  if(PCintPort::pinState) {
    unHoverInStart = TCNT1;
  }
  else {
    unHoverInShared = (TCNT1 - unHoverInStart)>>1;
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
  //Serial.println(unHoverInShared);
}


void attachISR() {

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(ROLL_IN_PIN, calcRoll,CHANGE);   
  PCintPort::attachInterrupt(PITCH_IN_PIN, calcPitch,CHANGE);
  PCintPort::attachInterrupt(YAW_IN_PIN, calcYaw,CHANGE);   
  PCintPort::attachInterrupt(AUTO_HOV_IN_PIN, hoverMode,CHANGE);
  //
}

void rxUpdate(int16_t &throttle, int16_t &hover, int16_t &pitch, int16_t &roll, int16_t &yaw, uint8_t &flags) {

  if(bUpdateFlagsShared) {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
  
    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    flags = bUpdateFlagsShared;
  
    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
    //
    if(flags & THROTTLE_FLAG) {
      throttle = unThrottleInShared;
      hover = unHoverInShared;
    }
    
    if(flags & PITCH_FLAG) pitch = unPitchInShared;
    if(flags & ROLL_FLAG) roll = unRollInShared;
    if(flags & YAW_FLAG) yaw = unYawInShared;
        
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
    
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }
}
