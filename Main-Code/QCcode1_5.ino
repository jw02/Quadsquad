#include <motors.h>
#include <rx.h>
//#include <PIDv1.h>
#include <Arduino.h>


#define MAX_THROTTLE 1890// this is the duration in microseconds of neutral throttle on an electric RC Car
#define NEUTRAL_THROTTLE 1500 // this is the duration in microseconds of neutral throttle on an electric RC Car
#define MIN_THROTTLE 910 // this is the duration in microseconds of neutral throttle on an electric RC Car
#define MIN_PRY 975 // this is the duration in microseconds of neutral throttle on an electric RC Car
#define MAX_PRY 1975 // this is the duration in microseconds of neutral throttle on an electric RC Car
#define MIN_PR -45 //Minimum pitch for rc input mapping
#define MAX_PR 45 //Maximum pitch for rc input mapping
#define MIN_YW -150 //Minimum yaw for rc input mapping
#define MAX_YW 150 //Maximum yaw for rc input mapping

int hovSpeed = 1500;
int pitchOffset = 0;
int rollOffset = 0;
int yawOffset = 0;


uint16_t unLastAuxIn = 0;
uint32_t ulVariance = 0;
uint32_t ulGetNextSampleMillis = 0;
uint16_t unMaxDifference = 0;



void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  attachMotors();
  attachISR();
}

int counter = 0;
void loop() {
  
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that their values will be retained
  // between calls to loop.
  static int16_t unThrottleRaw;
  static int16_t unPitchRaw;
  static int16_t unRollRaw;
  static int16_t unYawRaw;
  static int16_t unHoverRaw;
  
  // local copy of update flags
  static uint8_t bUpdateFlags;

  int unThrottleIn[4];  
  
  rxUpdate(unThrottleRaw, unHoverRaw, unPitchRaw, unRollRaw, unYawRaw, bUpdateFlags);

  unThrottleIn[0] = unThrottleRaw;
  unThrottleIn[1] = unThrottleRaw;
  unThrottleIn[2] = unThrottleRaw;
  unThrottleIn[3] = unThrottleRaw;
  
  // do any processing from here onwards
  // only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
  // variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
  // the interrupt routines and should not be used in loop

  // the following code provides simple pass through
  // this is a good initial test, the Arduino will pass through
  // receiver input as if the Arduino is not there.
  // This should be used to confirm the circuit and power
  // before attempting any custom processing in a project.

  // we are checking to see if the channel value has changed, this is indicated 
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing
  // this allows us to only calculate new values when we have new inputs, rather than
  // on every cycle.
  writeMotors(unThrottleIn);
  /**
  for(int nServo = 0;nServo < CONNECTED_SERVOS;nServo++) {
    CRCArduinoFastServos::writeMicroseconds(SERVO_THROTTLE(nServo),unThrottleIn[nServo]);
  }
  /**/
  bUpdateFlags = 0;
}

