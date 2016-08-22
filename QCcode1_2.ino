//#include <PIDv1.h>
#include <RCArduinoFastLib.h>

 // MultiChannels
//
// rcarduino.blogspot.com
//
// A simple approach for reading three RC Channels using pin change interrupts
//
// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
// 

// include the pinchangeint library - see the links in the related topics section above for details
#include <PinChangeInt.h>


// Assign your channel in pins
#define THROTTLE_IN_PIN 12
#define ROLL_IN_PIN 10
#define PITCH_IN_PIN 11
#define YAW_IN_PIN 13
#define AUTO_HOV_IN_PIN 14

// Assign your channel out pins
#define THROTTLE_OUT_PIN(x) (x+4)

//  Assign # of servos
#define CONNECTED_SERVOS 4

// Assign servo indexes
#define SERVO_THROTTLE(x) (x)
#define SERVO_FRAME_SPACE 8

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

#define MAX_THROTTLE 1972 // this is the duration in microseconds of neutral throttle on an electric RC Car
#define NEUTRAL_THROTTLE 1500 // this is the duration in microseconds of neutral throttle on an electric RC Car
#define MIN_THROTTLE 994 // this is the duration in microseconds of neutral throttle on an electric RC Car

int hovSpeed = 1500;
int pitchOffset = 0;
int rollOffset = 0;
int yawOffset = 0;


uint16_t unLastAuxIn = 0;
uint32_t ulVariance = 0;
uint32_t ulGetNextSampleMillis = 0;
uint16_t unMaxDifference = 0;


//
//  PID
//  definitions
#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0
#define REVERSE 1

//  declarations
unsigned long tik;
double Input, Output, setPoint;
double lastPV;
double kp, ki, kd, P, I, D;
int sampleTime = 1000;
double outMin, outMax;
bool inAuto = false;

int controllerDirection = DIRECT;

void setControllerDirection (int Direction){
  controllerDirection = Direction;
}

void initialize() {
  lastPV = Input;
  I = Output;
  if(I > outMax) I = outMax;
  else if(I < outMin) I = outMin;  
}

void setMode(int Mode){
  bool newAuto = (Mode == AUTOMATIC);
  if(newAuto && !inAuto) {
    initialize();
  }
  inAuto = newAuto;
}

void setOutputLimits(double Min, double Max) {
  if(Min > Max) return;
  outMin = Min;
  outMax = Max;

  if(Output > outMax) Output = outMax;
  else if(Output < outMin) Output = outMin;

  if(I > outMax) I = outMax;
  else if(I < outMin) I = outMin;
}

void setNewSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)sampleTime;
      ki *= ratio;
      kd /= ratio;
      sampleTime = (unsigned long)NewSampleTime;
   }
}

void setTunings(double Kp, double Ki, double Kd){

  if(Kp < 0 || Ki < 0 || Kd < 0) return;
  
  double SampleTimeInSec = double(sampleTime/1000.0);
  
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;

  if(controllerDirection == REVERSE){
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
}

void compute(double sP, double pV){
  
  if(!inAuto) return;
  unsigned long tok = millis();
  int tiktok = tok - tik;
  
  if(tiktok >= sampleTime){
    //Serial.println(pV);
    double error = sP - pV;
    I += ki*error;
    double dPV = (pV - lastPV);
        
    P = kp*error;
    D = -kd*dPV;

    /**
    Serial.print(error);
    Serial.print(" ");
    Serial.print(errSum);
    Serial.print(" ");
    Serial.println(dErr);
    /**/
    
    Output = P + I + D;
    if(Output > outMax){
      I -= Output - outMax;
      Output = outMax;
    }
    else if(Output < outMin){
      I += outMin - Output;
      Output = outMin;
    }
    
    //Serial.println(Output);
    Input += Output;
    //Serial.println(Input);
    lastPV = Input;
    tik = tok;
  }
}
//  END PID
//

void setup()
{
  Serial.begin(115200);

  //  SETUP PID FIRST
  //
  setControllerDirection(0);
  setMode(1);
  setOutputLimits(0.0, 255.0);
  setNewSampleTime(0);
  //
  //  END PID SETUP






  
  //  SETUP QUAD
  //  
  for(int nServo = 0;nServo < CONNECTED_SERVOS;nServo++) {
    CRCArduinoFastServos::attach(SERVO_THROTTLE(nServo), THROTTLE_OUT_PIN(nServo));
  }
  
  CRCArduinoFastServos::setup();

  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,2*2000);
  
  CRCArduinoFastServos::begin();

  //READ IN INITIAL PITCH, ROLL AND YAW OFFSETS
  pitchOffset = pulseIn(PITCH_IN_PIN, HIGH);
  rollOffset = pulseIn(ROLL_IN_PIN, HIGH);
  yawOffset = pulseIn(YAW_IN_PIN, HIGH);

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle,CHANGE);
  PCintPort::attachInterrupt(ROLL_IN_PIN, calcRoll,CHANGE);
  PCintPort::attachInterrupt(PITCH_IN_PIN, calcPitch,CHANGE);
  PCintPort::attachInterrupt(YAW_IN_PIN, calcYaw,CHANGE);
  PCintPort::attachInterrupt(AUTO_HOV_IN_PIN, hoverMode,CHANGE);
  //
  //  END QUAD SETUP
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that their values will be retained
  // between calls to loop.
  static int16_t unThrottleRaw;
  static int16_t unThrottleIn[4];
  static int16_t offsetsIn[3];
  static uint16_t unHoverIn;
  
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.
    //
    if(bUpdateFlags & THROTTLE_FLAG) {
      unThrottleRaw = unThrottleInShared;
      unHoverIn = unHoverInShared;
    }
    if(bUpdateFlags & PITCH_FLAG) {
      offsetsIn[0] = unPitchInShared - pitchOffset;
    }
    if(bUpdateFlags & ROLL_FLAG) {
      offsetsIn[1] = unRollInShared - rollOffset;
    }
    if(bUpdateFlags & YAW_FLAG) {
      offsetsIn[2] = unYawInShared - yawOffset;
    }
        
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
   
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }  

  //  ASSIGN THROTTLE VALUES BASED ON INTERRUPT
  for(int nServo = 0;nServo < CONNECTED_SERVOS;nServo++) unThrottleIn[nServo] = unThrottleRaw;

  // APPLY PID ALGORITHM TO OBTAIN TRUE OFFSETS
  //  PITCH
  setTunings(kpP, kiP, kdP);
  compute(offsetsIn[0], imuPitch);

  //  ROLL
  setTunings(kpR, kiR, kdR);
  compute(offsetsIn[1], imuRoll);

  //  YAW
  setTunings(kpY, kiY, kdY);
  compute(offsetsIn[2], imuYaw);
    
  //PITCH, ROLL, AND YAW APPLIED TO THROTTLE
  unThrottleIn[0] -= ((offsetsIn[0] + offsetsIn[1] - offsetsIn[2])/2);
  unThrottleIn[1] -= ((offsetsIn[0] - offsetsIn[1] + offsetsIn[2])/2);
  unThrottleIn[2] += ((offsetsIn[0] + offsetsIn[1] + offsetsIn[2])/2);
  unThrottleIn[3] += ((offsetsIn[0] - offsetsIn[1] - offsetsIn[2])/2);
  
  if(unHoverIn > 2000) for(int nServo = 0;nServo < CONNECTED_SERVOS;nServo++) unThrottleIn[nServo] = hovSpeed;
  
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
  for(int nServo = 0;nServo < CONNECTED_SERVOS;nServo++) {
    CRCArduinoFastServos::writeMicroseconds(SERVO_THROTTLE(nServo),unThrottleIn[nServo]);
  }
  bUpdateFlags = 0;
}


// simple interrupt service routine
void calcThrottle()
{
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