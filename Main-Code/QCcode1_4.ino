#include <PIDv1.h>
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
#include <Arduino.h>





//  SETTING UP FOR MPU STUFF  //
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

//  MPU6050 #defines
// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

#define MPU_PIN 15 // use pin 2 on Arduino Uno & most boards
//#define LED_PIN 15 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

//bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float yrp[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}





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

  //READ IN INITIAL PITCH, ROLL AND YAW OFFSETS
  //FROM THE TRANSMITTER
  pitchOffset = pulseIn(PITCH_IN_PIN, HIGH);
  Serial.println("Pitch: ");
  Serial.print(pitchOffset);
  rollOffset = pulseIn(ROLL_IN_PIN, HIGH);
  Serial.println("Roll: ");
  Serial.print(rollOffset);
  yawOffset = pulseIn(YAW_IN_PIN, HIGH);
  Serial.println("Yaw: ");
  Serial.print(yawOffset);



/**
//  MORE MPU SETUP  //
// join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
 // initialize device
  mpu.initialize();
  pinMode(MPU_PIN, INPUT);

  // verify connection
  //Serial.println(F("Testing device connections..."));
  if(!mpu.testConnection()) return;

  // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        // OLD INTERRUPT ROUTINE attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        PCintPort::attachInterrupt(MPU_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    //pinMode(LED_PIN, OUTPUT);
  
/**/





  /**/
  //  SETUP PID FIRST
  //
  setControllerDirection(0);
  setMode(1);

  // PID needs to be applied to as many variables that need to be controlled
  //
  // PITCH then ROLL then YAW
  setOutputLimits(MIN_PRY, MAX_PRY);
  setOutputLimits(MIN_PRY, MAX_PRY);
  setOutputLimits(MIN_PRY, MAX_PRY);
    
  // PITCH then ROLL then YAW
  setTunings(0.7, 0.2, 0.2);
  setTunings(0.2, 0.2, 0.2);
  setTunings(0.0, 0.0, 0.0);
  
  setNewSampleTime(0);
  //
  //  END PID SETUP
  /**/





  
  //  SETUP QUAD
  //  
  for(int nServo = 0;nServo < CONNECTED_SERVOS;nServo++) {
    CRCArduinoFastServos::attach(SERVO_THROTTLE(nServo), THROTTLE_OUT_PIN(nServo));
  }
  
  CRCArduinoFastServos::setup();

  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,2*2000);
  
  CRCArduinoFastServos::begin();

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


void loop() {
  //  CHECK for dmpReady to be true, if not do not go into loop...
  if(!dmpReady) return;
  
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that their values will be retained
  // between calls to loop.
  static int16_t unThrottleRaw;
  static int16_t unThrottleIn[4];
  static int16_t mpuVals[3];
  static int16_t offsetsIn[3];
  static int16_t offsetsInDeg[3];
  static uint16_t unHoverIn;
  
  // local copy of update flags
  static uint8_t bUpdateFlags;

if(bUpdateFlagsShared) {
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
    offsetsIn[0] = unPitchInShared;
  }
  if(bUpdateFlags & ROLL_FLAG) {
    offsetsIn[1] = unRollInShared;
  }
  if(bUpdateFlags & YAW_FLAG) {
    offsetsIn[2] = unYawInShared;
  }
      
  // clear shared copy of updated flags as we have already taken the updates
  // we still have a local copy if we need to use it in bUpdateFlags
  bUpdateFlagsShared = 0;
  
  interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
  // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
  // service routines own these and could update them at any time. During the update, the
  // shared copies may contain junk. Luckily we have our local copies to work with :-)
}



/**
    //Serial.println(packetSize);
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }
    //Serial.println(packetSize);

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);
            
            
            //mpuVals[0] = yrp[2] * 180/M_PI;
            //mpuVals[1] = yrp[1] * 180/M_PI;
            //mpuVals[2] = yrp[0] * 180/M_PI;
            
            //Serial.print("yrp\t");
            //Serial.print(yrp[0] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(yrp[1] * 180/M_PI);
            //Serial.print("\t");
            //Serial.println(yrp[2] * 180/M_PI);
            
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        //blinkState = !blinkState;
        //digitalWrite(LED_PIN, blinkState);
    }

/**/




 
    
  if(unHoverIn > 2000) {
    unThrottleRaw = hovSpeed;
    
    //  ASSIGN THROTTLE VALUES BASED ON INTERRUPT
    for(int nServo = 0;nServo < CONNECTED_SERVOS;nServo++) unThrottleIn[nServo] = unThrottleRaw;
    
    }
  else {
    
  //  ASSIGN THROTTLE VALUES BASED ON INTERRUPT
  for(int nServo = 0;nServo < CONNECTED_SERVOS;nServo++) unThrottleIn[nServo] = unThrottleRaw;
  
   if(offsetsInDeg[0] < MIN_PRY) offsetsInDeg[0] = MIN_PRY;
  if(offsetsInDeg[0] > MAX_PRY) offsetsInDeg[0] = MAX_PRY;
  offsetsInDeg[0] = -(map(offsetsIn[0], MIN_PRY, MAX_PRY, MIN_PR, MAX_PR));

  if(offsetsInDeg[1] < MIN_PRY) offsetsInDeg[1] = MIN_PRY;
  if(offsetsInDeg[1] > MAX_PRY) offsetsInDeg[1] = MAX_PRY;
  offsetsInDeg[1] = map(offsetsIn[1], MIN_PRY, MAX_PRY, MIN_PR, MAX_PR);
  
  if(offsetsInDeg[2] < MIN_PRY) offsetsInDeg[2] = MIN_PRY;
  if(offsetsInDeg[2] > MAX_PRY) offsetsInDeg[2] = MAX_PRY;
  offsetsInDeg[2] = map(offsetsIn[2], MIN_PRY, MAX_PRY, MIN_YW, MAX_YW);
  
  /**
  // APPLY PID ALGORITHM TO OBTAIN TRUE OFFSETS
  //  PITCH
  int tempOff0 = offsetsInDeg[0];
  int tempMPU0 = mpuVals[0];
  //Serial.println(tempOff0);
  offsetsInDeg[0] = compute(tempMPU0, tempOff0);
  //Serial.println(offsetsInDeg[0]);
  
  //  ROLL
  int tempOff1 = offsetsInDeg[1];
  int tempMPU1 = mpuVals[1];
  //offsetsInDeg[1] = compute(offsetsInDeg[1], mpuVals[1]);
  
  //  YAW
  int tempOff2 = offsetsInDeg[2];
  int tempMPU2 = mpuVals[2];
  //offsetsInDeg[2] = compute(offsetsInDeg[2], mpuVals[2]);
  /**/
  
  //PITCH, ROLL, AND YAW APPLIED TO THROTTLE
  /**
  unThrottleIn[0] -= ((offsetsInDeg[0] + offsetsInDeg[1] - offsetsInDeg[2]));
  if(unThrottleIn[0] < (MIN_THROTTLE+50)) unThrottleIn[0] = MIN_THROTTLE - 50;
  
  unThrottleIn[1] -= ((offsetsInDeg[0] - offsetsInDeg[1] + offsetsInDeg[2]));
  if(unThrottleIn[1] < (MIN_THROTTLE+50)) unThrottleIn[1] = MIN_THROTTLE - 50;
  
  unThrottleIn[2] += ((offsetsInDeg[0] + offsetsInDeg[1] + offsetsInDeg[2]));
  if(unThrottleIn[2] < (MIN_THROTTLE+50)) unThrottleIn[2] = MIN_THROTTLE - 50;
  
  unThrottleIn[3] += ((offsetsInDeg[0] - offsetsInDeg[1] - offsetsInDeg[2]));
  if(unThrottleIn[3] < (MIN_THROTTLE+50)) unThrottleIn[3] = MIN_THROTTLE - 50;
  /**/
  }
  
  //Serial.println(unThrottleRaw);
  
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
