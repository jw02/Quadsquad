// includes
#include <motors.h>
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
//#include <PinChangeInt.h>

void attachMotors() {
  
  //  SETUP QUAD
  //  
  for(int i= 0; i < CONNECTED_SERVOS; i++) {
    CRCArduinoFastServos::attach(SERVO_THROTTLE(i), THROTTLE_OUT_PIN(i));
  }
  
  CRCArduinoFastServos::setup();

  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,2*2000);
  
  CRCArduinoFastServos::begin();
  
}

void writeMotors(int mThrottle[4]) {

  for(int i = 0; i < CONNECTED_SERVOS; i++) {
    CRCArduinoFastServos::writeMicroseconds(SERVO_THROTTLE(i), mThrottle[i]);
  }
}

