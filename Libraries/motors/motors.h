#ifndef motors_h
#define motors_h

#include <Arduino.h>

#define CONNECTED_SERVOS 4

// Assign your channel out pins
#define THROTTLE_OUT_PIN(x) (x+4)

//  Assign # of servos
#define CONNECTED_SERVOS 4

// Assign servo indexes
#define SERVO_THROTTLE(x) (x)
#define SERVO_FRAME_SPACE 8

#include <Arduino.h>

	void attachMotors();

	void writeMotors(int array[]);
	
#endif
