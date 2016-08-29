#ifndef PIDv1_h
#define PIDv1_h

#include <Arduino.h>

  #define MANUAL 0
  #define AUTOMATIC 1
  #define DIRECT 0
  #define REVERSE 1


  const int NUMBER_OF_VARIABLES  = 10;
  
  //  functions used

  //  sets direction for the PID algorithm, some instances need a decreasing output from
  //  an increasing input. IE a refrigerator
  void setControllerDirection(int); 

  //  determines if the algorithm is in automatic mode or not based on user input
  //  and decides whether to go into the rest of the code or not
  void setMode(int);

  //  bounds the values between a min and max passed to the function
  void setOutputLimits(double, double);

  //  gives the user the ability to change the sampling interval
  void setNewSampleTime(int);

  //  establishes ki, kp, kd values
  void setTunings(double, double, double);

  //  actually computes the new output
  int compute(int, int);

  //  ensures the PID loop starts from the proper spot
  void Initialize();


#endif
