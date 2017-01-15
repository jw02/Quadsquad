#ifndef rx_h
#define rx_h

#include <Arduino.h>

    //  Using the PinChangeInt library, we will attach the interrupts
    //  used to read the channels
    void attachISR();

    //  Update the local copy of the throttle, pitch, roll, and yaw variables
    void rxUpdate(int16_t &, int16_t &, int16_t &, int16_t &, int16_t &, uint8_t &);

    void calcThrottle();

    void calcPitch();

    void calcRoll();

    void calcYaw();
    
#endif
