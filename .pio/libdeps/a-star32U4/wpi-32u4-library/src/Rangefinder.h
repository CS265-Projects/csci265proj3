#pragma once

#include <Arduino.h>

/** \class Rangefinder 
 * \brief A class to manage the HC-SR04 ultrasonic rangefinder. 
 * 
 * Uses a TRIG and ECHO pin to send chirps and detect round trip time.
 * 
 * The object rangefinder is declared extern to work with the ISRs, which 
 * means you must define your object with the same name:
 * 
 * Rangefinder rangefinder(<echo>, <trigger>);
 * */
class Rangefinder 
{
protected:
    volatile uint8_t state = 0;

    uint8_t echoPin = -1;
    uint8_t trigPin = -1;

    // for keeping track of ping intervals
    uint32_t lastPing = 0;     

    // we set the pingInterval to 10 ms, but it won't actually ping that fast
    // since it _only_ pings if the ECHO pin is low -- that is, it will ping
    // in 10 ms or when the last echo is done, whichever is _longer_ 
    uint32_t pingInterval = 10;    

    // for keeping track of echo duration
    volatile uint32_t pulseStart = 0;
    volatile uint32_t pulseEnd = 0;

    // holds the last recorded distance
    float distance = 99;

public:
    Rangefinder(uint8_t echo, uint8_t trig);

    // must call init() to set up pins and interrupts
    void init(void);

    // checks to see if it's time to emit a ping
    uint8_t checkPingTimer(void);

    // checks to see if an echo is complete
    uint16_t checkEcho(void);

    /** \brief Returns the last recorded distance in cm. The first call to 
     * getDistance() will return 99.
     * */
    float getDistance(void);

    // ISR for the echo pin
    void ISR_echo(void);
};

// ISR for the echo
void ISR_Rangefinder(void);

// we declare as extern so we can refer to it in the ISR
extern Rangefinder rangefinder;
