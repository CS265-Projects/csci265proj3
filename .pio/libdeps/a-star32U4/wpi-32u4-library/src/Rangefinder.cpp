#include <Rangefinder.h>
#include <pcint.h>

#define ECHO_RECD   0x02

/** \brief Constructor.
 * 
 * @param echo The echo pin. Must be interrupt enabled. PCInts OK.
 * @param trig The trigger pin.
 * */
Rangefinder::Rangefinder(uint8_t echo, uint8_t trig) 
{
    echoPin = echo;
    trigPin = trig;
}

// sets up the interface
void Rangefinder::init(void)
{
  // assert ECHO pin is an input
  pinMode(echoPin, INPUT);

  // register the interrupt for the echo
  if(digitalPinToInterrupt(echoPin) != NOT_AN_INTERRUPT)
  {
    Serial.println("Attaching rangefinder ISR");
    attachInterrupt(digitalPinToInterrupt(echoPin), ::ISR_Rangefinder, CHANGE);
  }
  else if(digitalPinToPCInterrupt(echoPin) != NOT_AN_INTERRUPT)
  {
    Serial.println("Attaching rangefinder PC_ISR");
    attachPCInt(digitalPinToPCInterrupt(echoPin), ::ISR_Rangefinder);
  }
  else
  {
    Serial.println("Not a rangefinder interrupt pin!");
  }

  //control pin for commanding pings must be an output
  pinMode(trigPin, OUTPUT);
}

/**
 * \brief checkPingTimer() checks to see if it's time to send a new ping.
 * 
 * You can make the pingInterval arbitrarily small, since it won't send a ping
 * if the ECHO pin is HIGH.
 * 
 * getDistance() calls this function, so you don't need to call this function manually.
 */
uint8_t Rangefinder::checkPingTimer(void)
{
    // if the echo pin is still high, just update the last ping time
    if(digitalRead(echoPin)) lastPing = millis();

    // check if we're ready to ping
    if(millis() - lastPing >= pingInterval)
    {
        //disable interrupts while we adjust the ISR variables
        cli();
        pulseEnd = pulseStart = 0;

        //clear out any leftover states
        state = 0;
        sei();

        // keep track of when we sent the last ping
        lastPing = millis();  

        // toggle the trigger pin to send a chirp
        digitalWrite(trigPin, HIGH); //commands a ping; leave high for the duration
        delayMicroseconds(30); //datasheet says hold HIGH for >20us; we'll use 30 to be 'safe'
        digitalWrite(trigPin, LOW); //unclear if pin has to stay HIGH
    }

    return state;
}

uint16_t Rangefinder::checkEcho(void)
{
    uint16_t echoLength = 0;
    if(state & ECHO_RECD)
    {
        cli();
        echoLength = pulseEnd - pulseStart;
        state &= ~ECHO_RECD;
        sei();
    }

    return echoLength;
}

float Rangefinder::getDistance(void)
{
    uint16_t pulseDur = checkEcho();
    if(pulseDur) distance = pulseDur / 58.0;

    // After we've checked for an echo, check to send the next ping
    checkPingTimer();

    return distance;
}

/** \brief ISR for the echo pin
 * 
 * Records both the start and stop (rise and fall) of the echo pin.
 * When the pin goes low, it sets a flag.
 * */
void Rangefinder::ISR_echo(void)
{
    if(digitalRead(echoPin))  //transitioned to HIGH
    {
        pulseStart = micros();
    }

    else                      //transitioned to LOW
    {
        pulseEnd = micros();
        state |= ECHO_RECD;
    } 
}

/** A global ISR, which calls Rangefinder::ISR_echo()
 * */
void ISR_Rangefinder(void)
{
    rangefinder.ISR_echo();
}