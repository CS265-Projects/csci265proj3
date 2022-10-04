/**
 * Demonstration of controlling two servos simultanesouly. See documentation for servo.h/.cpp for
 * more details.
 * 
 * */ 

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include <Chassis.h>

// Include Servo32u4 library
#include <Servo32u4.h>

// Declare a chassis object with nominal dimensions
// In practice, adjust the parameters: wheel diam, encoder counts, wheel track
Chassis chassis(7.0, 1440, 14.9);

// These can be any two of: 5, 6, 12, 13, EXCEPT both 6 and 12 at the same time (see library for reason)
Servo32U4Pin5 servoA;
Servo32U4Pin6 servoB;

// Setup the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

// Define the states
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_SERVO_TEST};
ROBOT_STATE robotState = ROBOT_IDLE;

// A helper function to stop the motors
void idle(void)
{
  Serial.println("idle()");

  //stop motors 
  chassis.idle();

  servoA.detach();
  servoB.detach();

  //set state to idle
  robotState = ROBOT_IDLE;
}

// Function to adjust all servos
void adjustServos(uint16_t uSeconds)
{
  if(robotState == ROBOT_SERVO_TEST)
  {
    servoA.writeMicroseconds(uSeconds);
    servoB.writeMicroseconds(uSeconds);
  }
}

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));

  //ENTER_SAVE idles, regardless of state -- E-stop
  if(keyPress == ENTER_SAVE) idle(); 

  switch(robotState)
  {
    case ROBOT_IDLE:
      if(keyPress == PLAY_PAUSE)
      {
        robotState = ROBOT_SERVO_TEST;
        servoA.attach();
        servoB.attach();
      }
      break;

    case ROBOT_SERVO_TEST:
      if(keyPress == VOLplus)  //VOL+ increases speed
      {
        adjustServos(2000);
      }

      if(keyPress == VOLminus)  //VOL- decreases speed
      {
        adjustServos(1000);
      }

      break;

     default:
      break;
  }
}

/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);

  // initialize the chassis (which also initializes the motors)
  chassis.init();
  idle();

  // these can be undone for the student to adjust
  chassis.setMotorPIDcoeffs(5, 0.5);

  // Setup the servos
  servoA.attach();
  servoB.attach();

  // initialize the IR decoder
  decoder.init();

  Serial.println("/setup()");
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop()
{
  // Check for a key press on the remote
  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);

  // A basic state machine
  switch(robotState)
  {
    default:
      break;
  }
}
