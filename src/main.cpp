#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include <Rangefinder.h>

#include <Chassis.h>

#define LED_PIN 13

//states for main Sense-Think-Act logic
#define STATE_BEGIN 1
#define STATE_SENSE 2
#define STATE_THINK 3
#define STATE_ACT 4
#define STATE_END 5

#define CM_TO_INCHES 0.393701

#define MAX_REPS 4

#define WHEEL_DIAMETER 7
#define ENCODER_COUNTS_PER_REV  1440
#define DIST_BETWEEN_WHEELS 14.9

#define MOTOR_K_P 5
#define MOTOR_K_I 0.5

#define SIDE_LENGTH  10
#define TURN_ANGLE  90
#define DRIVE_SPEED 4
#define TURN_SPEED 15

#define IR_DETECTOR_PIN 1

#define ECHO_PIN 11
#define TRIGGER_PIN 4

#define DARK_THRESHOLD 500


#define WANDER_ANGLE 360
#define WANDER_TURN_RATE 15
#define WANDER_SPEED 7
#define WANDER_DIST 10

#define WALL_FOLLOW_TURNANGLE 180
#define WALL_FOLLOW_TURNRATE 15

#define MASK_WALLFOLLOW  0x04 
#define MASK_WANDER      0x02
#define MASK_APPROACH    0x01

#define WALL_FOLLOW_ARCH 1
#define WALL_FOLLOW_TURN 2

#define K_P  4.7
#define K_I  0.01
#define K_D  2.3

#define NUM_ITERATIONS 50000

/*****************************************************/
/*  Glboal Variables                                 */
/*                                                   */
/*  Declare here variables and data structures that  */
/*  remain in memory                                 */
/*****************************************************/

int state= STATE_BEGIN;

int wallFollowState= WALL_FOLLOW_ARCH;

//IR Decoder keypress
int keyPress= 0;

uint8_t behaviorState= 0;


float distance= 0.0;
float inches= 0.0;

float ref= 6;
float meas= 0.0;
float eT= 0.0;
float eT_deltaT= 0.0;
float ddt_eT= 0.0;
float iEt= 0.0;
float actuation= 0.0;


bool wanderActivated= false;
bool wallFollowActivated= false;

int testIteration= 0;

//TODO:   global vars for IR decoder, RangeFinder,
//        IR reflectance, and Chassis
Chassis chassis(WHEEL_DIAMETER, ENCODER_COUNTS_PER_REV, DIST_BETWEEN_WHEELS);


Rangefinder rangefinder(ECHO_PIN, TRIGGER_PIN);


/****************************************************/
/*           Behavior and helper routines           */
/****************************************************/

//TODO:   add behavior routines to print values
//        of IR Reflectance, IR decoder (remote button press),
//        and rangefinder

//TODO:   add behavior routines to drive straight some distance,
//        and turn some angle to drive in rectangle 


/******
 * Helper routine for debugging hardware
 * while robot running 
 */
void setLED(bool value)
{
  Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}


/*****
 *  Stop the motors w/o locking them
 */
void idle() {
   Serial.println("idle()");
   setLED(HIGH);
 
   //to stop the motors w/o locking wheels
   chassis.idle();
}




/*****
 *  wallFollow
 * 
 *  Moves in an arch and turns around
 * 
 */
void wallFollowController () {
   while(true) {
    
    /***
     * only turn on if the approach controller
     * is not activated
     */
     if (behaviorState & MASK_WALLFOLLOW)  {
       Serial.println("wallFollowController:  activated");
       chassis.moveArch(10,25, true);
       chassis.turnFor(WALL_FOLLOW_TURNANGLE, WALL_FOLLOW_TURNRATE, true);
       behaviorState &= (~MASK_WALLFOLLOW);
     } else {
       break;
     }
   }
}



/*****
 *   wanderController
 * 
 *   Randomly select a direction to pursue, rotate to that heading, a
 *   and advaced forward in that direction
 */
void wanderController() {
  float turnAngle= rand() % WANDER_ANGLE;

  /****
   *  activate wander only if approach controller
   *  and wall follow controllers are not active
   */
  if ( behaviorState & MASK_WANDER) {
    Serial.println("wanderController:  activated");

    Serial.print("wanderController:  selected turnAngle= ");
    Serial.println(turnAngle);

    chassis.turnFor(turnAngle, WANDER_TURN_RATE, true);
    chassis.driveFor(WANDER_DIST, WANDER_SPEED, true);
    behaviorState &= (~MASK_WANDER);
  } 
}




/*****
 *   PID Control Law for approch controller
 * 
 */
float controlLaw(float Kp, float Ki, float Kd) {
   float result= Kp* eT_deltaT +  Ki* iEt + Kd* ddt_eT;

   return result;
}

/*****
 *   approachController
 * 
 *   Approach obstacle maintain distance actively
 */
void approachController() {
  eT= eT_deltaT;
  eT_deltaT= ref - meas;

  ddt_eT= eT_deltaT - eT;


  /***
   * run if activated
   */
  if ( (behaviorState & MASK_APPROACH) && (fabs(eT_deltaT) > 0.5) ) {
    Serial.println("approachController:  activated");
    /****
     *  If you are far away from the reference
     *  this means you are far away from the goal.
     *  In this case, turn off the Integral component
     *  as it is most useful for steady state error. 
     */
    if (fabs(actuation) > 10) {
      iEt= 0;
    } else {
      iEt+= eT_deltaT;
    }

    Serial.print("approachController:  meas= ");
    Serial.print(meas);
    Serial.print("  eT_deltaT= ");  
    Serial.print(eT_deltaT);
    Serial.print("  ddt_eT= ");
    Serial.print(ddt_eT);
    Serial.print("  iEt= ");
    Serial.println(iEt);

    /***
     *  Note:  When robot too close, error is negative
     *         and must drive forward.  When robot is
     *         too far, error is positive and must
     *         drive backwards.  So we negate actuation
     *         signal and use as speed.
     */

    actuation = controlLaw(K_P, K_I, K_D);

    Serial.print("approachController:  actuation= ");
    Serial.println(actuation);

    /***
     *  If error ( eT_deltaT) is mall and magnitude of actuation is very small, just
     *  call it zero.
     */
    if ( (fabs(actuation) < 0.2) || (fabs(eT_deltaT) < 0.1) ) { 
      actuation= 0.0;
      eT_deltaT= 0.0;
      eT= 0.0;
      chassis.idle();
    } else {

      /*****
       * Note:  In chassis.driveFor(), the way to get it
       *        to move backwards is by using a negative
       *        distance.  It is NOT by using negative
       *        speed which would be more intuitive.
       */
      if (actuation < 0) {
        chassis.driveFor(1, actuation, false);
      } else {
        chassis.driveFor(-1, actuation, false);
      }
    }
  } 
}



/****************************************************/
/*  Sketch Entrypoints here                         */
/****************************************************/

/*****
 *   Power cycle one-shot setup code
 *   Setup board configuration and object/system initializaitons
 */

void setup() {
  // put your setup code here, to run once:

  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);
  //while(!Serial) {
  //  ;
  //}

  chassis.init();
  chassis.setMotorPIDcoeffs(MOTOR_K_P, MOTOR_K_I);



  rangefinder.init();

  pinMode(LEFT_LINE_SENSE, INPUT);
  pinMode(RIGHT_LINE_SENSE, INPUT);

  state= STATE_BEGIN;

  chassis.driveFor(1,1,true);
  idle();
}


/******
 * Main entrypoint Behavior code
 */

void loop() {
  // put your main code here, to run repeatedly:

  switch(state)  {
    case STATE_BEGIN:
      Serial.println("STATE_BEGIN");

      behaviorState= 0;
      testIteration= 0;

     
      //Note:  ref is fixed in globals
      // Init the control signals
      meas= 0.0;
      eT= 0.0;
      eT_deltaT= 0.0;
      ddt_eT= 0.0;
      iEt= 0.0;

      wallFollowState= WALL_FOLLOW_ARCH;

      state= STATE_SENSE;
      break;

    case STATE_SENSE:
      Serial.println();
      Serial.println("STATE_SENSE");


      //ultrasonic rangefinder
      distance= rangefinder.getDistance();


      state= STATE_THINK;
      break;

    case STATE_THINK:
      Serial.println("STATE_THINK");
      //delay(100);

      inches= distance * CM_TO_INCHES;
      Serial.print("rangefinder dist= ");
      Serial.println(inches);
      meas= inches;

      state= STATE_ACT;
      break;

    case STATE_ACT:
      Serial.println("STATE_ACT");



      if (chassis.checkMotionComplete()) {
        Serial.println("checkMotion IS complete");

        
        /****
         *  Test here turning on each
         *  basis behavior
         */
        behaviorState|= MASK_APPROACH; 
        approachController();
        behaviorState &= (~MASK_APPROACH);

        //behaviorState|= MASK_WALLFOLLOW;
        //wallFollowController();
        //behaviorState&= (~MASK_WALLFOLLOW);

        //behaviorState|= MASK_WANDER;
        //wanderController();
        //behaviorState&= MASK_WANDER;

      } else {
        delay(5);
        Serial.println("checkMotion NOT complete");
      }
     
 
      testIteration= testIteration + 1;
      Serial.print("testIteration= ");  
      Serial.println(testIteration);

      if (testIteration < NUM_ITERATIONS) {
        state = STATE_SENSE;
      } else {
        state= STATE_END;
      }
      break;   

    case STATE_END:
      chassis.idle();
      break;
  }
}

