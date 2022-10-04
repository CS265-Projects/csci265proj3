#pragma once

#include <Arduino.h>
#include <Romi32U4Motors.h>

/** \class Chassis
 * The Chassis class manages the motors and encoders.
 * 
 * Chassis sets up a hardware-based timer on a 16ms interval. At each interrupt, it
 * reads the current encoder counts and, if controlling for speed, calculates the 
 * effort using a PID controller for each motor, which can be adjusted by the user.
 * 
 * The encoders are attached automatically and the encoders will count regardless of 
 * the state of the robot.
 * 
 * Several methods are provided for low level control to commands for driving or turning. 
 * */
class Chassis
{
public:
    LeftMotor leftMotor;
    RightMotor rightMotor;

protected:
    const float cmPerEncoderTick;
    const float robotRadius;
    const uint16_t ctrlIntervalMS = 16;

public:
    /** \brief Chassis constructor.
     * 
     * @param wheelDiam Wheel diameter in cm.
     * @param ticksPerRevolution Enccoder ticks per _wheel_ revolution.
     * @param wheelTrack Distance between wheels in cm.
     * */
    Chassis(float wheelDiam = 7, float ticksPerRevolution = 1440, float wheelTrack = 14.7) 
        : cmPerEncoderTick(wheelDiam * M_PI / ticksPerRevolution), robotRadius(wheelTrack / 2.0)
        {}
    
    //Mods

    /** \brief Commands the chassis to drive in arch for set angle
    * @param xDot linear velocity
    * @param thetaDot angular velocity
    * @param block is it blocking call
    */
    void moveArch(float xDot, float thetaDot, bool block);

    //END Mods

    /** \brief Initializes the chassis. Must be called in setup().
     * */
    void init(void);
    
    /** \brief Sets PID coefficients for the motors. Not independent.
     * */
    void setMotorPIDcoeffs(float kp, float ki);

    /** \brief Idles chassis. Motors will stop.
     * */
    void idle(void);

    /** \brief Sets motor efforts. Max speed is 420.
     * 
     * @param leftEffort Effort for left motor
     * @param rightEffort Effort for right motor
     * */
    void setMotorEfforts(int leftEffort, int rightEffort);

    /** \brief Sets target wheel speeds in cm/sec.
     * 
     * @param leftSpeed Target speed for left wheel in cm/sec
     * @param rightSpeed Target speed for right wheel in cm/sec
     * */
    void setWheelSpeeds(float leftSpeed, float rightSpeed);

    /** \brief Sets target motion for the chassis.
     * 
     * @param forwardSpeed Target forward speed in cm/sec
     * @param rightSpeed Target spin rate in deg/sec
     * */
    void setTwist(float forwardSpeed, float turningSpeed);

    /** \brief Commands the robot to drive at a distance and speed.
     * 
     * The chassis will stop when the distance is reached.
     * 
     * @param forwardDistance Target distance in cm
     * @param forwardSpeed Target speed rate in cm/sec
     * @param block If true, the function blocks until the motion is complete
     * */
    void driveFor(float forwardDistance, float forwardSpeed, bool block = false);
    
    /** \brief Commands the chassis to turn a set angle.
     * 
     * @param turnAngle Target angle to turn in degrees
     * @param turningSpeed Target spin rate in deg/sec
     * @param block If true, the function blocks until the motion is complete
     * */
    void turnFor(float turnAngle, float turningSpeed, bool block = false);

    /** \brief Checks if the motion commanded by driveFor() or turnFor() is done.
     * 
     * \return Returns true if the motion is complete.
     * */
    bool checkMotionComplete(void);

    void printSpeeds(void);
    void printEncoderCounts(void);

    inline void updateEncoderDeltas();
};

extern Chassis chassis;
