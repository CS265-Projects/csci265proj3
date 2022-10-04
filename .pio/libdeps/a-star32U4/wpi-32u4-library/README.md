# wpi-32u4-library

Version: 3.5.0<br/>
Release date: 2022-03-26<br/>
Forked from [www.pololu.com](https://www.pololu.com/)

## Summary

This is a C++ library for the Arduino IDE that helps access the on-board hardware of the [Romi 32U4 Control Board](https://www.pololu.com/product/3544).

The Romi 32U4 Control Board turns the Romi chassis into a programmable, Arduino-compatible robot.  It has an integrated AVR ATmega32U4 microcontroller, motor drivers, encoders, buzzer, buttons, and an LSM6DS33 accelerometer and gyro.

This library includes code for accessing the LSM6DS33, forked from the separate [LSM6 library](https://github.com/pololu/lsm6-arduino).

## Installing the library (platformio)

Add the following lines to your platformio.ini file:

~~~{.cpp}
lib_deps = 
     Wire
     wpi-32u4-library
~~~

You're done.

## Classes and functions

The main classes and functions provided by the library are listed below:

* Romi32U4ButtonA
* Romi32U4ButtonB
* Romi32U4ButtonC
* Chassis
* Romi32U4Encoders
* Romi32U4Motors
* Rangefinder
* IR Remote
* PIDController
* Servo32U4PinN (where N can be one of a set of pins)

## Component libraries

This library also includes copies of several other Arduino libraries inside it which are used to help implement the classes and functions above.

* [FastGPIO](https://github.com/pololu/fastgpio-arduino)
* [Pushbutton](https://github.com/pololu/pushbutton-arduino)
* [USBPause](https://github.com/pololu/usb-pause-arduino)

You can use these libraries in your sketch automatically without any extra installation steps and without needing to add any extra `#include` lines to your sketch.

You should avoid adding extra `#include` lines such as `#include <Pushbutton.h>` because then the Arduino IDE might try to use the standalone Pushbutton library (if you previously installed it), and it would conflict with the copy of the Pushbutton code included in this library. 

## Documentation

Documentation found at https://wpiroboticsengineering.github.io/wpi-32u4-library/annotated.html

Some library files (LCD, buzzer) have been removed, since they conflict with some of the changes in the background (mostly to timers).

