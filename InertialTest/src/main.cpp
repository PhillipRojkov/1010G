/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "Odometry.h"
#include "vex.h" 

using namespace vex;

// A global instance of competition
competition Competition;

// Create defenitions of classes
Odometry odometry;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Brain.Screen.clearScreen(red);

  // Calibrate inertial sensor
  IMU.startCalibration();
  wait(1500, msec);
  Brain.Screen.clearScreen(green);
  wait(500, msec);
  Brain.Screen.clearScreen();
}

void odometryThread(){
  while(true) {
  odometry.setXY();
  odometry.printCoordinates();
  wait(5, msec);
  }
}

void autonomous(void) {
  Brain.Screen.clearScreen(); // Clear the auto selection text
  // .........................................................................
  thread odomThread(odometryThread);
  // ..........................................................................
}

void usercontrol(void) {
  while (true) {
    // ........................................................................
    wait(10, msec); // Sleep the task for a short amount of time to prevent
                    // wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}