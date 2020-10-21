/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1           Controller
// DriveFL              Motor         1
// DriveFR              Motor         2
// DriveBL              Motor         3
// DriveBR              Motor         4
// IntakeL              Motor         5
// IntakeR              Motor         6
// IndexerL             Motor         7
// IndexerR             Motor         8
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

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

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.

    //Simple linear mecanum drive
    DriveFL.spin(forward, Controller1.Axis2.value() + Controller1.Axis1.value() + Controller1.Axis4.value(), vex::velocityUnits::pct);
    DriveFR.spin(forward, Controller1.Axis2.value() - Controller1.Axis1.value() + Controller1.Axis4.value(), vex::velocityUnits::pct);
    DriveBL.spin(forward, Controller1.Axis2.value() + Controller1.Axis1.value() - Controller1.Axis4.value(), vex::velocityUnits::pct);
    DriveBR.spin(forward, Controller1.Axis2.value() - Controller1.Axis1.value() - Controller1.Axis4.value(), vex::velocityUnits::pct);

    //Simple intake on top right bumper
    if (Controller1.ButtonR1.pressing()) {
      IntakeL.spin(forward, 100, vex::velocityUnits::pct);
      IntakeR.spin(forward, 100, vex::velocityUnits::pct);
    } else if(Controller1.ButtonR1.pressing()) { //Simple outtake on bottom right bumper
      IntakeL.spin(reverse, 100, vex::velocityUnits::pct);
      IntakeR.spin(reverse, 100, vex::velocityUnits::pct);  
    } else {
      IntakeL.stop(vex::brakeType::hold);
      IntakeR.stop(vex::brakeType::hold);
    }
    
    //TODO indexer speed set to low value for testing so we don't break axles
    //Simple indexer up on top left bumper
    if (Controller1.ButtonL1.pressing()) {
      IndexerL.spin(forward, 1, vex::velocityUnits::pct);
      IndexerR.spin(forward, 1, vex::velocityUnits::pct);
    } else if(Controller1.ButtonL1.pressing()) { //Simple indexer down on bottom left bumper
      IndexerL.spin(reverse, 1, vex::velocityUnits::pct);
      IndexerR.spin(reverse, 1, vex::velocityUnits::pct);  
    } else {
      IndexerL.stop(vex::brakeType::hold);
      IndexerR.stop(vex::brakeType::hold);
    }

    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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
