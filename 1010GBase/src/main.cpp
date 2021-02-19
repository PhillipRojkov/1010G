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
// Controller1           Controller   Primary
// Controller2           Controller   Secondary
// IMU                   inertial     15
// DriveFL              Motor         20
// DriveFR              Motor         8
// DriveBL              Motor         19
// DriveBR              Motor         9
// IntakeL              Motor         12
// IntakeR              Motor         2
// IndexerL             Motor         11
// IndexerR             Motor         1
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "AutoMasters.cpp"
#include "DriveClass.h"
#include "Odometry.cpp"
#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// Create defenitions of classes
DriveClass driveClass;
AutoMasters autoMasters;
Odometry odometry;

//Auto selector integer
int selection = 0;
/*
* 0 : Home row, right starting position
* 1 : Two + middle, right starting position
* 2 : Two + right side, right starting position
*/
int numOfAutos = 3;
bool selecting = false;

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

  odometry.calibrating = false;

  //Auto selection
  while (true) {
    if (selector.pressing() && !selecting) {
      selecting = true;
      if (selection < numOfAutos - 1) {
        selection++;
      } else {
        selection = 0;
      }
    } else if (!selector.pressing()) {
      selecting = false;
    }
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Auto Selection :");
    Brain.Screen.setCursor(2, 18);
    Brain.Screen.print(selection);
    wait (20, msec);
  }
}

void autonomous(void) {  
  Brain.Screen.clearScreen(); //Clear the auto selection text
  // .........................................................................
  //autoMasters.newSkillsNew();

  if (selection == 0) {
    autoMasters.rightHome();
  } else if (selection == 1) {
    autoMasters.rightTwoAndMiddle();
  } else if (selection == 2) {
    autoMasters.rightTwoAndSide();
  }
  // ..........................................................................
}

void usercontrol(void) {
  bool indexWait = true;
  double timeToIndex = 2;
  double t = 0;

  // User control code here, inside the loop
  while (1) {
    // ........................................................................
    driveClass.enableIndex = false;

    driveClass.runTankBase();

    driveClass.indexSense();

    driveClass.index();

    driveClass.intake();

    if (Controller2.ButtonA.pressing()) {
     driveClass.resetScoreNum();
     driveClass.enableIndex = true;
    }

    driveClass.score();

    driveClass.checkPosition1();

    //odometry.computeLocation();

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(potL.angle(degrees));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print(potR.angle(degrees));


    if (driveClass.enableIndex) {
      indexWait = false;
    }

    //If enableIndex is false for less than three seconds in a row
    //set enableIndex = true
    //Start counting up
    if (!driveClass.enableIndex && t <= Brain.timer(sec) && !indexWait) {
      t = timeToIndex + Brain.timer(sec);
      indexWait = true;
    } else if (t > Brain.timer(sec)){
      driveClass.enableIndex = true;
    }

    driveClass.intake();

    driveClass.cIndex();
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