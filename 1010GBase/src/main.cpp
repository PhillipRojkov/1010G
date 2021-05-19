/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "AutoMasters.h"
#include "DriveClass.h"
#include "vex.h" 

using namespace vex;

// A global instance of competition
competition Competition;

// Create defenitions of classes
DriveClass driveClass;
AutoMasters autoMasters;
int selection = 0;
/*
 * 0 : Home row, right starting position
 * 1 : Two + middle, right starting position
 * 2 : Two + right side, right starting position
 * 3 : Skills auto
 */
int numOfAutos = 4;
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
/*
  // Calibrate inertial sensors
  IMUL.startCalibration();
  IMUR.startCalibration();
  encoderL.resetPosition();
  encoderR.resetPosition();
  encoderS.resetRotation();*/
  wait(1500, msec);
  Brain.Screen.clearScreen(green);
  wait(500, msec);
  Brain.Screen.clearScreen();

  // Auto selection
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
    wait(20, msec);
  }
}

void odometryThread(){
  while(true) {
  autoMasters.runOdometry();
  wait(4, msec);
  }
}

void autonomous(void) {
  Brain.Screen.clearScreen(); // Clear the auto selection text
  // .........................................................................
  thread odomThread(odometryThread); //Start odometry thread

  autoMasters.LRTAuto();

  if (selection == 0) {
    autoMasters.leftTwoAndMiddle();
    //autoMasters.skills();
  } else if (selection == 1) {
    autoMasters.leftHome();
  } else if (selection == 2) {
    autoMasters.rightTwoAndSide();
  } else if (selection == 3) {
    autoMasters.rightHome();
  }
  // ..........................................................................
}

void usercontrol(void) {
/*
  if (selection == 0) { //Run Flipout for driver skills
    autoMasters.runFlipout();
    wait(300, msec);
  }*/

  // User control code here, inside the loop
  while (1) {
    // ........................................................................
    driveClass.runTankBase();
    driveClass.indexSense();
    driveClass.index();
    driveClass.newIntake();

    if (Controller2.ButtonA.pressing()) {
      driveClass.resetScoreNum();
      driveClass.enableIndex = true;
    }
    driveClass.score();
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