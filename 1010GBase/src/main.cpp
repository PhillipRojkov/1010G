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

double period = 1; //On/off length in seconds
bool flash = true;
void lightThread() {
  while (flash) {
    colourSelector.setLightPower(100);
    wait(period, sec);
    colourSelector.setLightPower(0);
    wait(period, sec);
  }
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  colourSelector.setLightPower(100);
  Brain.Screen.clearScreen(red);

  // Calibrate inertial sensors
  IMUL.startCalibration();
  IMUR.startCalibration();
  encoderL.resetRotation();
  encoderR.resetRotation();
  encoderS.resetRotation();
  wait(1500, msec);
  Brain.Screen.clearScreen(green);
  colourSelector.setLightPower(0);
  wait(500, msec);
  Brain.Screen.clearScreen();

  thread flasher(lightThread);

  // Auto selection
  while (true) {
    if (colourSelector.hue() < 30 && !selecting) {
      selecting = true;
      if (selection < numOfAutos - 1) {
        selection++;
      } else {
        selection = 0;
      }
    } else if (colourSelector.hue() > 30) {
      selecting = false;
    }
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Auto Selection :");
    switch(selection) {
      case 0:
        period = 1;
        Brain.Screen.setCursor(2, 18);
        Brain.Screen.print("Left Two and Middle %d", selection);
        break;
      case 1:
        period = 0.5;
        Brain.Screen.setCursor(2, 18);
        Brain.Screen.print("Home row %d", selection);
        break;
      case 2:
        period = 0.25;
        Brain.Screen.setCursor(2, 18);
        Brain.Screen.print("Left two %d", selection);
        break;
    }
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
  flash = false;
  colourSelector.setLightPower(0);
  Brain.Screen.clearScreen(); // Clear the auto selection text
  // .........................................................................
  thread odomThread(odometryThread); //Start odometry thread

  switch(selection) {
    case 0:
      autoMasters.leftTwoAndMiddle();
      break;
    case 1:
      autoMasters.leftHome();
      break;
    case 2:
      autoMasters.leftTwo();
      break;
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
    if (Controller2.ButtonB.pressing()) {
      IndexerLow.stop(hold);
    }
    if (Controller2.ButtonUp.pressing()) {
      IndexerTop.spin(forward, 20, pct);
    } else if (Controller2.ButtonDown.pressing()) {
      IndexerTop.spin(reverse, 20, pct);
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