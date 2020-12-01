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
#include "DriveClass.cpp"
#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// Create defenitions of classes
DriveClass driveClass;
AutoMasters autoMasters;

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

// Loop times and constants
int loopTime = 10;                       // loop pause in msec
double msecLoopTime = loopTime / 1000.0; // loop pause in seconds
double g = 9.80665;                      // graviational constant
double PI = 3.14159265359;

// Odometery return variables
double h = 0; // Heading in degrees. Rotation clockwise is positive, does not
              // reset at 360
double x = 0; // x position on the field (side to side from starting position)
              // in meters
double y = 0; // y position on the field (forwards/backwards from starting
              // position) in meters

// Variables used for calculating location
double viX = 0;
double viY = 0;

double absAvgDriveRPM() { // Returns average of all drive speeds
  return (fabs(DriveBL.velocity(rpm)) + fabs(DriveBR.velocity(rpm)) +
          fabs(DriveFL.velocity(rpm)) + fabs(DriveFR.velocity(rpm))) /
         4;
}

// Sets X and Y equal to the global position coordinates
void computeLocation() {
  double accelX = IMU.acceleration(yaxis); // SIDE VECTOR - Right is positive
  double accelY =
      IMU.acceleration(xaxis); // FORWARD VECTOR - Forward is positive

  // Cancel out vertical component from potential tipping
  accelX = accelX + sin(IMU.roll() * (PI / 180));
  accelY = accelY - sin(IMU.pitch() * (PI / 180));

  // Cancel noise
  if (fabs(accelX) < 0.06) {
    accelX = 0.0;
  }
  if (fabs(accelY) < 0.06) {
    accelY = 0.0;
  }

  // Create global x and y vectors
  double globalAccelX = accelX * sin(IMU.rotation() * (PI / 180)) +
                        accelY * cos(IMU.rotation() * (PI / 180));
  double globalAccelY = accelX * cos(IMU.rotation() * (PI / 180)) +
                        accelY * sin(IMU.rotation() * (PI / 180));

  // convert from Gs to m/s2
  globalAccelX *= g;
  globalAccelY *= g;

  // Use Kinematics Equation to conver to distance
  x += viX * msecLoopTime + 0.5 * globalAccelX * pow(msecLoopTime, 2);
  y += viY * msecLoopTime + 0.5 * globalAccelY * pow(msecLoopTime, 2);

  // Create initial VX and VY for the next run (in 10 msec)
  viX += globalAccelX * msecLoopTime;
  viY += globalAccelY * msecLoopTime;

  // Cancel noise on VX & VY
  if ((fabs(viX) < 0.4 && fabs(globalAccelX) < 0.06 * g) ||
      (fabs(globalAccelX) < 0.06 * g && absAvgDriveRPM() < 2)) {
    viX = 0.0;
  }
  if ((fabs(viY) < 0.4 && fabs(globalAccelY) < 0.06 * g) ||
      (fabs(globalAccelY) < 0.06 * g && absAvgDriveRPM() < 2)) {
    // Brain.Screen.drawCircle(300, 100, 100, red);
    viY = 0.0;
  } else {
    // Brain.Screen.drawCircle(300, 100, 100, green);
  }

  if (fabs(globalAccelY) < 0.06 * g) {
    Brain.Screen.drawCircle(300, 100, 100, red);
  } else {
    Brain.Screen.drawCircle(300, 100, 100, green);
  }
}

void screenPrint() {
  // Debug
  // Print X and Y field position
  Brain.Screen.setCursor(1, 2);
  Brain.Screen.print(x);
  Brain.Screen.setCursor(1, 18);
  Brain.Screen.print(y);

  // Print IMU rotation
  Brain.Screen.setCursor(4, 2);
  Brain.Screen.print(IMU.rotation());
}

void autonomous(void) {
  // ..........................................................................
  pre_auton();

  autoMasters.skillsAuto();
  // ..........................................................................
}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // ........................................................................
    driveClass.runTankBase();

    driveClass.indexSense();

    driveClass.index();

    driveClass.intake();

    computeLocation();

    screenPrint();
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
