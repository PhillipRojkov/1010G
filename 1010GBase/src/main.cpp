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
// DriveFL              Motor         20
// DriveFR              Motor         8
// DriveBL              Motor         19
// DriveBR              Motor         9
// IntakeL              Motor         12
// IntakeR              Motor         2
// IndexerL             Motor         11
// IndexerR             Motor         1
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

double initialSpeed = 10; //Speed from which a robot accelerates in autonomous functions

void RedLeftCorner () {

}

void resetDriveEncoder() { //Resets all driver encoder positions to zero
  DriveBL.resetPosition();
  DriveBR.resetPosition();
  DriveFL.resetPosition();
  DriveFR.resetPosition();
}

double avgDriveEncoder() { //Returns average of all driver encoder positions
  return (DriveBL.position(vex::rotationUnits::deg) + 
        DriveBR.position(vex::rotationUnits::deg) +
        DriveFL.position(vex::rotationUnits::deg) +
        DriveBR.position(vex::rotationUnits::deg)) /4;
}

void drive(int dir, double speed) { //Drive forward (dir = 1) or backward (dir = -1)
    DriveBL.spin(forward, speed * dir, vex::velocityUnits::pct);
    DriveBR.spin(forward, speed * dir, vex::velocityUnits::pct);
    DriveFL.spin(forward, speed * dir, vex::velocityUnits::pct);
    DriveFR.spin(forward, speed * dir, vex::velocityUnits::pct);
}

void brakeDrive() { //Stop the drive using brake mode brake
  DriveBL.stop(vex::brake);
  DriveBR.stop(vex::brake);
  DriveFL.stop(vex::brake);
  DriveFR.stop(vex::brake);
}

void autoForward(double degrees, double iDeg, double fDeg, double speed) { //Forward auto function. degrees > iDeg + fDeg
 resetDriveEncoder();
 
 while (avgDriveEncoder() < iDeg) { //Accelerate for the initial degrees (iDeg)
    double accelerate = speed * avgDriveEncoder() / iDeg;

    if (accelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      accelerate = initialSpeed;
    }

    //Run the drive
    drive(1, accelerate);

    wait(10, msec);
  }
  while (avgDriveEncoder() < degrees - fDeg) { //Drive at speed up until you reach final degrees (fDeg) threshold
    //Run the drive
    drive(1, speed);

    wait(10, msec);
  }
  while (avgDriveEncoder() < degrees) { //Decellerate for the final degrees (fDeg)
    double deccelerate = speed - speed * avgDriveEncoder() / degrees;

    if (deccelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      deccelerate = initialSpeed;
    }

    //Run the drive
    drive(1, deccelerate);

    wait(10, msec);
  }

  //Stop the drive
  brakeDrive();
}

void autoBackward(double degrees, double iDeg, double fDeg, double speed) { //Backward auto function. degrees > iDeg + fDeg
 resetDriveEncoder();
 
 while (fabs(avgDriveEncoder()) < iDeg) { //Accelerate for the initial degrees (iDeg)
    double accelerate = speed * avgDriveEncoder() / iDeg;

    if (accelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      accelerate = initialSpeed;
    }

    //Run the drive
    drive(-1, accelerate);

    wait(10, msec);
  }
  while (fabs(avgDriveEncoder()) < degrees - fDeg) { //Drive at speed up until you reach final degrees (fDeg) threshold
    //Run the drive
    drive(-1, speed);

    wait(10, msec);
  }
  while (fabs(avgDriveEncoder()) < degrees) { //Decellerate for the final degrees (fDeg)
    double deccelerate = speed - speed * avgDriveEncoder() / degrees;

    if (deccelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      deccelerate = initialSpeed;
    }

    //Run the drive
    drive(-1, deccelerate);

    wait(10, msec);
  }
  
  //Stop the drive
  brakeDrive();
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.

  autoForward(720, 180, 180, 100);
  autoBackward(720, 180, 180, 100);

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
    DriveFR.spin(forward, Controller1.Axis2.value() - Controller1.Axis1.value() - Controller1.Axis4.value(), vex::velocityUnits::pct);
    DriveBL.spin(forward, Controller1.Axis2.value() + Controller1.Axis1.value() - Controller1.Axis4.value(), vex::velocityUnits::pct);
    DriveBR.spin(forward, Controller1.Axis2.value() - Controller1.Axis1.value() + Controller1.Axis4.value(), vex::velocityUnits::pct);

    //Simple intake on top right bumper
    if (Controller1.ButtonR1.pressing()) {
      IntakeL.spin(forward, 127, vex::voltageUnits::volt);
      IntakeR.spin(forward, 127, vex::voltageUnits::volt);
    } else if(Controller1.ButtonR2.pressing()) { //Simple outtake on bottom right bumper
      IntakeL.spin(reverse, 127, vex::voltageUnits::volt);
      IntakeR.spin(reverse, 127, vex::voltageUnits::volt);
    } else {
      IntakeL.stop(vex::brakeType::hold);
      IntakeR.stop(vex::brakeType::hold);
    }
    
    //TODO indexer speed set to low value for testing so we don't break axles
    //Simple indexer up on top left bumper
    if (Controller1.ButtonL1.pressing()) {
      IndexerL.spin(forward, 127, vex::voltageUnits::volt);
      IndexerR.spin(forward, 127, vex::voltageUnits::volt);
    } else if(Controller1.ButtonL2.pressing()) { //Simple indexer down on bottom left bumper
      IndexerL.spin(reverse, 127, vex::voltageUnits::volt);
      IndexerR.spin(reverse, 127, vex::voltageUnits::volt);
    } else {
      IndexerL.stop(vex::brakeType::hold);
      IndexerR.stop(vex::brakeType::hold);
    }

    // ........................................................................

    wait(10, msec); // Sleep the task for a short amount of time to
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
