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
}

double initialSpeed = 10; //Speed from which a robot accelerates in autonomous functions

void resetDriveEncoders() { //Resets all driver encoder positions to zero
  DriveBL.resetPosition();
  DriveBR.resetPosition();
  DriveFL.resetPosition();
  DriveFR.resetPosition();
}

double avgDriveEncoder() { //Returns average of all driver encoder positions
  return (DriveBL.position(vex::deg) + 
        DriveBR.position(vex::deg) +
        DriveFL.position(vex::deg) +
        DriveBR.position(vex::deg)) /4;
}

double absAvgDriveEncoder() { //Returns average of all drive encoder abs positions
 return (fabs(DriveBL.position(vex::deg)) + 
        fabs(DriveBR.position(vex::deg)) +
        fabs(DriveFL.position(vex::deg)) +
        fabs(DriveBR.position(vex::deg))) /4;
}

void drive(int dir, double speed) { //Drive forward (dir = 1) or backward (dir = -1)
    DriveBL.spin(forward, speed * dir, vex::pct);
    DriveBR.spin(forward, speed * dir, vex::pct);
    DriveFL.spin(forward, speed * dir, vex::pct);
    DriveFR.spin(forward, speed * dir, vex::pct);
}

void turn(int dir, double speed) { //Turn right (dir = 1) or left (dir = -1)
    DriveBL.spin(forward, speed * dir, vex::pct);
    DriveBR.spin(forward, speed * -dir, vex::pct);
    DriveFL.spin(forward, speed * dir, vex::pct);
    DriveFR.spin(forward, speed * -dir, vex::pct);
}

void strafe(int dir, double speed) { //Strafe right (dir = 1) or left (dir = -1)
    DriveBL.spin(forward, speed * -dir, vex::pct);
    DriveBR.spin(forward, speed * dir, vex::pct);
    DriveFL.spin(forward, speed * dir, vex::pct);
    DriveFR.spin(forward, speed * -dir, vex::pct);
}

void brakeDrive() { //Stop the drive using brake mode brake
  DriveBL.stop(vex::brake);
  DriveBR.stop(vex::brake);
  DriveFL.stop(vex::brake);
  DriveFR.stop(vex::brake);
}

void autoForward(double degrees, double iDeg, double fDeg, double speed) { //Forward auto function. degrees > iDeg + fDeg
 resetDriveEncoders();
 
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
  resetDriveEncoders();
  while (avgDriveEncoder() < fDeg) { //Decellerate for the final degrees (fDeg)
    double deccelerate = speed - speed * avgDriveEncoder() / fDeg;

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
 resetDriveEncoders();
 
 while (fabs(avgDriveEncoder()) < iDeg) { //Accelerate for the initial degrees (iDeg)
    double accelerate = speed * absAvgDriveEncoder() / iDeg;

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
  resetDriveEncoders();
  while (fabs(avgDriveEncoder()) < fDeg) { //Decellerate for the final degrees (fDeg)
    double deccelerate = speed - speed * absAvgDriveEncoder() / fDeg;

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

void autoTurnLeft(double degrees, double iDeg, double fDeg, double speed) { //Turn left auto function. degrees > iDeg + fDeg
  resetDriveEncoders();
  
  //accelerate from initialSpeed to speed while turning through iDeg
  while (absAvgDriveEncoder() < iDeg) {
    double accelerate = speed * absAvgDriveEncoder() / iDeg;

    if (accelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      accelerate = initialSpeed;
    }
    turn(-1, accelerate);

    wait(10, msec);
  }
   while (absAvgDriveEncoder() < degrees - fDeg) { //turn until fDeg at speed
    turn(-1, speed);

    wait(10, msec);
  }
  resetDriveEncoders();
  while (absAvgDriveEncoder() < fDeg) { //Decellerate while turning through fDeg
    double deccelerate = speed - speed * absAvgDriveEncoder() / fDeg;

    if (deccelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      deccelerate = initialSpeed;
    }
    turn(-1, deccelerate);

    wait(10, msec);
  }
  
  //stop the drive
  brakeDrive();
}

void autoTurnRight(double degrees, double iDeg, double fDeg, double speed) { //Turn right auto function. degrees > iDeg + fDeg
  resetDriveEncoders();
  
  //accelerate from initialSpeed to speed while turning through iDeg
  while (absAvgDriveEncoder() < iDeg) {
    double accelerate = speed * absAvgDriveEncoder() / iDeg;

    if (accelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      accelerate = initialSpeed;
    }
    turn(1, accelerate);

    wait(10, msec);
  }
   while (absAvgDriveEncoder() < degrees - fDeg) { //turn until fDeg at speed
    turn(1, speed);

    wait(10, msec);
  }
  resetDriveEncoders();
  while (absAvgDriveEncoder() < fDeg) { //Decellerate while turning through fDeg
    double deccelerate = speed - speed * absAvgDriveEncoder() / fDeg;

    if (deccelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      deccelerate = initialSpeed;
    }
    turn(1, deccelerate);

    wait(10, msec);
  }
  
  //stop the drive
  brakeDrive();
}

void autoStrafeLeft (double degrees, double iDeg, double fDeg, double speed) { //Strafe left auto function. degrees > iDeg + fDeg) {
  resetDriveEncoders();
  
  //accelerate from initialSpeed to speed while strafing through iDeg
  while (absAvgDriveEncoder() < iDeg) {
    double accelerate = speed * absAvgDriveEncoder() / iDeg;

    if (accelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      accelerate = initialSpeed;
    }
    strafe(-1, accelerate);

    wait(10, msec);
  }
   while (absAvgDriveEncoder() < degrees - fDeg) { //strafe until fDeg at speed
    strafe(-1, speed);

    wait(10, msec);
  }
  resetDriveEncoders();
  while (absAvgDriveEncoder() < fDeg) { //Decellerate while strafing through fDeg
    double deccelerate = speed - speed * absAvgDriveEncoder() / fDeg;

    if (deccelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      deccelerate = initialSpeed;
    }
    strafe(-1, deccelerate);

    wait(10, msec);
  }
  
  //stop the drive
  brakeDrive();
}

void autoStrafeRight (double degrees, double iDeg, double fDeg, double speed) { //Strafe right auto function. degrees > iDeg + fDeg) {
 resetDriveEncoders();
  
  //accelerate from initialSpeed to speed while strafing through iDeg
  while (absAvgDriveEncoder() < iDeg) {
    double accelerate = speed * absAvgDriveEncoder() / iDeg;

    if (accelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      accelerate = initialSpeed;
    }
    strafe(1, accelerate);

    wait(10, msec);
  }
   while (absAvgDriveEncoder() < degrees - fDeg) { //strafe until fDeg at speed
    strafe(1, speed);

    wait(10, msec);
  }
  resetDriveEncoders();
  while (absAvgDriveEncoder() < fDeg) { //Decellerate while strafing through fDeg
    double deccelerate = speed - speed * absAvgDriveEncoder() / fDeg;

    if (deccelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      deccelerate = initialSpeed;
    }
    strafe(1, deccelerate);

    wait(10, msec);
  }
  
  //stop the drive
  brakeDrive();
}

void intake(double volts) {
  IntakeL.spin(forward, volts, vex::volt);
  IntakeR.spin(forward, volts, vex::volt);
}

void outake(double volts) {
  IntakeL.spin(reverse, volts, vex::volt);
  IntakeR.spin(reverse, volts, vex::volt);
}

void intakeBrake() {
  IntakeL.stop(hold);
  IntakeR.stop(hold);
}

void index(double volts) {
  IndexerL.spin(forward, volts, vex::volt);
  IndexerR.spin(forward, volts, vex::volt);
}

void outdex(double volts) {
  IndexerL.spin(reverse, volts, vex::volt);
  IndexerR.spin(reverse, volts, vex::volt);
}

void indexerBrake() {
  IndexerL.stop(hold);
  IndexerR.stop(hold);
}

//Autonomous master functions
void Calibrate () { //Runs every single action
  autoForward(720, 180, 260, 100);

  autoTurnLeft(360, 90, 90, 100);

  autoTurnRight(360, 90, 90, 100);

  autoStrafeLeft(720, 180, 180, 100);

  autoStrafeRight(720, 180, 180, 100);

  autoBackward(720, 180, 180, 100);

  intake(100);
  wait(1000, msec);

  outake(100);
  wait(1000, msec);

  intakeBrake();
  wait(1000, msec);

  index(127);
  wait(1000, msec);

  outdex(127);
  wait(1000, msec);

  indexerBrake();
}

void SkillsAuto() { //Start red, left of middle
  outake(100);

  autoForward(1000, 180, 180, 100);

  intakeBrake();

  autoStrafeRight(360, 90, 90, 80);

  autoForward(500, 180, 1, 100);
  
  autoBackward(360, 90, 90, 100);

  autoForward(360, 90, 1, 100);
    
  autoBackward(360, 90, 90, 100);

  autoForward(360, 90, 1, 100);

  autoBackward(360, 90, 90, 100);
}

void RedLeftCorner() {

}

void BlueLeftCorner() {

}

void autonomous(void) {
  // ..........................................................................
  Calibrate();
  // ..........................................................................
}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // ........................................................................
    //Simple linear mecanum drive
    /*DriveFL.spin(forward, Controller1.Axis2.value() + Controller1.Axis1.value() + Controller1.Axis4.value(), vex::pct);
    DriveFR.spin(forward, Controller1.Axis2.value() - Controller1.Axis1.value() - Controller1.Axis4.value(), vex::pct);
    DriveBL.spin(forward, Controller1.Axis2.value() + Controller1.Axis1.value() - Controller1.Axis4.value(), vex::pct);
    DriveBR.spin(forward, Controller1.Axis2.value() - Controller1.Axis1.value() + Controller1.Axis4.value(), vex::pct); */

  if (abs(Controller1.Axis3.value()) > 10) { 
    DriveFL.spin(vex::directionType::fwd, Controller1.Axis3.value(),vex::velocityUnits::pct);
    DriveBL.spin(vex::directionType::fwd, Controller1.Axis3.value(),vex::velocityUnits::pct);
  } else {
    DriveFL.stop(coast);
    DriveBL.stop(coast);
  }
  if (abs(Controller1.Axis2.value()) > 10) {
    DriveBR.spin(vex::directionType::fwd, Controller1.Axis2.value(),vex::velocityUnits::pct);
    DriveFR.spin(vex::directionType::fwd, Controller1.Axis2.value(),vex::velocityUnits::pct);
  } else {
    DriveFR.stop(coast);
    DriveBR.stop(coast);
  }

  if(Controller1.ButtonL1.pressing()) {
    DriveBL.spin(directionType::fwd, 100, velocityUnits::pct);
    DriveFL.spin(directionType::rev, 100, velocityUnits::pct);
    DriveBR.spin(directionType::rev, 100, velocityUnits::pct);
    DriveFR.spin(directionType::fwd, 100, velocityUnits::pct);
  }

  if(Controller1.ButtonR1.pressing()) {
    DriveBL.spin(directionType::rev, 100, velocityUnits::pct);
    DriveFL.spin(directionType::fwd, 100, velocityUnits::pct);
    DriveBR.spin(directionType::fwd, 100, velocityUnits::pct);
    DriveFR.spin(directionType::rev, 100, velocityUnits::pct);
  }

    //Simple intake on top right bumper
    if (Controller2.ButtonR1.pressing()) {
      IntakeL.spin(forward, 127, vex::volt);
      IntakeR.spin(forward, 127, vex::volt);
    } else if(Controller2.ButtonR2.pressing()) { //Simple outtake on bottom right bumper
      IntakeL.spin(reverse, 127, vex::volt);
      IntakeR.spin(reverse, 127, vex::volt);
    } else if(Controller2.ButtonUp.pressing()) { //Simple outtake on bottom right bumper
      IntakeL.spin(forward, 127, vex::volt);
      IntakeR.spin(forward, 127, vex::volt);
    } else if(Controller2.ButtonDown.pressing()) { //Simple outtake on bottom right bumper
      IntakeL.spin(reverse, 127, vex::volt);
      IntakeR.spin(reverse, 127, vex::volt);
    } else {
      IntakeL.stop(vex::hold);
      IntakeR.stop(vex::hold);
    }
    
    //Simple indexer up on top left bumper
    if (Controller2.ButtonL1.pressing()) {
      IndexerL.spin(forward, 127, vex::volt);
      IndexerR.spin(forward, 127, vex::volt);
    } else if(Controller2.ButtonL2.pressing()) { //Simple indexer down on bottom left bumper
      IndexerL.spin(reverse, 127, vex::volt);
      IndexerR.spin(reverse, 127, vex::volt);
    } else if(Controller2.ButtonUp.pressing()) { //Simple indexer down on bottom left bumper
      IndexerL.spin(forward, 127, vex::volt);
      IndexerR.spin(forward, 127, vex::volt);
    } else if(Controller2.ButtonDown.pressing()) { //Simple indexer down on bottom left bumper
      IndexerL.spin(reverse, 127, vex::volt);
      IndexerR.spin(reverse, 127, vex::volt);
    } else {
      IndexerL.stop(vex::hold);
      IndexerR.stop(vex::hold);
    }
    // ........................................................................
    wait(10, msec); // Sleep the task for a short amount of time to prevent wasted resources.
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
