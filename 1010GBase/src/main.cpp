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
  
  Brain.Screen.clearScreen(red);

  //Calibrate inertial sensor
  IMU.startCalibration();
  wait(1500, msec);
  Brain.Screen.clearScreen(green);
  wait(500, msec);
  Brain.Screen.clearScreen();
}

double initialSpeed = 10; //Speed from which a robot accelerates in autonomous functions

int turnMargin = 400; //Time in msec for which turn needs to be at the correct angle
double turnRange = 0.1; //Range in which the turn needs to be in order to stop method

int loopTime = 10; //loop pause in msec
long double msecLoopTime = loopTime / 1000.0; //loop pause in seconds

long double g = 9.80665; //graviational constant

//PID constants
double drivekP = 0.8;
double drivekD = 0.5;
double drivekI = 0.002;

double turnkP = 1.4;
double turnkD = 2.2;
double turnkI = 0.00002;

double strafekP = 5;
double strafekD = loopTime;
double strafekI = 0.001;

//Location Variables
double h = 0; //Heading in degrees. Rotation clockwise is positive, does not reset at 360
long double x = 0; //x position on the field (side to side from starting position) in meters
long double y = 0; //y position on the field (forwards/backwards from starting position) in meters

//Variables used for calculating location
long double viX = 0;
long double viY = 0;

//Variables used for calculating PID
double error; //SensorValue - DesiredValue : Position
double prevError = 0; //Position loopTime msec ago
double derivative; //error - prevError : speed
double totalError = 0; //+= error

//Sets X and Y equal to the global position coordinates
void computeLocation() {
  long double accelX = -IMU.acceleration(yaxis); //Right is positive
  long double accelY = -IMU.acceleration(zaxis); //Forward is positive

  //Cancel out vertical component from potential tipping

  //Cancel noise
  if (fabsl(accelX) < 0.04) {
    accelX = 0;
  }
  if (fabsl(accelY) < 0.04) {
    accelY = 0;
  }

  //Convert from local XY to global XY
  accelX = accelX * cos(IMU.rotation()) + accelY * sin(IMU.rotation()) + accelX * -sin(IMU.pitch()) + accelY * -sin(IMU.roll());
  accelY = accelY * cos(IMU.rotation()) + accelX * sin(IMU.rotation()) + accelX * -sin(IMU.pitch()) + accelY * -sin(IMU.roll());

  //convert from Gs to m/s2
  accelX *= g;
  accelY *= g;

  //Use Kinematics Equation to conver to distance
  x += viX * msecLoopTime + 0.5 * accelX * powl(msecLoopTime, 2);
  y += viY * msecLoopTime + 0.5 * accelY * powl(msecLoopTime, 2);

  //Create initial VX and VY for the next run (in 10 msec)
  viX += accelX * msecLoopTime;
  viY += accelY * msecLoopTime;

  //Cancel noise on VX & VY
  if (fabsl(viX) < 0.05 && fabsl(accelX) < 0.04 * g) {
    viX = 0;
  }
  if (fabsl(viY) < 0.05 && fabsl(accelY) < 0.04 * g) {
    viY = 0;
  }
}

void screenPrint(){
  //Debug
  //Print X and Y field position
  Brain.Screen.setCursor(1, 2);
  Brain.Screen.print(x);
  Brain.Screen.setCursor(1, 18);
  Brain.Screen.print(y);
  
  //Print IMU rotation
  Brain.Screen.setCursor(4, 2);
  Brain.Screen.print(IMU.rotation());

  //Print drive rpms
  Brain.Screen.setCursor(6, 2);
  Brain.Screen.print(DriveFL.velocity(rpm));
  Brain.Screen.setCursor(7, 2);
  Brain.Screen.print(DriveFR.velocity(rpm));
  Brain.Screen.setCursor(8, 2);
  Brain.Screen.print(DriveBL.velocity(rpm));
  Brain.Screen.setCursor(9, 2);
  Brain.Screen.print(DriveBR.velocity(rpm));
}

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

void drive(int dir, double speed) { //Drive forward (dir = 1) or backward (dir = -1). Called every loopTime msec
    //PID
    //Used to make robot go straight
    error = IMU.rotation() - h;
    derivative = error - prevError;
    totalError += error;
    prevError = error;
    
    DriveBL.spin(forward, speed * dir - error * drivekP - totalError * drivekI - derivative * drivekD, vex::pct);
    DriveBR.spin(forward, speed * dir + error * drivekP + totalError * drivekI + derivative * drivekD, vex::pct);
    DriveFL.spin(forward, speed * dir - error * drivekP - totalError * drivekI - derivative * drivekD, vex::pct);
    DriveFR.spin(forward, speed * dir + error * drivekP + totalError * drivekI + derivative * drivekD, vex::pct);
}

void strafe(int dir, double speed) { //Strafe right (dir = 1) or left (dir = -1)
    //PID
    //Used to make robot go straight
    error = IMU.rotation() - h;
    derivative = error - prevError;
    totalError += error;
    prevError = error;
    
    DriveBL.spin(forward, speed * -dir - error * strafekP - totalError * strafekI - derivative * strafekD, vex::pct);
    DriveBR.spin(forward, speed * dir + error * strafekP + totalError * strafekI + derivative * strafekD, vex::pct);
    DriveFL.spin(forward, speed * dir - error * strafekP - totalError * strafekI - derivative * strafekD, vex::pct);
    DriveFR.spin(forward, speed * -dir + error * strafekP + totalError * strafekI + derivative * strafekD, vex::pct);
}

void brakeDrive() { //Stop the drive using brake mode brake
  DriveBL.stop(vex::brake);
  DriveBR.stop(vex::brake);
  DriveFL.stop(vex::brake);
  DriveFR.stop(vex::brake);
}

void autoForward(double degrees, double iDeg, double fDeg, double speed) { //Forward auto function. degrees > iDeg + fDeg
 resetDriveEncoders();

 h = IMU.rotation();

 while (avgDriveEncoder() < iDeg) { //Accelerate for the initial degrees (iDeg)
    double accelerate = speed * avgDriveEncoder() / iDeg;

    if (accelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      accelerate = initialSpeed;
    }

    //Run the drive
    drive(1, accelerate);

    wait(loopTime, msec);
  }
  while (avgDriveEncoder() < degrees - fDeg) { //Drive at speed up until you reach final degrees (fDeg) threshold
    //Run the drive
    drive(1, speed);

    wait(loopTime, msec);
  }
  resetDriveEncoders();
  while (avgDriveEncoder() < fDeg) { //Decellerate for the final degrees (fDeg)
    double deccelerate = speed - speed * avgDriveEncoder() / fDeg;

    if (deccelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      deccelerate = initialSpeed;
    }

    //Run the drive
    drive(1, deccelerate);

    wait(loopTime, msec);
  }

  //Stop the drive
  brakeDrive();
}

void autoBackward(double degrees, double iDeg, double fDeg, double speed) { //Backward auto function. degrees > iDeg + fDeg
 resetDriveEncoders();
 
 h = IMU.rotation();

 while (fabs(avgDriveEncoder()) < iDeg) { //Accelerate for the initial degrees (iDeg)
    double accelerate = speed * absAvgDriveEncoder() / iDeg;

    if (accelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      accelerate = initialSpeed;
    }

    //Run the drive
    drive(-1, accelerate);

    wait(loopTime, msec);
  }
  while (fabs(avgDriveEncoder()) < degrees - fDeg) { //Drive at speed up until you reach final degrees (fDeg) threshold
    //Run the drive
    drive(-1, speed);

    wait(loopTime, msec);
  }
  resetDriveEncoders();
  while (fabs(avgDriveEncoder()) < fDeg) { //Decellerate for the final degrees (fDeg)
    double deccelerate = speed - speed * absAvgDriveEncoder() / fDeg;

    if (deccelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      deccelerate = initialSpeed;
    }

    //Run the drive
    drive(-1, deccelerate);

    wait(loopTime, msec);
  }
  
  //Stop the drive
  brakeDrive();
}

void autoTurnTo(double degrees) { //+degrees turns right, -degrees turns left
  int t = 0; //Time variable

  while(t < turnMargin) { //break when time exceeds the turnMargin
    //PID
    //Used to make robot go straight
    error = IMU.rotation() - degrees;
    derivative = error - prevError;
    totalError += error;
    prevError = error;
    
    DriveBL.spin(forward, -error * turnkP - totalError * turnkI - derivative * turnkD, vex::pct);
    DriveBR.spin(forward, error * turnkP + totalError * turnkI + derivative * turnkD, vex::pct);
    DriveFL.spin(forward, -error * turnkP - totalError * turnkI - derivative * turnkD, vex::pct);
    DriveFR.spin(forward, error * turnkP + totalError * turnkI + derivative * turnkD, vex::pct);

    wait(loopTime, msec);

    if (error < turnRange && error > -turnRange) { //increase time when the robot is pointing in turnRange
      t += loopTime;
    } else {
      t = 0;
    }
  }
  
  //stop the drive
  brakeDrive();
}

void autoStrafeLeft (double degrees, double iDeg, double fDeg, double speed) { //Strafe left auto function. degrees > iDeg + fDeg) {
  resetDriveEncoders();
  
  h = IMU.rotation();

  //accelerate from initialSpeed to speed while strafing through iDeg
  while (absAvgDriveEncoder() < iDeg) {
    double accelerate = speed * absAvgDriveEncoder() / iDeg;

    if (accelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      accelerate = initialSpeed;
    }
    strafe(-1, accelerate);

    wait(loopTime, msec);
  }
   while (absAvgDriveEncoder() < degrees - fDeg) { //strafe until fDeg at speed
    strafe(-1, speed);

    wait(loopTime, msec);
  }
  resetDriveEncoders();
  while (absAvgDriveEncoder() < fDeg) { //Decellerate while strafing through fDeg
    double deccelerate = speed - speed * absAvgDriveEncoder() / fDeg;

    if (deccelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      deccelerate = initialSpeed;
    }
    strafe(-1, deccelerate);

    wait(loopTime, msec);
  }
  
  autoTurnTo(h);

  //stop the drive
  brakeDrive();
}

void autoStrafeRight (double degrees, double iDeg, double fDeg, double speed) { //Strafe right auto function. degrees > iDeg + fDeg) {
  resetDriveEncoders();
  
  h = IMU.rotation();

  //accelerate from initialSpeed to speed while strafing through iDeg
  while (absAvgDriveEncoder() < iDeg) {
    double accelerate = speed * absAvgDriveEncoder() / iDeg;

    if (accelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      accelerate = initialSpeed;
    }
    strafe(1, accelerate);

    wait(loopTime, msec);
  }
   while (absAvgDriveEncoder() < degrees - fDeg) { //strafe until fDeg at speed
    strafe(1, speed);

    wait(loopTime, msec);
  }
  resetDriveEncoders();
  while (absAvgDriveEncoder() < fDeg) { //Decellerate while strafing through fDeg
    double deccelerate = speed - speed * absAvgDriveEncoder() / fDeg;

    if (deccelerate < initialSpeed) { //Make sure that the motors never move slower than initalSpeed
      deccelerate = initialSpeed;
    }
    strafe(1, deccelerate);

    wait(loopTime, msec);
  }
  
  autoTurnTo(h);

  //stop the drive
  brakeDrive();
}

void intake(double speed) {
  IntakeL.spin(forward, speed, vex::pct);
  IntakeR.spin(forward, speed, vex::pct);
}

void outake(double speed) {
  IntakeL.spin(reverse, speed, vex::pct);
  IntakeR.spin(reverse, speed, vex::pct);
}

void intakeBrake() {
  IntakeL.stop(hold);
  IntakeR.stop(hold);
}

void index(double speed) {
  IndexerL.spin(forward, speed, vex::pct);
  IndexerR.spin(forward, speed, vex::pct);
}

void outdex(double speed) {
  IndexerL.spin(reverse, speed, vex::pct);
  IndexerR.spin(reverse, speed, vex::pct);
}

void indexerBrake() {
  IndexerL.stop(hold);
  IndexerR.stop(hold);
}

//Autonomous master functions
void autoCalibrate () { //Runs every single action
  autoForward(720, 180, 360, 100);

  autoTurnTo(-90);

  autoTurnTo(0);

  autoStrafeLeft(720, 180, 180, 100);

  autoStrafeRight(720, 180, 180, 100);

  autoBackward(720, 180, 180, 100);

  intake(100);
  wait(1000, msec);

  outake(100);
  wait(1000, msec);

  intakeBrake();

  index(127);
  wait(1000, msec);

  outdex(127);
  wait(1000, msec);

  indexerBrake();
}

void turnCalibrate () {
  autoTurnTo(-90);
  autoTurnTo(90);
  autoTurnTo(-90);
  autoTurnTo(90);
  autoTurnTo(-90);
  autoTurnTo(90);
  autoTurnTo(-90);
  autoTurnTo(90);
  autoTurnTo(-90);
  autoTurnTo(0);
}

void strafeCalibrate () {

}

void skillsAuto() { //Start red, left of middle
  intake(100);

  autoForward(1000, 180, 180, 100);

  intakeBrake();

  autoStrafeRight(360, 180, 180, 70);

  autoForward(500, 180, 1, 70);
  
  autoBackward(250, 90, 180, 70);

  autoForward(250, 90, 1, 70);
    
  autoBackward(250, 90, 180, 70);

  autoForward(250, 90, 1, 70);

  autoBackward(250, 90, 180, 70);

  autoForward(250, 90, 1, 70);

  autoBackward(250, 90, 180, 70);
  
  autoForward(270, 90, 1, 70);

  autoBackward(270, 90, 180, 70);

  autoForward(270, 90, 1, 70);

  autoBackward(270, 90, 180, 70);

  autoForward(270, 90, 1, 70);
}

void redLeftCorner() {

}

void blueLeftCorner() {

}

void autonomous(void) {
  // ..........................................................................
  pre_auton();

  turnCalibrate();
  // ..........................................................................
}

void usercontrol(void) {
  pre_auton();

  // User control code here, inside the loop
  while (1) {
    // ........................................................................
    //Simple linear mecanum drive
    /*DriveFL.spin(forward, Controller1.Axis2.value() + Controller1.Axis1.value() + Controller1.Axis4.value(), vex::pct);
    DriveFR.spin(forward, Controller1.Axis2.value() - Controller1.Axis1.value() - Controller1.Axis4.value(), vex::pct);
    DriveBL.spin(forward, Controller1.Axis2.value() + Controller1.Axis1.value() - Controller1.Axis4.value(), vex::pct);
    DriveBR.spin(forward, Controller1.Axis2.value() - Controller1.Axis1.value() + Controller1.Axis4.value(), vex::pct); */

  if (abs(Controller1.Axis3.value()) > loopTime) { 
    DriveFL.spin(vex::directionType::fwd, Controller1.Axis3.value(),vex::velocityUnits::pct);
    DriveBL.spin(vex::directionType::fwd, Controller1.Axis3.value(),vex::velocityUnits::pct);
  } else {
    DriveFL.stop(brake);
    DriveBL.stop(brake);
  }
  if (abs(Controller1.Axis2.value()) > loopTime) {
    DriveBR.spin(vex::directionType::fwd, Controller1.Axis2.value(),vex::velocityUnits::pct);
    DriveFR.spin(vex::directionType::fwd, Controller1.Axis2.value(),vex::velocityUnits::pct);
  } else {
    DriveFR.stop(brake);
    DriveBR.stop(brake);
  }

  double multiplier = 0.6;

  if(Controller1.ButtonL1.pressing()) {
    DriveBL.spin(directionType::fwd, 80 + Controller1.Axis3.value() * multiplier, velocityUnits::pct);
    DriveFL.spin(directionType::rev, 80 - Controller1.Axis3.value() * multiplier , velocityUnits::pct);
    DriveBR.spin(directionType::rev, 80 - Controller1.Axis2.value() * multiplier, velocityUnits::pct);
    DriveFR.spin(directionType::fwd, 80  + Controller1.Axis2.value() * multiplier, velocityUnits::pct);
  }

  if(Controller1.ButtonR1.pressing()) {
    DriveBL.spin(directionType::rev, 80 - Controller1.Axis3.value() * multiplier, velocityUnits::pct);
    DriveFL.spin(directionType::fwd, 80 + Controller1.Axis3.value() * multiplier, velocityUnits::pct);
    DriveBR.spin(directionType::fwd, 80 + Controller1.Axis2.value() * multiplier, velocityUnits::pct);
    DriveFR.spin(directionType::rev, 80 - Controller1.Axis2.value() * multiplier, velocityUnits::pct);
  }

    //Simple intake on top right bumper
    if (Controller2.ButtonR1.pressing()) {
      IntakeL.spin(forward, 127, vex::pct);
      IntakeR.spin(forward, 127, vex::pct);
    } else if(Controller2.ButtonR2.pressing()) { //Simple outtake on bottom right bumper
      IntakeL.spin(reverse, 127, vex::pct);
      IntakeR.spin(reverse, 127, vex::pct);
    } else if(Controller2.ButtonUp.pressing()) { //Simple outtake on bottom right bumper
      IntakeL.spin(forward, 127, vex::pct);
      IntakeR.spin(forward, 127, vex::pct);
    } else if(Controller2.ButtonDown.pressing()) { //Simple outtake on bottom right bumper
      IntakeL.spin(reverse, 127, vex::pct);
      IntakeR.spin(reverse, 127, vex::pct);
    } else {
      IntakeL.stop(vex::hold);
      IntakeR.stop(vex::hold);
    }
    
    //Simple indexer up on top left bumper
    if (Controller2.ButtonL1.pressing()) {
      IndexerL.spin(forward, 127, vex::pct);
      IndexerR.spin(forward, 127, vex::pct);
    } else if(Controller2.ButtonL2.pressing()) { //Simple indexer down on bottom left bumper
      IndexerL.spin(reverse, 127, vex::pct);
      IndexerR.spin(reverse, 127, vex::pct);
    } else if(Controller2.ButtonUp.pressing()) { //Simple indexer down on bottom left bumper
      IndexerL.spin(forward, 127, vex::pct);
      IndexerR.spin(forward, 127, vex::pct);
    } else if(Controller2.ButtonDown.pressing()) { //Simple indexer down on bottom left bumper
      IndexerL.spin(reverse, 127, vex::pct);
      IndexerR.spin(reverse, 127, vex::pct);
    } else {
      IndexerL.stop(vex::hold);
      IndexerR.stop(vex::hold);
    }

    computeLocation();

    screenPrint();
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
