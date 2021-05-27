#pragma once
#include "vex.h"

class AutoFunctions { // Holds basic autonomous functions
public:
  // Indexer parameters
  bool position1;
  bool position2;
  bool position3;

private:
  double h = 0; // Heading in degrees. Rotation clockwise is positive, does not
                // reset at 360

  // Loop times and constants
  int loopTime = 10; // loop pause in msec

  // Intake parameters
  // pot parameters
  int leftPotDesired = 30; // pot values range from 0 to leftPotDesired
  // When the pot value is greater than leftPotDesired, intakeL is out
  int rightPotDesired = 105; // pot values range from 360 to rightPotDesired
  // When the pot value is less than rightPotDesired, intakeR is out
  int potRange1 = 7;
  int potRange2 = 2;
  bool leftBrake = false;
  bool rightBrake = false;
  bool doIntake = false;
  // intake kP
  double intakekP = 4;
  double intakekI = 0.01;
  double intakekD = 0;
  double leftIntakeTotalError = 0;
  double rightIntakeTotalError = 0;
  double leftIntakePrevError = 0;
  double rightIntakePrevError = 0;

  // Autonomous parameters
  double initialSpeed =
      10; // Speed from which a robot accelerates in autonomous functions

  // Turning
  int turnMargin =
      40; // Time in msec for which turn needs to be at the correct angle
      public:
  double turnRange = 5; // Range (+-degrees) in which the turn needs to be in
                          // order to stop method

  // Strafing
  double strafeConstant =
      0.8; // Constant by which the speed of the front motors
           // is multiplied to straighten strafe drive

  // Wall Aligment
  double alignMargin = 200; // msec for which robot needs to be aligned
  double alignRange = 10;   // mm within which robot aligns

  // PID constants
  // Drive
  double drivekP = 0.1;     // 1.4
  double drivekD = 0;     // 2.3
  double drivekI = 0; // 0.000005

  // Turn
  double turnkP = 0.45; //1
  double turnkD = 0.09; //11.4
  double turnkI = 0; //0.002

  // Align
  double alignkP = 0.5;
  double alignkD = 16; // 3
  double alignkI = 0;  // 0.0024

  // Strafe
  double strafekP = 5;
  double strafekD = 10;
  double strafekI = 0.001;

  // Variables used for calculating PID
  double error;          // SensorValue - DesiredValue : Position
  double prevError = 0;  // Position loopTime msec ago
  double derivative;     // error - prevError : speed
  double totalError = 0; //+= error

  bool leftIntakeOpen = false;
  bool rightIntakeOpen = false;
  double openTime = 0.1;
  double openTL = 0;
  double openTR = 0;
  bool leftIntakeLogic = false;
  bool rightIntakeLogic = false;

public:
  void resetDriveEncoders(); // Resets all driver encoder positions to zero
  void resetPID();  // Set all PID values to zero
  double avgDriveEncoder(); // Returns average of all driver encoder positions
  double absAvgDriveEncoder(); // Returns average of all drive encoder abs positions
  void timeOutDrive(double t, double speed);
  void drive(int dir, double speed); // Drive forward (dir = 1) or backward (dir = -1). Called every loopTime msec
  void strafe(int dir, double speed); // Strafe right (dir = 1) or left (dir = -1)
  void brakeDrive(); // Stop the drive using brake mode brake
  void autoForward(double degrees, double iDeg, double fDeg, double speed); // Forward auto function. degrees > iDeg + fDeg
  void autoForward(double degrees, double iDeg, double fDeg, bool intake, double speed); // Forward auto function. degrees > iDeg + fDeg
  void dumbForward(double degrees, double iDeg, double fDeg, double speed); // No auto index
  void dumbBackward(double degrees, double iDeg, double fDeg, double speed); //No auto index
  void autoBackward(double degrees, double iDeg, double fDeg, double speed); // Backward auto function. degrees > iDeg + fDeg
  void autoTurnTo(double degrees); //PID turn, +degrees turns right, -degrees turns left
  void autoStrafeLeft(double degrees, double iDeg, double fDeg, double speed); // Strafe left auto function. degrees > iDeg + fDeg
  void autoStrafeRight(double degrees, double iDeg, double fDeg, double speed); // Strafe right auto function. degrees > iDeg + fDeg
  void intake(double speed); // Run intakes at speed
  void openDegrees(double speed, double degrees);
  void openOneIntake(double speed, double degrees, int side); //Open right if side = 1, left if side = -1
  void autoIntake(); // Automatically intake balls using vision
  void openIntakeDepreciated(); // Open intakes OLD
  void openIntake();
  void openIntakeTo(); // Open intakes and then continue
  void intakeBrake(); // Stop intakes using braketype hold
  void index(double speed); // Run indexer at speed
  void pIndex(double speed, double degrees); // Run indexer for set number of degrees at speed
  void outdex(double speed); // Outdex at speed
  void pOutdex(double speed, double degrees); //Outdex for a set number of degrees at speed
  void indexerBrake(); // Hold indexer
  void cIndex(); // Auto index
  void indexSense(); // Set position variables for auto index
  void shoot(); // Shoot 1 ball
  void doubleShot(); // Shoot 2 balls
  void alignTurnRight(double speed, double degrees); // Turn right into corner goal until param degrees is reached
  void alignTurnLeft(double speed, double degrees); // Turn left into corner goal until param degrees is reached
  void flipout(); // Run hood flipout
};