#pragma once
#include "vex.h"

class AutoFunctions { //Holds basic autonomous functions
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
      70; // Time in msec for which turn needs to be at the correct angle
  double turnRange = 0.7; // Range (+-degrees) in which the turn needs to be in
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
  double drivekP = 1.4;     // 1.6
  double drivekD = 2.3;     // 14
  double drivekI = 0.00005; // 0.08

  // Turn
  double turnkP = 1.4;
  double turnkD = 2.3;
  double turnkI = 0.00005;

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

  public:
  void resetDriveEncoders();

  void resetPID();

  double avgDriveEncoder();

  double absAvgDriveEncoder();

  void drive(int dir, double speed);

  void strafe(int dir,
              double speed); // Strafe right (dir = 1) or left (dir = -1)

  void brakeDrive(); // Stop the drive using brake mode brake

  void
  autoForward(double degrees, double iDeg, double fDeg,
              double speed); // Forward auto function. degrees > iDeg + fDeg

              void
  autoForward(double degrees, double iDeg, double fDeg, bool intake,
              double speed); // Forward auto function. degrees > iDeg + fDeg

  void
  dumbForward(double degrees, double iDeg, double fDeg,
              double speed);

  void
  dumbBackward(double degrees, double iDeg, double fDeg,
              double speed);

  void
  autoBackward(double degrees, double iDeg, double fDeg,
               double speed);

  void autoTurnTo(double degrees);

  void autoStrafeLeft(
      double degrees, double iDeg, double fDeg,
      double speed);

  void autoStrafeRight(
      double degrees, double iDeg, double fDeg,
      double speed);

  void intake(double speed);

  void autoIntake();

  void openIntake();

  void openIntakeTo();

  void intakeBrake();

  void index(double speed);

  void pIndex(double speed, double degrees);

  void outdex(double speed);

  void pOutdex(double speed, double degrees);

  void indexerBrake();

  void cIndex();

  void indexSense();

  void shoot();

  void doubleShot();

  void alignTurnRight(double speed, double degrees);

  void alignTurnLeft(double speed, double degrees);
};