#pragma once
#include "vex.h"

using namespace vex;

class AutoMasters { // Holds all master autonomous programs
  // Indexer parameters
  bool position1;
  bool position2;
  bool position3;

  double h = 0; // Heading in degrees. Rotation clockwise is positive, does not
                // reset at 360

  // Loop times and constants
  int loopTime = 10; // loop pause in msec

  // Intake parameters
  // pot parameters
  int leftPotDesired = 220; // pot values range from 0 to leftPotDesired
  // When the pot value is greater than leftPotDesired, intakeL is out
  int rightPotDesired = 115; // pot values range from 360 to rightPotDesired
  // When the pot value is less than rightPotDesired, intakeR is out
  int potRange1 = 5;
  int potRange2 = 3;
  bool leftBrake = false;
  bool rightBrake = false;
  bool doIntake = false;
  // intake kP
  double intakekP = 1.2;
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
      100; // Time in msec for which turn needs to be at the correct angle
  double turnRange = 0.7; // Range (+-degrees) in which the turn needs to be in
                          // order to stop method

  // Strafing
  double strafeConstant = 0.65; // Constant by which the speed of the front motors
                             // is multiplied to straighten strafe drive

  // Wall Aligment
  double alignMargin = 200; // msec for which robot needs to be aligned
  double alignRange = 10;   // mm within which robot aligns

  // PID constants
  // Drive
  double drivekP = 1.6;  // 1.6
  double drivekD = 14;   // 14
  double drivekI = 0.08; // 0.08

  // Turn
  double turnkP = 1.4;
  double turnkD = 2.2;
  double turnkI = 0.00002;

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

  void resetDriveEncoders() { // Resets all driver encoder positions to zero
    DriveBL.resetPosition();
    DriveBR.resetPosition();
    DriveFL.resetPosition();
    DriveFR.resetPosition();
  }

  void resetPID() { // Set all PID values to zero
    error = 0;
    prevError = 0;
    derivative = 0;
    totalError = 0;
  }

  double avgDriveEncoder() { // Returns average of all driver encoder positions
    return (DriveBL.position(vex::deg) + DriveBR.position(vex::deg) +
            DriveFL.position(vex::deg) + DriveBR.position(vex::deg)) /
           4;
  }

  double
  absAvgDriveEncoder() { // Returns average of all drive encoder abs positions
    return (fabs(DriveBL.position(vex::deg)) +
            fabs(DriveBR.position(vex::deg)) +
            fabs(DriveFL.position(vex::deg)) +
            fabs(DriveBR.position(vex::deg))) /
           4;
  }

public:
  void drive(int dir, double speed) { // Drive forward (dir = 1) or backward
                                      // (dir = -1). Called every loopTime msec
    // PID
    // Used to make robot go straight
    error = IMU.rotation() - h;
    derivative = error - prevError;
    totalError += error;
    prevError = error;

    DriveBL.spin(forward,
                 speed * dir - error * drivekP - totalError * drivekI -
                     derivative * drivekD,
                 vex::pct);
    DriveBR.spin(forward,
                 speed * dir + error * drivekP + totalError * drivekI +
                     derivative * drivekD,
                 vex::pct);
    DriveFL.spin(forward,
                 speed * dir - error * drivekP - totalError * drivekI -
                     derivative * drivekD,
                 vex::pct);
    DriveFR.spin(forward,
                 speed * dir + error * drivekP + totalError * drivekI +
                     derivative * drivekD,
                 vex::pct);
  }

  void strafe(int dir,
              double speed) { // Strafe right (dir = 1) or left (dir = -1)
    // PID
    // Used to make robot go straight
    error = IMU.rotation() - h;
    derivative = error - prevError;
    totalError += error;
    prevError = error;

    DriveBL.spin(forward,
                 speed * -dir - error * strafekP - totalError * strafekI -
                     derivative * strafekD,
                 vex::pct);
    DriveBR.spin(forward,
                 speed * dir + error * strafekP + totalError * strafekI +
                     derivative * strafekD,
                 vex::pct);
    DriveFL.spin(forward,
                 speed * dir * strafeConstant - error * strafekP -
                     totalError * strafekI - derivative * strafekD,
                 vex::pct);
    DriveFR.spin(forward,
                 speed * -dir * strafeConstant + error * strafekP +
                     totalError * strafekI + derivative * strafekD,
                 vex::pct);
  }

  void brakeDrive() { // Stop the drive using brake mode brake
    DriveBL.stop(vex::brake);
    DriveBR.stop(vex::brake);
    DriveFL.stop(vex::brake);
    DriveFR.stop(vex::brake);
  }

  void
  autoForward(double degrees, double iDeg, double fDeg,
              double speed) { // Forward auto function. degrees > iDeg + fDeg
    resetDriveEncoders();
    h = IMU.rotation();
    while (avgDriveEncoder() <
           iDeg) { // Accelerate for the initial degrees (iDeg)
      double accelerate = speed * avgDriveEncoder() / iDeg;
      if (accelerate < initialSpeed) { // Make sure that the motors never move
                                       // slower than initalSpeed
        accelerate = initialSpeed;
      }
      // Run the drive
      drive(1, accelerate);
      cIndex();
      wait(loopTime, msec);
    }
    while (avgDriveEncoder() <
           degrees - fDeg) { // Drive at speed up until you reach final degrees
                             // (fDeg) threshold
      // Run the drive
      drive(1, speed);
      cIndex();
      wait(loopTime, msec);
    }
    resetDriveEncoders();
    while (avgDriveEncoder() <
           fDeg) { // Decellerate for the final degrees (fDeg)
      double deccelerate = speed - speed * avgDriveEncoder() / fDeg;
      if (deccelerate < initialSpeed) { // Make sure that the motors never move
                                        // slower than initalSpeed
        deccelerate = initialSpeed;
      }
      // Run the drive
      drive(1, deccelerate);
      cIndex();
      wait(loopTime, msec);
    }
    resetPID();
    // Stop the drive
    brakeDrive();
  }

  void
  autoForward(double degrees, double iDeg, double fDeg, bool intake,
              double speed) { // Forward auto function. degrees > iDeg + fDeg
    resetDriveEncoders();
    h = IMU.rotation();
    while (avgDriveEncoder() <
           iDeg) { // Accelerate for the initial degrees (iDeg)
      double accelerate = speed * avgDriveEncoder() / iDeg;
      if (accelerate < initialSpeed) { // Make sure that the motors never move
                                       // slower than initalSpeed
        accelerate = initialSpeed;
      }
      // Run the drive
      drive(1, accelerate);
      autoIntake();
      cIndex();
      wait(loopTime, msec);
    }
    while (avgDriveEncoder() <
           degrees - fDeg) { // Drive at speed up until you reach final degrees
                             // (fDeg) threshold
      // Run the drive
      drive(1, speed);
      autoIntake();
      cIndex();
      wait(loopTime, msec);
    }
    resetDriveEncoders();
    while (avgDriveEncoder() <
           fDeg) { // Decellerate for the final degrees (fDeg)
      double deccelerate = speed - speed * avgDriveEncoder() / fDeg;
      if (deccelerate < initialSpeed) { // Make sure that the motors never move
                                        // slower than initalSpeed
        deccelerate = initialSpeed;
      }
      // Run the drive
      drive(1, deccelerate);
      autoIntake();
      cIndex();
      wait(loopTime, msec);
    }
    resetPID();
    // Stop the drive
    brakeDrive();
  }

  void
  dumbForward(double degrees, double iDeg, double fDeg,
              double speed) { // Forward auto function. degrees > iDeg + fDeg
    resetDriveEncoders();

    h = IMU.rotation();

    while (avgDriveEncoder() <
           iDeg) { // Accelerate for the initial degrees (iDeg)
      double accelerate = speed * avgDriveEncoder() / iDeg;

      if (accelerate < initialSpeed) { // Make sure that the motors never move
                                       // slower than initalSpeed
        accelerate = initialSpeed;
      }

      // Run the drive
      drive(1, accelerate);

      wait(loopTime, msec);
    }
    while (avgDriveEncoder() <
           degrees - fDeg) { // Drive at speed up until you reach final degrees
                             // (fDeg) threshold
      // Run the drive
      drive(1, speed);

      wait(loopTime, msec);
    }
    resetDriveEncoders();
    while (avgDriveEncoder() <
           fDeg) { // Decellerate for the final degrees (fDeg)
      double deccelerate = speed - speed * avgDriveEncoder() / fDeg;

      if (deccelerate < initialSpeed) { // Make sure that the motors never move
                                        // slower than initalSpeed
        deccelerate = initialSpeed;
      }

      // Run the drive
      drive(1, deccelerate);

      wait(loopTime, msec);
    }

    resetPID();

    // Stop the drive
    brakeDrive();
  }

  void
  dumberForward(double speed) { // Drive into the goal until both bumpers press
    resetDriveEncoders();
    h = IMU.rotation();
    while (!bumperL.pressing() || !bumperR.pressing()) {
      drive(1, speed);
      wait(loopTime, msec);
    }
    resetPID();
    brakeDrive();
  }

  void
  autoBackward(double degrees, double iDeg, double fDeg,
               double speed) { // Backward auto function. degrees > iDeg + fDeg
    resetDriveEncoders();

    h = IMU.rotation();

    while (fabs(avgDriveEncoder()) <
           iDeg) { // Accelerate for the initial degrees (iDeg)
      double accelerate = speed * absAvgDriveEncoder() / iDeg;

      if (accelerate < initialSpeed) { // Make sure that the motors never move
                                       // slower than initalSpeed
        accelerate = initialSpeed;
      }

      // Run the drive
      drive(-1, accelerate);

      cIndex();

      wait(loopTime, msec);
    }
    while (fabs(avgDriveEncoder()) <
           degrees - fDeg) { // Drive at speed up until you reach final degrees
                             // (fDeg) threshold
      // Run the drive
      drive(-1, speed);

      cIndex();

      wait(loopTime, msec);
    }
    resetDriveEncoders();
    while (fabs(avgDriveEncoder()) <
           fDeg) { // Decellerate for the final degrees (fDeg)
      double deccelerate = speed - speed * absAvgDriveEncoder() / fDeg;

      if (deccelerate < initialSpeed) { // Make sure that the motors never move
                                        // slower than initalSpeed
        deccelerate = initialSpeed;
      }

      // Run the drive
      drive(-1, deccelerate);

      cIndex();

      wait(loopTime, msec);
    }

    resetPID();

    // Stop the drive
    brakeDrive();
  }

  void autoTurnTo(double degrees) { //+degrees turns right, -degrees turns left
    int t = 0;                      // Time variable

    while (t < turnMargin) { // break when time exceeds the turnMargin
      // PID
      error = IMU.rotation() - degrees;
      derivative = error - prevError;
      totalError += error;
      prevError = error;

      // Run motors according to PID values
      DriveBL.spin(forward,
                   -error * turnkP - totalError * turnkI - derivative * turnkD,
                   vex::pct);
      DriveBR.spin(forward,
                   error * turnkP + totalError * turnkI + derivative * turnkD,
                   vex::pct);
      DriveFL.spin(forward,
                   -error * turnkP - totalError * turnkI - derivative * turnkD,
                   vex::pct);
      DriveFR.spin(forward,
                   error * turnkP + totalError * turnkI + derivative * turnkD,
                   vex::pct);

      cIndex(); // Auto index during turn

      wait(loopTime, msec); // Wait to prevent wasted resources
      // Exit the turn function once the robot is pointing in the correct
      // direction
      if (fabs(error) < turnRange) { // increase time value when the robot is
                                     // pointing within turnRange
        t += loopTime;
      } else {
        t = 0;
      }
    }
    resetPID();
    // stop the drive
    brakeDrive();
  }

  void autoStrafeLeft(
      double degrees, double iDeg, double fDeg,
      double speed) { // Strafe left auto function. degrees > iDeg + fDeg) {
    resetDriveEncoders();

    h = IMU.rotation();

    // accelerate from initialSpeed to speed while strafing through iDeg
    while (absAvgDriveEncoder() < iDeg) {
      double accelerate = speed * absAvgDriveEncoder() / iDeg;

      if (accelerate < initialSpeed) { // Make sure that the motors never move
                                       // slower than initalSpeed
        accelerate = initialSpeed;
      }
      strafe(-1, accelerate);

      cIndex();

      wait(loopTime, msec);
    }
    while (absAvgDriveEncoder() < degrees - fDeg) { // strafe until fDeg at
                                                    // speed
      strafe(-1, speed);

      cIndex();

      wait(loopTime, msec);
    }
    resetDriveEncoders();
    while (absAvgDriveEncoder() <
           fDeg) { // Decellerate while strafing through fDeg
      double deccelerate = speed - speed * absAvgDriveEncoder() / fDeg;

      if (deccelerate < initialSpeed) { // Make sure that the motors never move
                                        // slower than initalSpeed
        deccelerate = initialSpeed;
      }
      strafe(-1, deccelerate);

      cIndex();

      wait(loopTime, msec);
    }

    resetPID();

    autoTurnTo(h);

    // stop the drive
    brakeDrive();
  }

  void autoStrafeRight(
      double degrees, double iDeg, double fDeg,
      double speed) { // Strafe right auto function. degrees > iDeg + fDeg) {
    resetDriveEncoders();

    h = IMU.rotation();

    // accelerate from initialSpeed to speed while strafing through iDeg
    while (absAvgDriveEncoder() < iDeg) {
      double accelerate = speed * absAvgDriveEncoder() / iDeg;

      if (accelerate < initialSpeed) { // Make sure that the motors never move
                                       // slower than initalSpeed
        accelerate = initialSpeed;
      }
      strafe(1, accelerate);

      cIndex();

      wait(loopTime, msec);
    }
    while (absAvgDriveEncoder() < degrees - fDeg) { // strafe until fDeg at
                                                    // speed
      strafe(1, speed);

      cIndex();

      wait(loopTime, msec);
    }
    resetDriveEncoders();
    while (absAvgDriveEncoder() <
           fDeg) { // Decellerate while strafing through fDeg
      double deccelerate = speed - speed * absAvgDriveEncoder() / fDeg;

      if (deccelerate < initialSpeed) { // Make sure that the motors never move
                                        // slower than initalSpeed
        deccelerate = initialSpeed;
      }
      strafe(1, deccelerate);

      cIndex();

      wait(loopTime, msec);
    }

    resetPID();

    autoTurnTo(h);

    // stop the drive
    brakeDrive();
  }

  void intake(double speed) {
    IntakeL.spin(forward, speed, vex::pct);
    IntakeR.spin(forward, speed, vex::pct);
  }

  void autoIntake() {
    // Red code
    for (int i = 0; i < 2; i++) {
      if (i == 0) {
        VisionSensor.takeSnapshot(SIG_2); // Red
      } else {
        VisionSensor.takeSnapshot(SIG_1); // Blue
      }
      if (VisionSensor.largestObject.exists &&
          VisionSensor.largestObject.width > 90) {
        if (VisionSensor.largestObject.centerY >
            180) { // Close until position1 is clicked
          doIntake = true;
        } else if (VisionSensor.largestObject.centerY > 80 &&
                   VisionSensor.largestObject.centerY <= 180) { // Open
          openIntake();
        }
        if (doIntake && position1) {
          doIntake = false;
        }
        if (doIntake) {
          IntakeL.spin(forward, 100, vex::pct);
          IntakeR.spin(forward, 100, vex::pct);
          leftIntakeTotalError = 0;
          rightIntakeTotalError = 0;
          cIndex();
        }
      }
    }
  }

  void openIntake() {
    // PID open on bottom right
    // bumper Left Intake
    double leftError = leftPotDesired - potL.angle(deg);
    leftIntakeTotalError += leftError;
    double leftDerivative = leftError - leftIntakePrevError;
    leftIntakePrevError = leftError;
    // Spin intake with PID values
    IntakeL.spin(reverse,
                 leftError * intakekP + leftIntakeTotalError * intakekI -
                     leftDerivative * intakekD,
                 pct);
    if (fabs(leftError) < potRange2) {
      leftBrake = true;
    }
    if (fabs(leftError) < potRange1 && leftBrake) {
      IntakeL.stop(hold);
    }
    if (fabs(leftError) >= potRange1) {
      leftBrake = false;
    }
    // Right Intake
    double rightError = -rightPotDesired + potR.angle(deg);
    rightIntakeTotalError += rightError;
    double rightDerivative = rightError - rightIntakePrevError;
    rightIntakePrevError = rightError;
    // Spin intake with PID values
    IntakeR.spin(reverse,
                 rightError * intakekP + rightIntakeTotalError * intakekI -
                     rightDerivative * intakekD,
                 pct);
    if (fabs(rightError) < potRange2) {
      rightBrake = true;
    }
    if (fabs(rightError) < potRange1 && rightBrake) {
      IntakeR.stop(hold);
    }
    if (fabs(rightError) >= potRange1) {
      rightBrake = false;
    }
  }

  void openIntakeTo() {
    // PID open on bottom right
    while (fabs(leftPotDesired - potL.angle(deg)) < potRange1 && fabs(-rightPotDesired + potR.angle(deg)) < potRange1) {
      double leftError = leftPotDesired - potL.angle(deg);
      leftIntakeTotalError += leftError;
      double leftDerivative = leftError - leftIntakePrevError;
      leftIntakePrevError = leftError;
      // Spin intake with PID values
      IntakeL.spin(reverse,
                   leftError * intakekP + leftIntakeTotalError * intakekI -
                       leftDerivative * intakekD,
                   pct);
      if (fabs(leftError) < potRange2) {
        leftBrake = true;
      }
      if (fabs(leftError) < potRange1 && leftBrake) {
        IntakeL.stop(hold);
      }
      if (fabs(leftError) >= potRange1) {
        leftBrake = false;
      }
      // Right Intake
      double rightError = -rightPotDesired + potR.angle(deg);
      rightIntakeTotalError += rightError;
      double rightDerivative = rightError - rightIntakePrevError;
      rightIntakePrevError = rightError;
      // Spin intake with PID values
      IntakeR.spin(reverse,
                   rightError * intakekP + rightIntakeTotalError * intakekI -
                       rightDerivative * intakekD,
                   pct);
      if (fabs(rightError) < potRange2) {
        rightBrake = true;
      }
      if (fabs(rightError) < potRange1 && rightBrake) {
        IntakeR.stop(hold);
      }
      if (fabs(rightError) >= potRange1) {
        rightBrake = false;
      }
      wait(loopTime, msec);
    }
  }

  void intakeBrake() {
    IntakeL.stop(hold);
    IntakeR.stop(hold);
  }

  void index(double speed) {
    IndexerTop.spin(forward, speed, vex::pct);
    IndexerLow.spin(forward, speed, vex::pct);
  }

  void pIndex(double speed, double degrees) {
    IndexerTop.resetPosition();
    IndexerLow.resetPosition();

    while (fabs(IndexerLow.position(vex::degrees)) < degrees) {
      IndexerTop.spin(forward, speed, pct);
      IndexerLow.spin(forward, speed, pct);

      wait(loopTime, msec);
    }
  }

  void outdex(double speed) {
    IndexerTop.spin(reverse, speed, vex::pct);
    IndexerLow.spin(reverse, speed, vex::pct);
  }

  void pOutdex(double speed, double degrees) {
    IndexerTop.resetPosition();
    IndexerLow.resetPosition();

    while (fabs(IndexerLow.position(vex::degrees)) < degrees) {
      IndexerTop.spin(reverse, speed, pct);
      IndexerLow.spin(reverse, speed, pct);

      wait(loopTime, msec);
    }
  }

  void indexerBrake() {
    IndexerTop.stop(hold);
    IndexerLow.stop(hold);
  }

  void cIndex() {
    indexerBrake();
    indexSense();
    if (!position3) {
      IndexerTop.spin(forward, 80, pct);
      IndexerLow.spin(forward, 80, pct);
    }
    if (position3 && !position2) {
      IndexerLow.spin(forward, 80, pct);
    }
  }

  void indexSense() { // Sets index ball position variables
    if (LinePosition1.pressing()) {
      position1 = true;
      Brain.Screen.drawCircle(300, 100, 50, green);
    } else {
      position1 = false;
      Brain.Screen.drawCircle(300, 100, 50, black);
    }

    if (LinePosition2.pressing()) {
      position2 = true;
      Brain.Screen.drawCircle(200, 100, 50, green);
    } else {
      position2 = false;
      Brain.Screen.drawCircle(200, 100, 50, black);
    }

    if (LinePosition3.pressing()) {
      position3 = true;
      Brain.Screen.drawCircle(100, 100, 50, green);
    } else {
      position3 = false;
      Brain.Screen.drawCircle(100, 100, 50, black);
    }
  }

  void shoot() {
    double rotTo = IndexerTop.position(deg) + 1000;
    while (IndexerTop.position(deg) < rotTo) {
      IndexerTop.spin(forward, 100, vex::pct);
      wait(10, msec);
    }
    indexerBrake();
  }

  void doubleShot() {
    // Run single shot
    shoot();
    while (!position3) {
      // cIndex until position3
      cIndex();
      wait(10, msec);
    }
    shoot();
  }
  // -------------------------------------------------------------------------------
public:
  // Master Functions
  void autoCalibrate() { // Runs every single action
    autoForward(720, 180, 360, 100);
    autoTurnTo(-90);
    autoTurnTo(0);
    autoTurnTo(90);
    autoTurnTo(0);
    autoStrafeLeft(720, 180, 180, 100);
    autoStrafeRight(720, 180, 180, 100);
    autoBackward(720, 180, 360, 100);
    autoTurnTo(0);
    intake(100);
    wait(1000, msec);
    openIntake();
    wait(1000, msec);
    intakeBrake();
    index(127);
    wait(1000, msec);
    outdex(127);
    wait(1000, msec);
    indexerBrake();
  }

  void turnCalibrate() {
    autoTurnTo(-90);
    autoTurnTo(135);
    autoTurnTo(-135);
    autoTurnTo(180);
    autoTurnTo(-180);
    autoTurnTo(0);
  }

  void newSkillsNew() {
    // Flipout
    indexerBrake();
    openIntakeTo();
    wait(1000, msec);
    dumbForward(310, 100, 100, 50);
    intake(100);
    // Goal 1
    autoForward(320, 100, 100, 80);
    autoTurnTo(-135);
    intakeBrake();
    autoForward(770, 100, 100, 80);
    shoot();
    // Goal 2
    autoBackward(350, 100, 100, 80);
    autoTurnTo(0);
    openIntake();
    autoForward(1050, 100, 100, true, 80);
    autoTurnTo(-90);
    autoForward(300, 100, 1, 80);
    intakeBrake();
    dumberForward(40);
    shoot();
    // Goal 3
    autoBackward(450, 50, 50, 80);
    autoTurnTo(0);
    autoForward(1300, 100, 300, true, 80);
    autoTurnTo(-45);
    autoStrafeLeft(400, 100, 100, 70);
    autoForward(500, 100, 100, 80);
    // Goal 4
    // Goal 5
    // Goal 6
    // Goal 7
    // Goal 8
    // Goal centre
  }

  void redRightCorner() {
    // Intake
    indexerBrake();
    openIntake();
    wait(800, msec);
    // Forward - grab ball
    dumbForward(310, 100, 100, 60);
    intake(100);

    intakeBrake();

    // Turn right 45
    autoTurnTo(25);

    // autoForward(60, 1, 1, 80);

    // Score 1
    pIndex(100, 700);
    // pOutdex(100, 200);
    // Backward
    autoBackward(500, 50, 50, 100);
    intake(100);
    // Turn left 0
    autoTurnTo(0);
    // Backward
    autoBackward(900, 50, 50, 100);
    // Turn right 90
    autoTurnTo(90);
    intakeBrake();
    // Forward
    autoForward(100, 50, 50, 100);
    // Score
    pIndex(100, 900);
    // Backward
    autoBackward(100, 50, 50, 100);
    // Turn left 180
    autoTurnTo(180);
    // Forward
    autoForward(1000, 50, 50, 100);
    // Intake
    intake(100);
    // Turn right 135
    autoTurnTo(135);
    // Forward
    autoForward(520, 10, 250, 100);

    autoBackward(50, 10, 10, 80);

    autoForward(50, 10, 10, 80);

    intakeBrake();

    // Score
    pIndex(100, 6000);

    autoBackward(100, 1, 1, 60);
  }

  void blueRigntCorner() {}
};