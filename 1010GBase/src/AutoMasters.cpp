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
    doIntake = false; // Stop the intaking
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
      indexSense();
      wait(loopTime, msec);
    }
    while (avgDriveEncoder() <
           degrees - fDeg) { // Drive at speed up until you reach final degrees
                             // (fDeg) threshold
      // Run the drive
      drive(1, speed);
      indexSense();
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
      indexSense();
      wait(loopTime, msec);
    }
    resetPID();
    // Stop the drive
    brakeDrive();
  }

  void
  dumbBackward(double degrees, double iDeg, double fDeg,
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
      indexSense();
      wait(loopTime, msec);
    }
    while (fabs(avgDriveEncoder()) <
           degrees - fDeg) { // Drive at speed up until you reach final degrees
                             // (fDeg) threshold
      // Run the drive
      drive(-1, speed);
      indexSense();
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
      indexSense();
      wait(loopTime, msec);
    }
    resetPID();
    // Stop the drive
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
          VisionSensor.largestObject.width > 80) {
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
    double leftError = -leftPotDesired + potL.angle(deg);
    leftIntakeTotalError += leftError;
    double leftDerivative = leftError - leftIntakePrevError;
    leftIntakePrevError = leftError;
    // Spin intake with PID values
    /*IntakeL.spin(reverse,
                 leftError * dC.intakekP + dC.leftIntakeTotalError * dC.intakekI
       - leftDerivative * dC.intakekD, pct);*/
    IntakeL.spin(reverse, 100, pct);
    if (leftError < potRange2) {
      leftBrake = true;
    }
    if (leftError < potRange1 && leftBrake) {
      IntakeL.stop(hold);
    }
    if (leftError >= potRange1) {
      leftBrake = false;
    }
    // Right Intake
    double rightError = -rightPotDesired + potR.angle(deg);
    rightIntakeTotalError += rightError;
    double rightDerivative = rightError - rightIntakePrevError;
    rightIntakePrevError = rightError;
    // Spin intake with PID values
    /*IntakeR.spin(reverse,
                 rightError * dC.intakekP +
                     dC.rightIntakeTotalError * dC.intakekI -
                     rightDerivative * dC.intakekD,
                 pct);*/
    IntakeR.spin(reverse, 100, pct);
    if (rightError < potRange2) { // inside small range
      rightBrake = true;
    }
    if (rightError < potRange1 &&
        rightBrake) { // inside large range and rightBrake
      IntakeR.stop(hold);
    }
    if (rightError >= potRange1) { // out of large range
      rightBrake = false;
    }
  }

  void openIntakeTo() {
    // PID open on bottom right
    while (fabs(leftPotDesired - potL.angle(deg)) > 10 ||
           fabs(-rightPotDesired + potR.angle(deg)) > 10) {
      // PID open on bottom right
      // bumper Left Intake
      double leftError = -leftPotDesired + potL.angle(deg);
      leftIntakeTotalError += leftError;
      double leftDerivative = leftError - leftIntakePrevError;
      leftIntakePrevError = leftError;
      // Spin intake with PID values
      /*IntakeL.spin(reverse,
                   leftError * dC.intakekP + dC.leftIntakeTotalError *
         dC.intakekI - leftDerivative * dC.intakekD, pct);*/
      IntakeL.spin(reverse, 100, pct);
      if (leftError < potRange2) {
        leftBrake = true;
      }
      if (leftError < potRange1 && leftBrake) {
        IntakeL.stop(hold);
      }
      if (leftError >= potRange1) {
        leftBrake = false;
      }
      // Right Intake
      double rightError = -rightPotDesired + potR.angle(deg);
      rightIntakeTotalError += rightError;
      double rightDerivative = rightError - rightIntakePrevError;
      rightIntakePrevError = rightError;
      // Spin intake with PID values
      /*IntakeR.spin(reverse,
                   rightError * dC.intakekP +
                       dC.rightIntakeTotalError * dC.intakekI -
                       rightDerivative * dC.intakekD,
                   pct);*/
      IntakeR.spin(reverse, 100, pct);
      if (rightError < potRange2) { // inside small range
        rightBrake = true;
      }
      if (rightError < potRange1 &&
          rightBrake) { // inside large range and rightBrake
        IntakeR.stop(hold);
      }
      if (rightError >= potRange1) { // out of large range
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
      IndexerTop.spin(forward, 100, pct);
      IndexerLow.spin(forward, 80, pct);
    }
    if (position3 && !position2) {
      IndexerLow.spin(forward, 80, pct);
    }
  }

  void indexSense() { // Sets index ball position variables
  if (LinePosition1.value(pct) < 60) { //Position 1
    position1 = true;
    Brain.Screen.drawCircle(300, 100, 50, green); //Visualisation
  } else {
    position1 = false;
    Brain.Screen.drawCircle(300, 100, 50, black);
  }

  if (LinePosition2.value(pct) < 70) { //Position 2
    position2 = true;
    Brain.Screen.drawCircle(200, 100, 50, green);
  } else {
    position2 = false;
    Brain.Screen.drawCircle(200, 100, 50, black);
  }

  if (LinePosition3L.value(pct) < 70) { //Position 3
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

  // Params
  // Aligns on a corner goal by turning to the right,
  // pivots around front right wheel, coasts back right wheel,
  // drives both left wheels
  // speed: speed at which the left motors spin
  // degreees: where the robot will face
  void alignTurnRight(double speed, double degrees) {
    resetDriveEncoders();
    DriveFR.stop(hold);  // Pivot on front right wheel
    DriveBR.stop(coast); // Coast on back right wheel

    // Run loop when heading is less than degrees
    // heading increases as robot turns right
    while (IMU.rotation() < degrees) {
      DriveFL.spin(forward, speed, pct);
      DriveBL.spin(forward, speed, pct);
      wait(10, msec);
    }
    resetPID();
    brakeDrive();
  }

  // Params
  // Aligns on a corner goal by turning to the left,
  // pivots around front left wheel, coasts back left wheel,
  // drives both right wheels
  // speed: speed at which the right motors spin
  // degreees: where the robot will face
  void alignTurnLeft(double speed, double degrees) {
    resetDriveEncoders();
    DriveFL.stop(hold);  // Pivot on front right wheel
    DriveBL.stop(coast); // Coast on back right wheel

    // Run loop when heading is greater than degrees
    // heading decreases (and goes negative) as robot turns left
    while (IMU.rotation() > degrees) {
      DriveFR.spin(forward, speed, pct);
      DriveBR.spin(forward, speed, pct);
      wait(10, msec);
    }
    resetPID();
    brakeDrive();
  }
  // -------------------------------------------------------------------------------
public:
  // Master Functions
  void newSkillsNew() {
    // Flipout
    openIntakeTo();
    wait(700, msec);
    dumbForward(200, 100, 100, 40);
    intake(100);
    dumbForward(100, 50, 50, 50);
    // Goal 1
    autoForward(350, 100, 100, 90);
    autoTurnTo(-135);
    intakeBrake();
    autoForward(800, 100, 100, 90);
    shoot();
    // Goal 2
    autoBackward(260, 100, 100, 90);
    autoTurnTo(0);
    openIntake();
    autoForward(1190, 100, 100, true, 90);
    autoTurnTo(-90);
    intakeBrake();
    autoForward(250, 100, 1, 90);
    wait(800, msec);
    shoot();
    // Goal 3
    autoBackward(450, 50, 50, 90);
    autoTurnTo(0);
    openIntake();
    autoForward(1400, 100, 600, true, 90);
    autoTurnTo(-45);
    autoStrafeLeft(600, 100, 100, 80);
    intakeBrake();
    autoForward(600, 100, 100, 90);
    shoot();
    // Goal 4
    autoBackward(1200, 100, 100, 90);
    autoTurnTo(90);
    openIntake();
    autoForward(680, 100, 300, true, 90);
    autoTurnTo(0);
    intakeBrake();
    autoForward(820, 100, 1, 90);
    wait(800, msec);
    shoot();
    // Goal 5
    autoBackward(70, 40, 30, 90);
    autoTurnTo(90);
    openIntake();
    autoForward(1100, 100, 100, true, 90);
    autoTurnTo(45);
    intakeBrake();
    autoForward(400, 100, 100, 90);
    shoot();
    // Goal 6
    autoBackward(250, 100, 100, 90);
    autoTurnTo(180);
    openIntake();
    autoForward(1245, 100, 100, true, 90);
    autoTurnTo(90);
    intakeBrake();
    autoForward(250, 100, 100, 90);
    wait(800, msec);
    shoot();
    // Goal 7
    autoBackward(450, 50, 50, 90);
    autoTurnTo(180);
    openIntake();
    autoForward(1300, 100, 400, true, 90);
    autoTurnTo(135);
    autoStrafeLeft(500, 100, 100, 80);
    intakeBrake();
    autoForward(600, 100, 100, 90);
    shoot();
    // Goal 8
    autoBackward(250, 100, 100, 90);
    autoTurnTo(270);
    autoForward(1200, 100, 100, 90);
    autoTurnTo(180);
    autoForward(250, 100, 100, 90);
    wait(800, msec);
    shoot();
    // Goal centre
    autoBackward(200, 100, 100, 90);
    autoTurnTo(0);
    openIntake();
    autoForward(500, 100, 100, true, 90);
    autoStrafeLeft(400, 100, 100, 80);
    intakeBrake();
    autoForward(400, 100, 200, 90);
    autoBackward(400, 100, 200, 90);
    autoStrafeRight(400, 100, 100, 80);
    openIntake();
    autoForward(300, 100, 100, 90);
    wait(800, msec);
    shoot();
    wait(200, msec);
    autoBackward(200, 100, 100, 90);
  }

  void rightHome() {
    // Flipout
    openIntake();
    wait(200, msec);
    // Goal 1
    dumbForward(40, 20, 40, 100);
    intake(100);
    autoTurnTo(25);
    intakeBrake();
    autoForward(100, 50, 1, 100);
    alignTurnRight(100, 42);
    shoot();
    // Goal 2
    autoBackward(600, 50, 100, 100);
    autoTurnTo(0);
    autoBackward(1100, 50, 100, 100);
    autoTurnTo(90); 
    intakeBrake();
    autoForward(350, 50, 150, 100);
    wait(400, msec);
    shoot();
    // Goal 3
    autoBackward(250, 50, 50, 100);
    autoTurnTo(180);
    autoForward(900, 50, 100, 100);
    autoTurnTo(135); 
    openIntake();
    autoForward(200, 50, 1, 100);
    intake(100);
    autoForward(200, 1, 50, 100);
    wait(300, msec);
    autoForward(150, 1, 100, 100);
    intakeBrake();
    shoot();
    autoBackward(200, 1, 1, 100);
  }

  void rightTwoAndMiddle() {
    // Flipout
    openIntake();
    wait(200, msec);
    // Goal 1
    dumbForward(40, 20, 40, 100);
    intake(100);
    autoTurnTo(25);
    intakeBrake();
    autoForward(100, 50, 1, 100);
    alignTurnRight(100, 42);
    shoot();
    // Goal 2
    autoBackward(600, 50, 100, 100);
    autoTurnTo(0);
    autoBackward(1100, 50, 100, 100);
    autoTurnTo(90); 
    intakeBrake();
    autoForward(350, 50, 150, 100);
    wait(400, msec);
    shoot();
    // Goal middle
    autoBackward(200, 50, 50, 100);
    autoTurnTo(0);
    autoBackward(600, 100, 100, 100);
    autoTurnTo(-90);
    autoForward(800, 100, 100, 80);
    autoStrafeRight(700, 100, 100, 100);
    autoBackward(500, 50, 100, 100);
  }

  void rightTwoAndSide() {
    int t = Brain.timer(sec);
    // Flipout
    openIntake();
    wait(200, msec);
    // Goal 1
    dumbForward(40, 20, 40, 100);
    intake(100);
    autoTurnTo(25);
    intakeBrake();
    autoForward(100, 50, 1, 100);
    alignTurnRight(100, 42);
    shoot();
    // Goal 2
    autoBackward(600, 50, 100, 100);
    autoTurnTo(0);
    autoBackward(1100, 50, 100, 100);
    autoTurnTo(90); 
    intakeBrake();
    autoForward(350, 50, 150, 100);
    wait(400, msec);
    shoot();
    // Goal side
    indexerBrake();
    intake(100);
    wait(700, msec);
    //dumbBackward(50, 10, 10, 40);
    while (!position1 && !position2 && !position3) {
      indexSense();
      wait(10, msec);
    }
    intakeBrake();
    autoBackward(350, 50, 50, 100);
    intake(100);
    autoTurnTo(-15);
    intakeBrake();
    autoForward(1500, 50, 150, 100);
    autoTurnTo(-75);
    brakeDrive();
    h = IMU.rotation();
      resetPID();
    while (Brain.timer(sec) < t + 14.3) {
      drive(1, 100);
    }
    shoot();
    autoBackward(400, 1, 50, 100);
  }
};