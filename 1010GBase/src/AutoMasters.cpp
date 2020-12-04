#include "vex.h"

using namespace vex;

class AutoMasters { // Holds all master autonomous programs
// Indexer parameters
  bool position1;
  bool position2;
  bool position3;
  int linePos1Pct = 71;
  int linePos2Pct = 69;
  bool goingTo3 = false;

  double h = 0; // Heading in degrees. Rotation clockwise is positive, does not
                // reset at 360

  // Loop times and constants
  int loopTime = 10; // loop pause in msec

  // Autonomous parameters
  double initialSpeed =
      10; // Speed from which a robot accelerates in autonomous functions

  // Turning
  int turnMargin =
      200; // Time in msec for which turn needs to be at the correct angle
  double turnRange = 0.7; // Range (+-degrees) in which the turn needs to be in
                          // order to stop method

  // PID constants
  // Drive
  double drivekP = 0; // 0.8
  double drivekD = 0; // 0.5
  double drivekI = 0; // 0.002

  // Turn
  double turnkP = 1.4;
  double turnkD = 2.2;
  double turnkI = 0.00002;

  // Strafe
  double strafekP = 5;
  double strafekD = loopTime;
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
                 speed * dir - error * strafekP - totalError * strafekI -
                     derivative * strafekD,
                 vex::pct);
    DriveFR.spin(forward,
                 speed * -dir + error * strafekP + totalError * strafekI +
                     derivative * strafekD,
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

    // Stop the drive
    brakeDrive();
  }

  void autoTurnTo(double degrees) { //+degrees turns right, -degrees turns left
    int t = 0;                      // Time variable

    while (t < turnMargin) { // break when time exceeds the turnMargin
      // PID
      // Used to make robot go straight
      error = IMU.rotation() - degrees;
      derivative = error - prevError;
      totalError += error;
      prevError = error;

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

cIndex();

      wait(loopTime, msec);

      if (fabs(error) <
          turnRange) { // increase time when the robot is pointing in turnRange
        t += loopTime;
      } else {
        t = 0;
      }
    }

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

    autoTurnTo(h);

    // stop the drive
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

  void pIndex(double speed, double degrees) {
    IndexerL.resetPosition();
    IndexerR.resetPosition();

    while (fabs(IndexerR.position(vex::degrees)) < degrees) {
      IndexerL.spin(forward, speed, pct);
      IndexerR.spin(forward, speed, pct);

      wait(loopTime, msec);
    }
  }

  void outdex(double speed) {
    IndexerL.spin(reverse, speed, vex::pct);
    IndexerR.spin(reverse, speed, vex::pct);
  }

  void pOutdex(double speed, double degrees) {
    IndexerL.resetPosition();
    IndexerR.resetPosition();

    while (fabs(IndexerR.position(vex::degrees)) < degrees) {
      IndexerL.spin(reverse, speed, pct);
      IndexerR.spin(reverse, speed, pct);

      wait(loopTime, msec);
    }
  }

  void indexerBrake() {
    IndexerL.stop(hold);
    IndexerR.stop(hold);
  }

  void cIndex() {
    indexSense();
      if (!position2) {
        IndexerL.spin(forward, 80, pct);
        IndexerR.spin(forward, 80, pct);
      } else if (!position2 && !goingTo3) {
        IndexerL.stop(hold);
        IndexerR.stop(hold);
      } else if (!position2 && position3) {
        IndexerL.spin(reverse, 80, pct);
        IndexerR.spin(reverse, 80, pct);
      }
      if (position1 && position2 && !position3) {
        // spin until position3 reached
        goingTo3 = true;
        IndexerL.spin(forward, 80, pct);
        IndexerR.spin(forward, 80, pct);
      }
      if (position3) {
        goingTo3 = false;
      }
      if (position3) {
        IndexerL.stop(hold);
        IndexerR.stop(hold);
      }
  }

  void indexSense() { // Sets index ball position variables
    if (LinePosition1.value(pct) <= linePos1Pct) {
      position1 = true;
    } else {
      position1 = false;
    }

    if (LinePosition2.value(pct) <= linePos2Pct) {
      position2 = true;
    } else {
      position2 = false;
    }

    if (LinePosition3.pressing()) {
      position3 = true;
    } else {
      position3 = false;
    }
  }

  void shoot () {
    while (position3 && position2) {
      index(100);
      indexSense();
      wait(10, msec);
    }
    indexerBrake();
  }
  // -------------------------------------------------------------------------------
public:
  // Master Functions
  void autoCalibrate() { // Runs every single action
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

  void turnCalibrate() {
    autoTurnTo(-90);
    autoTurnTo(135);
    autoTurnTo(-135);
    autoTurnTo(180);
    autoTurnTo(-180);
    autoTurnTo(0);
  }

  void skillsAuto() { // Start red, left of middle
    intake(100);

    autoForward(570, 180, 180, 100);

    autoTurnTo(135);

    intakeBrake();

    autoForward(650, 180, 1, 100);

    // First goal
    pIndex(100, 1300);
    pOutdex(100, 300);
    indexerBrake();

    autoBackward(150, 70, 70, 90);

    intake(100);

    autoTurnTo(0);

    autoForward(1350, 180, 600, 100);

    autoTurnTo(90);

    pIndex(100, 360);
    indexerBrake();

    intakeBrake();

    autoForward(110, 90, 1, 100);

    // middle goal
    pIndex(100, 2000);
    indexerBrake();
    wait(500, msec);
    pIndex(100, 2000);
    indexerBrake();

    autoBackward(110, 90, 90, 100);

    intake(100);

    autoTurnTo(-7);

    autoForward(1400, 180, 500, 100);

    autoBackward(200, 100, 100, 100);

    autoTurnTo(45);

    pIndex(100, 360);
    indexerBrake();

    intakeBrake();

    autoForward(720, 180, 1, 100);

    // Back goal
    pIndex(100, 1000);
    pOutdex(100, 300);
    indexerBrake();

    autoBackward(680, 180, 180, 100);
  }

  void redRightCorner() {
    //Intake
    intake(100);
    wait(1000, msec);
    //Forward - grab ball
    autoForward(100, 300, 100, 70);
    //Turn right 45
    autoTurnTo(30);

    while (!position2 && !position3) {
      cIndex();
      wait(10, msec);
    }

    intakeBrake();

    //Score 1
    shoot();
    //Backward
    //Turn left 0
    //Backward
    //Turn right 90
    //Forward
    //Score
    //Backward
    //Turn left 180
    //Forward
    //Intake
    //Turn right 135
    //Score
  }

  void blueRigntCorner() {

  }
};