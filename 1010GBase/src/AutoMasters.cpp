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

  // Autonomous parameters
  double initialSpeed =
      10; // Speed from which a robot accelerates in autonomous functions

  // Turning
  int turnMargin =
      100; // Time in msec for which turn needs to be at the correct angle
  double turnRange = 0.7; // Range (+-degrees) in which the turn needs to be in
                          // order to stop method

  //Wall Aligment
  double alignMargin = 200; //msec for which robot needs to be aligned 
  double alignRange = 10; //mm within which robot aligns

  // PID constants
  // Drive
  double drivekP = 1.6; // 1.6
  double drivekD = 14; // 14
  double drivekI = 0.08; // 0.08

  // Turn
  double turnkP = 1.4;
  double turnkD = 2.2;
  double turnkI = 0.00002;

  // Align
  double alignkP = 0.5;
  double alignkD = 16; //3
  double alignkI = 0; //0.0024

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

  void resetPID() {
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

  void autoBackAlign(double d,
                     double speed) { // Backward auto alignment function
    resetDriveEncoders();
    
/*    int t = 0;                      // Time variable

    h = IMU.rotation();

    while (t < alignMargin) {
      // PID
      // Used to make robot go straight and stop when at the right distance from
      // wall
      //error = (IMU.rotation() - h) + ((DistanceSensor.objectDistance(mm) - 400) - d);
      error = DistanceSensor.objectDistance(mm) - d;
      derivative = error - prevError;
      totalError += error;
      prevError = error;

      DriveBL.spin(forward,
                   -speed - error * alignkP - totalError * alignkI -
                       derivative * alignkD,
                   vex::pct);
      DriveBR.spin(forward,
                   -speed - error * alignkP - totalError * alignkI -
                       derivative * alignkD,
                   vex::pct);
      DriveFL.spin(forward,
                   -speed - error * alignkP - totalError * alignkI -
                       derivative * alignkD,
                   vex::pct);
      DriveFR.spin(forward,
                   -speed - error * alignkP - totalError * alignkI -
                       derivative * alignkD,
                   vex::pct);

      cIndex();

      if (fabs(error) < alignRange) {
        t += loopTime;
      } else {
        t = 0;
      }

      wait(loopTime, msec);
    }
*/

    while (d < DistanceSensor.objectDistance(mm) - 200 ) {
      //Drive backwards
      drive(-1, speed);

      wait(loopTime, msec);
    }
    while (d < DistanceSensor.objectDistance(mm) - 200) {
      //Drive backwards but slowly

       double deccelerate = speed * (DistanceSensor.objectDistance(mm) - d);

      if (deccelerate < initialSpeed) { // Make sure that the motors never move
                                        // slower than initalSpeed
        deccelerate = initialSpeed;
      }

      // Run the drive
      drive(-1, deccelerate);

      wait(loopTime, msec);
    }

    resetPID();

    autoTurnTo(h);
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

  void outake(double speed) {
    IntakeL.spin(reverse, speed, vex::pct);
    IntakeR.spin(reverse, speed, vex::pct);
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

    if (LinePosition2.pressing() || LinePosition2R.pressing()) {
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

  void skillsNew() { //Start left corner against wall
/*
     //Flipout
    indexerBrake();
    intake(100);
    wait(800, msec);
    dumbForward(310, 100, 100, 50);

    //Goal 1
    autoForward(320, 100, 100, 80);
    autoTurnTo(-135);
    intakeBrake();
    autoForward(770, 100, 100, 80);
    shoot();

    //Goal 2
    autoBackward(250, 100, 150, 80);
    autoTurnTo(-90);
    autoBackward(1310, 100, 100, 80);
    autoTurnTo(-180);
    autoForward(150, 40, 40, 80);
    shoot();

    //Goal 3
    autoBackward(80, 40, 40, 80);
    autoTurnTo(-270);
    intake(100);
    autoForward(1100, 100, 100, 80);
    autoTurnTo(-230);
    intakeBrake();
    autoForward(500, 40, 40, 80);
    shoot();

    //Goal 4
    autoBackward(1150, 250, 250, 80);
    autoTurnTo(-270);
    intake(100);
    autoForward(980, 150, 150, 60);
    autoBackward(150, 70, 70, 80);
    autoTurnTo(-360);
    autoForward(900, 200, 500, 80);
    autoTurnTo(-270);
    intakeBrake();
    autoForward(180, 80, 100, 70);
    shoot();

    //Goal 5
    autoBackward(500, 200, 250, 70);
    autoTurnTo(-360);
    intake(100);
    autoForward(1300, 100, 500, 80);
    autoBackward(140, 70, 70, 70);
    autoTurnTo(-315);
    intakeBrake();
    autoForward(850, 100, 100, 70);
    shoot();
    
    //Goal 6
    autoBackward(500, 100, 100, 80);
    autoTurnTo(-450);
    autoForward(1170, 100, 100, 80);
    autoTurnTo(-360);
    autoForward(300, 50, 100, 70);
    shoot();
*/
    //Middle
    autoBackward(130, 50, 100, 80);
    autoTurnTo(-180);
    intake(100);
    autoForward(700, 100, 100, 60);
    //Poke align
    autoStrafeLeft(200, 80, 80, 50);
    intakeBrake();
    //Poke
    outake(100);
    autoForward(200, 100, 100, 60);
    intakeBrake();
    autoBackward(200, 100, 100, 80);
    //Realign
    autoStrafeRight(200, 80, 80, 50);
    intake(100);
    //Hug
    autoForward(250, 50, 50, 60);
    wait(300, msec);
    shoot();
    wait(300, msec);
    outake(100);
    autoBackward(300, 100, 100, 80);
   /* //Repoke
    autoStrafeLeft(200, 80, 80, 50);
    intakeBrake();
    autoForward(300, 100, 100, 60);
    autoBackward(300, 100, 100, 60);*/
  }

  void skillsTriplePoke() { // Start red, right corner goal like match auto
    // Intake
    indexerBrake();
    intake(100);
    wait(800, msec);
    // Forward - grab ball
    dumbForward(310, 100, 100, 60);

    intakeBrake();

    // Turn right 45
    autoTurnTo(25);

    // autoForward(60, 1, 1, 80);

    // Score 1
    pIndex(100, 700);
    // pOutdex(100, 200);
    // Backward
    autoBackward(500, 50, 200, 100);
    // Turn left 0
    autoTurnTo(0);
    // Backward
    autoBackward(870, 50, 50, 100);
    // Turn right 90
    autoTurnTo(-90);

    autoForward(1170, 50, 200, 80);
    autoBackward(300, 10, 10, 60);
    autoForward(300, 10, 10, 60);
    autoBackward(300, 10, 10, 60);
    autoForward(300, 10, 10, 60);
  }

  void redRightCorner() {
    // Intake
    indexerBrake();
    intake(100);
    wait(800, msec);
    // Forward - grab ball
    dumbForward(310, 100, 100, 60);
    /*
        while (!position2 && !position3) {
          cIndex();
          wait(10, msec);
        }*/

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

    outake(100);

    autoBackward(100, 1, 1, 60);
  }

  void blueRigntCorner() {}
};