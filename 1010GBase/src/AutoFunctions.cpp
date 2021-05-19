#include "AutoFunctions.h"

void AutoFunctions::resetDriveEncoders() {
  DriveBL.resetPosition();
  DriveBR.resetPosition();
  DriveFL.resetPosition();
  DriveFR.resetPosition();
}

void AutoFunctions::resetPID() {
  error = 0;
  prevError = 0;
  derivative = 0;
  totalError = 0;
}

double AutoFunctions::avgDriveEncoder() {
  return (DriveBL.position(vex::deg) + DriveBR.position(vex::deg) +
          DriveFL.position(vex::deg) + DriveBR.position(vex::deg)) /
         4;
}

double AutoFunctions::absAvgDriveEncoder() {
  return (fabs(DriveBL.position(vex::deg)) + fabs(DriveBR.position(vex::deg)) +
          fabs(DriveFL.position(vex::deg)) + fabs(DriveBR.position(vex::deg))) /
         4;
}

void AutoFunctions::drive(int dir, double speed) {
  // PID
  // Used to make robot go straight
  error = IMUL.rotation() - h;
  derivative = error - prevError;
  totalError += error;
  prevError = error;

  DriveBL.spin(forward,
               speed * dir - error * drivekP - totalError * drivekI -
                   derivative * drivekD,
               pct);
  DriveBR.spin(forward,
               speed * dir + error * drivekP + totalError * drivekI +
                   derivative * drivekD,
               pct);
  DriveFL.spin(forward,
               speed * dir - error * drivekP - totalError * drivekI -
                   derivative * drivekD,
               pct);
  DriveFR.spin(forward,
               speed * dir + error * drivekP + totalError * drivekI +
                   derivative * drivekD,
               pct);
}

void AutoFunctions::strafe(
    int dir,
    double speed) { // Strafe right (dir = 1) or left (dir = -1)
  // PID
  // Used to make robot go straight
  error = IMUL.rotation() - h;
  derivative = error - prevError;
  totalError += error;
  prevError = error;

  DriveBL.spin(forward,
               speed * -dir - error * strafekP - totalError * strafekI -
                   derivative * strafekD,
               pct);
  DriveBR.spin(forward,
               speed * dir + error * strafekP + totalError * strafekI +
                   derivative * strafekD,
               pct);
  DriveFL.spin(forward,
               speed * dir * strafeConstant - error * strafekP -
                   totalError * strafekI - derivative * strafekD,
               pct);
  DriveFR.spin(forward,
               speed * -dir * strafeConstant + error * strafekP +
                   totalError * strafekI + derivative * strafekD,
               pct);
}

void AutoFunctions::brakeDrive() {
  DriveBL.stop(vex::brake);
  DriveBR.stop(vex::brake);
  DriveFL.stop(vex::brake);
  DriveFR.stop(vex::brake);
}

void AutoFunctions::autoForward(double degrees, double iDeg, double fDeg,
                                double speed) {
  resetDriveEncoders();
  h = IMUL.rotation();
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
  while (avgDriveEncoder() < fDeg) { // Decellerate for the final degrees (fDeg)
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

void AutoFunctions::autoForward(double degrees, double iDeg, double fDeg,
                                bool intake, double speed) {
  resetDriveEncoders();
  h = IMUL.rotation();
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
  while (avgDriveEncoder() < fDeg) { // Decellerate for the final degrees (fDeg)
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

void AutoFunctions::timeOutDrive(double t, double speed) {
  double newT = Brain.timer(sec) + t;
  while (Brain.timer(sec) < newT) {
  DriveFL.spin(forward, speed, pct);
  DriveFR.spin(forward, speed, pct);
  DriveBL.spin(forward, speed, pct);
  DriveBR.spin(forward, speed, pct);
  wait(10, msec);
  }
  DriveFL.stop();
  DriveFR.stop();
  DriveBL.stop();
  DriveBR.stop();
}

void AutoFunctions::dumbForward(double degrees, double iDeg, double fDeg,
                                double speed) {
  resetDriveEncoders();
  h = IMUL.rotation();
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
  while (avgDriveEncoder() < fDeg) { // Decellerate for the final degrees (fDeg)
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

void AutoFunctions::dumbBackward(double degrees, double iDeg, double fDeg,
                                 double speed) {
  resetDriveEncoders();
  h = IMUL.rotation();
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

void AutoFunctions::autoBackward(double degrees, double iDeg, double fDeg,
                                 double speed) {
  resetDriveEncoders();
  h = IMUL.rotation();
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

void AutoFunctions::autoTurnTo(double degrees) {
  int t = 0;               // Time variable
  while (t < turnMargin) { // break when time exceeds the turnMargin
    // PID
    error = IMUL.rotation() - degrees;
    derivative = error - prevError;
    totalError += error;
    prevError = error;

    // Run motors according to PID values
    DriveBL.spin(forward,
                 -error * turnkP - totalError * turnkI - derivative * turnkD,
                 pct);
    DriveBR.spin(forward,
                 error * turnkP + totalError * turnkI + derivative * turnkD,
                 pct);
    DriveFL.spin(forward,
                 -error * turnkP - totalError * turnkI - derivative * turnkD,
                 pct);
    DriveFR.spin(forward,
                 error * turnkP + totalError * turnkI + derivative * turnkD,
                 pct);
    cIndex();             // Auto index during turn
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

void AutoFunctions::autoStrafeLeft(double degrees, double iDeg, double fDeg,
                                   double speed) {
  resetDriveEncoders();
  h = IMUL.rotation();
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

void AutoFunctions::autoStrafeRight(double degrees, double iDeg, double fDeg,
                                    double speed) {
  resetDriveEncoders();
  h = IMUL.rotation();
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

void AutoFunctions::intake(double speed) {
  IntakeL.spin(forward, speed, pct);
  IntakeR.spin(forward, speed, pct);
}

void AutoFunctions::openDegrees(double speed, double degrees) {
  IntakeL.rotateFor(-degrees, deg, speed, rpm, false);
  IntakeR.rotateFor(-degrees, deg, speed, rpm, false);
}

void AutoFunctions::openOneIntake(double speed, double degrees, int side) {
  if (side == 1) { //Open right intake
    IntakeR.rotateFor(-degrees, deg, speed, rpm, false);
  }
  if (side == -1) { //Open left intake
    IntakeL.rotateFor(-degrees, deg, speed, rpm, false);
  }
}

void AutoFunctions::autoIntake() {
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
        IntakeL.spin(forward, 100, pct);
        IntakeR.spin(forward, 100, pct);
        leftIntakeTotalError = 0;
        rightIntakeTotalError = 0;
        cIndex();
      }
    }
  }
}

void AutoFunctions::openIntakeDepreciated() {
   // open Left Intake on bottom right bumper
  IntakeL.spin(reverse, 100, pct); // Run left intake
  if (IntakeLineL.value(pct) < 50) {
    IntakeL.stop();
  }
  // Right Intake
  IntakeR.spin(reverse, 100, pct); // Run right intake
  if (IntakeLineR.value(pct) < 50) {
    IntakeR.stop();
  }
}

void AutoFunctions::openIntake() {
  if (!leftIntakeLogic) { //Set a brief period of time in which effficiency is ignored
    openTL = Brain.timer(sec) + openTime;
    leftIntakeLogic = true;
  }
  if (!rightIntakeLogic) {
    openTR = Brain.timer(sec) + openTime;
    rightIntakeLogic = true;
  }
    if (Brain.timer(sec) < openTL || (IntakeL.efficiency() > 0 && !leftIntakeOpen)) {
      IntakeL.spin(reverse, 100, pct);
      Brain.Screen.drawRectangle(440, 220, 40, 20, green);
    } else {
      leftIntakeOpen = true;
      IntakeL.stop(hold);
      Brain.Screen.drawRectangle(440, 220, 40, 20, red);
    }
    if (Brain.timer(sec) < openTR || (IntakeR.efficiency() > 0 && !rightIntakeOpen)) {
      IntakeR.spin(reverse, 100, pct);
      Brain.Screen.drawRectangle(440, 0, 40, 20, green);
    } else {
      rightIntakeOpen = true;
      IntakeR.stop(hold);
      Brain.Screen.drawRectangle(440, 0, 40, 20, red);
    }
    Brain.Screen.setCursor(10, 10);
    Brain.Screen.print(IntakeL.efficiency(pct));
}

void AutoFunctions::openIntakeTo() {
  while (!(IntakeLineL.value(pct) < 50) || !(IntakeLineR.value(pct) < 50)) {
     // open Left Intake on bottom right bumper
  IntakeL.spin(reverse, 100, pct); // Run left intake
  if (IntakeLineL.value(pct) < 50) {
    IntakeL.stop();
  }
  // Right Intake
  IntakeR.spin(reverse, 100, pct); // Run right intake
  if (IntakeLineR.value(pct) < 50) {
    IntakeR.stop();
  }
    wait(loopTime, msec);
  }
}

void AutoFunctions::intakeBrake() {
  IntakeL.stop(hold);
  IntakeR.stop(hold);
}

void AutoFunctions::index(double speed) {
  IndexerTop.spin(forward, speed, pct);
  IndexerLow.spin(forward, speed, pct);
}

void AutoFunctions::pIndex(double speed, double degrees) {
  IndexerTop.resetPosition();
  IndexerLow.resetPosition();

  while (fabs(IndexerLow.position(vex::degrees)) < degrees) {
    IndexerTop.spin(forward, speed, pct);
    IndexerLow.spin(forward, speed, pct);

    wait(loopTime, msec);
  }
}

void AutoFunctions::outdex(double speed) {
  IndexerTop.spin(reverse, speed, pct);
  IndexerLow.spin(reverse, speed, pct);
}

void AutoFunctions::pOutdex(double speed, double degrees) {
  IndexerTop.resetPosition();
  IndexerLow.resetPosition();

  while (fabs(IndexerLow.position(vex::degrees)) < degrees) {
    IndexerTop.spin(reverse, speed, pct);
    IndexerLow.spin(reverse, speed, pct);

    wait(loopTime, msec);
  }
}

void AutoFunctions::indexerBrake() {
  IndexerTop.stop(hold);
  IndexerLow.stop(hold);
}

void AutoFunctions::cIndex() {
  indexerBrake();
  indexSense();
  if (!position3) {
    IndexerTop.spin(forward, 35, pct);
    IndexerLow.spin(forward, 100, pct);
  }
  if (position3 && !position2) {
    IndexerLow.spin(forward, 70, pct);
  }
}

void AutoFunctions::indexSense() {
  if (LinePosition2.value(pct) < 30 && LinePosition2.value(pct) > 1) { // Position 2
    position2 = true;
    Brain.Screen.drawCircle(200, 100, 50, green);
  } else {
    position2 = false;
    Brain.Screen.drawCircle(200, 100, 50, black);
  }

  if (LinePosition3.objectDistance(mm) < 20) { // Position 3
    position3 = true;
    Brain.Screen.drawCircle(100, 100, 50, green);
  } else {
    position3 = false;
    Brain.Screen.drawCircle(100, 100, 50, black);
  }
}

void AutoFunctions::shoot() {
  double rotTo = IndexerTop.position(deg) + 400;
  while (IndexerTop.position(deg) < rotTo) {
    IndexerTop.spin(forward, 100, pct);
    //REMOVE AFTER TOURNAMENT
    IndexerLow.spin(forward, 100, pct);
    wait(10, msec);
  }
  indexerBrake();
}

void AutoFunctions::doubleShot() {
  double indexRotation = IndexerTop.position(degrees);
  while (IndexerTop.position(deg) < indexRotation + 800) {
    IndexerTop.spin(forward, 100, pct);
    if (IndexerTop.position(degrees) > indexRotation + 200 && IndexerTop.position(degrees) < indexRotation + 600) {
      IndexerLow.spin(forward, 80, pct);
    } else {
      IndexerLow.stop(hold);
    }
    wait(10, msec);
  }
  indexerBrake();
}

// Params
// Aligns on a corner goal by turning to the right,
// pivots around front right wheel, coasts back right wheel,
// drives both left wheels
// speed: speed at which the left motors spin
// degreees: where the robot will face
void AutoFunctions::alignTurnRight(double speed, double degrees) {
  resetDriveEncoders();
  DriveFR.stop(hold);  // Pivot on front right wheel
  DriveBR.stop(coast); // Coast on back right wheel

  // Run loop when heading is less than degrees
  // heading increases as robot turns right
  while (IMUL.rotation() < degrees) {
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
void AutoFunctions::alignTurnLeft(double speed, double degrees) {
  resetDriveEncoders();
  DriveFL.stop(hold);  // Pivot on front right wheel
  DriveBL.stop(coast); // Coast on back right wheel

  // Run loop when heading is greater than degrees
  // heading decreases (and goes negative) as robot turns left
  while (IMUL.rotation() > degrees) {
    DriveFR.spin(forward, speed, pct);
    DriveBR.spin(forward, speed, pct);
    wait(10, msec);
  }
  resetPID();
  brakeDrive();
}

void AutoFunctions::flipout() {
  IndexerLow.rotateFor(30, deg, 200, rpm, true);
}