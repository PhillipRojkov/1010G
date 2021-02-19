#include "DriveClass.h"

void DriveClass::runTankBase() { // Linear tank drive with mecanum on bumpers
  // Left stick
  if (abs(Controller1.Axis3.value()) > controllerDeadZone) {
    DriveFL.spin(vex::directionType::fwd, Controller1.Axis3.value(),
                 vex::velocityUnits::pct);
    DriveBL.spin(vex::directionType::fwd, Controller1.Axis3.value(),
                 vex::velocityUnits::pct);
  } else {
    DriveFL.stop(brake);
    DriveBL.stop(brake);
  }
  // Right stick
  if (abs(Controller1.Axis2.value()) > controllerDeadZone) {
    DriveBR.spin(vex::directionType::fwd, Controller1.Axis2.value(),
                 vex::velocityUnits::pct);
    DriveFR.spin(vex::directionType::fwd, Controller1.Axis2.value(),
                 vex::velocityUnits::pct);
  } else {
    DriveFR.stop(brake);
    DriveBR.stop(brake);
  }
  // Left bumper strafe left
  if (Controller1.ButtonL1.pressing()) {
    DriveBL.spin(directionType::fwd,
                 strafeSpeed +
                     Controller1.Axis3.value() * strafeStickMultiplier,
                 velocityUnits::pct);
    DriveFL.spin(directionType::rev,
                 strafeSpeed * frontStrafeSpeedMultiplier -
                     Controller1.Axis3.value() * strafeStickMultiplier,
                 velocityUnits::pct);
    DriveBR.spin(directionType::rev,
                 strafeSpeed -
                     Controller1.Axis2.value() * strafeStickMultiplier,
                 velocityUnits::pct);
    DriveFR.spin(directionType::fwd,
                 strafeSpeed * frontStrafeSpeedMultiplier +
                     Controller1.Axis2.value() * strafeStickMultiplier,
                 velocityUnits::pct);
  }
  // Right bumper strafe right
  if (Controller1.ButtonR1.pressing()) {
    DriveBL.spin(directionType::rev,
                 strafeSpeed -
                     Controller1.Axis3.value() * strafeStickMultiplier,
                 velocityUnits::pct);
    DriveFL.spin(directionType::fwd,
                 strafeSpeed * frontStrafeSpeedMultiplier +
                     Controller1.Axis3.value() * strafeStickMultiplier,
                 velocityUnits::pct);
    DriveBR.spin(directionType::fwd,
                 strafeSpeed +
                     Controller1.Axis2.value() * strafeStickMultiplier,
                 velocityUnits::pct);
    DriveFR.spin(directionType::rev,
                 strafeSpeed * frontStrafeSpeedMultiplier -
                     Controller1.Axis2.value() * strafeStickMultiplier,
                 velocityUnits::pct);
  }
}

void DriveClass::runArcadeBase() { // Linear mecanum drive in arcade style
  DriveFL.spin(forward,
               Controller1.Axis2.value() + Controller1.Axis1.value() +
                   Controller1.Axis4.value(),
               vex::pct);
  DriveFR.spin(forward,
               Controller1.Axis2.value() - Controller1.Axis1.value() -
                   Controller1.Axis4.value(),
               vex::pct);
  DriveBL.spin(forward,
               Controller1.Axis2.value() + Controller1.Axis1.value() -
                   Controller1.Axis4.value(),
               vex::pct);
  DriveBR.spin(forward,
               Controller1.Axis2.value() - Controller1.Axis1.value() +
                   Controller1.Axis4.value(),
               vex::pct);
}

void DriveClass::index() { // Indexer override
  // Simple indexer up on top left bumper
  if (Controller2.ButtonL1.pressing()) {
    IndexerTop.spin(forward, 100, vex::pct);
    IndexerLow.spin(forward, 80, vex::pct);
    enableIndex = false; // disable auto indexing
  } else if (Controller2.ButtonL2
                 .pressing()) { // Simple indexer down on bottom left bumper
    IndexerTop.spin(reverse, 100, vex::pct);
    IndexerLow.spin(reverse, 100, vex::pct);
    enableIndex = false; // disable auto indexing
  } else {
    IndexerTop.stop(vex::hold);
    IndexerLow.stop(vex::hold);
  }
}

void DriveClass::cIndex() { // Automatic index
  if (enableIndex) {
    if (!position3) { // Always try to fill position 3
      IndexerTop.spin(forward, 40, pct);
      IndexerLow.spin(forward, 40, pct);
    }
    if (position3 &&
        !position2) { // If position 3 is filled, fill position2
      IndexerLow.spin(forward, 40, pct);
    }
  }
}

void DriveClass::openIntake() {
  // PID open on bottom right
  // bumper Left Intake
  double leftError = -leftPotDesired + potL.angle(deg);
  IntakeL.spin(reverse, 100, pct); //Run left intake
  if (leftError < potRange2) { //inside small range
    leftBrake = true;
  }
  if (leftError < potRange1 && leftBrake) { //inside large range and left brake
    IntakeL.stop(hold);
  }
  if (leftError >= potRange1) { //out of large range
    leftBrake = false;
  }
  // Right Intake
  double rightError = -rightPotDesired + potR.angle(deg);
  IntakeR.spin(reverse, 100, pct); //Run right intake
  if (rightError < potRange2) { //inside small range
    rightBrake = true;
  }
  if (rightError < potRange1 && rightBrake) { //inside large range and rightBrake
    IntakeR.stop(hold);
  }
  if (rightError >= potRange1) { //out of large range
    rightBrake = false;
  }
  enableIndex = false;
}

void DriveClass::intake() {
  // Intake on top right bumper
  if (Controller2.ButtonR1.pressing()) {
    IntakeL.spin(forward, 100, vex::pct);
    IntakeR.spin(forward, 100, vex::pct);
    leftIntakeTotalError = 0;
    rightIntakeTotalError = 0;
    enableIndex = true;
    doIntake = false;
  } else if (Controller2.ButtonR2.pressing()) {
    openIntake();
    doIntake = false;
  } else if (Controller2.ButtonUp.pressing()) { // Auto intake
    intakeSense();
  } else {
    leftIntakeTotalError = 0;
    rightIntakeTotalError = 0;
    IntakeL.stop(hold);
    IntakeR.stop(hold);
    doIntake = false;
  }
}

void DriveClass::indexSense() { // Sets index ball position variables
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

  if (LinePosition3L.value(pct) < 67 || LinePosition3T.value(pct) < 67) { //Position 3
    position3 = true;
    Brain.Screen.drawCircle(100, 100, 50, green);
  } else {
    position3 = false;
    Brain.Screen.drawCircle(100, 100, 50, black);
  }
}

void DriveClass::intakeSense() {
  // Red code
  for (int i = 0; i < 2; i++) {
    if (i == 0) {
      VisionSensor.takeSnapshot(SIG_2); //Red signature
    } else {
      VisionSensor.takeSnapshot(SIG_1); //Blue signature
    }
    if (VisionSensor.largestObject.exists &&
        VisionSensor.largestObject.width > 100) { //If the object exists and is relatively close
      if (VisionSensor.largestObject.centerY >
          190) { // If the object is near, close until position1 is clicked
        doIntake = true;
        enableIndex = true;
      } else if (VisionSensor.largestObject.centerY > 80 &&
                 VisionSensor.largestObject.centerY <= 190) { // If the object is far, open intake
        openIntake();
        doIntake = false;
      }
      if (doIntake && position1) { //Stop intaking once the ball enters position 1
        doIntake = false;
      }
      if (doIntake) { //Run the intakes if doIntake is true
        IntakeL.spin(forward, 100, vex::pct);
        IntakeR.spin(forward, 100, vex::pct);
        leftIntakeTotalError = 0;
        rightIntakeTotalError = 0;
        enableIndex = true;
      }
    }
  }
}

void DriveClass::score() { // Score macro
  // Score 1
  if (Controller1.ButtonL2.pressing() && scoreNum == 0) {
    scoreNum = 1;
    indexRotation = IndexerTop.position(degrees);
  } else if (Controller1.ButtonR2.pressing() && scoreNum == 0) {
    scoreNum = 2;
    indexRotation = IndexerTop.position(degrees);
  }

  if (scoreNum == 1) {
    if (IndexerTop.position(degrees) < indexRotation + 600) {
      IndexerTop.spin(forward, 100, pct);
    } else {
      enableIndex = true;
      scoreNum--;
    }
  }

  if (scoreNum == 2) {
    // Run single shot
    if (IndexerTop.position(degrees) < indexRotation + 600) {
      IndexerTop.spin(forward, 100, pct);
    } else if (!position3) {
      // cIndex until position3
      enableIndex = true;
      cIndex();
    } else if (position3) {
      // Reset indexRotation
      indexRotation = IndexerTop.position(degrees);
      // Run single shot again
      scoreNum--;
    }
  }
}

void DriveClass::resetScoreNum() { scoreNum = 0; }

void DriveClass::checkPosition1() { // Run auto index for some time if position1
                                    // is activated
  if (position1) {
    // cIndex for timeToIndex seconds
    t = Brain.timer(seconds) + timeToIndex;
  }
  if (Brain.timer(seconds) < t) {
    enableIndex = true;
  }
}