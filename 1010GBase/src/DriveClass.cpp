#include "DriveClass.h"

DriveClass dC;

void DriveClass::runTankBase() { // Linear tank drive with mecanum on bumpers
  // Left stick
  if (abs(Controller1.Axis3.value()) > dC.controllerDeadZone) {
    DriveFL.spin(vex::directionType::fwd, Controller1.Axis3.value(),
                 vex::velocityUnits::pct);
    DriveBL.spin(vex::directionType::fwd, Controller1.Axis3.value(),
                 vex::velocityUnits::pct);
  } else {
    DriveFL.stop(brake);
    DriveBL.stop(brake);
  }
  // Right stick
  if (abs(Controller1.Axis2.value()) > dC.controllerDeadZone) {
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
                 dC.strafeSpeed +
                     Controller1.Axis3.value() * dC.strafeStickMultiplier,
                 velocityUnits::pct);
    DriveFL.spin(directionType::rev,
                 dC.strafeSpeed * dC.frontStrafeSpeedMultiplier -
                     Controller1.Axis3.value() * dC.strafeStickMultiplier,
                 velocityUnits::pct);
    DriveBR.spin(directionType::rev,
                 dC.strafeSpeed -
                     Controller1.Axis2.value() * dC.strafeStickMultiplier,
                 velocityUnits::pct);
    DriveFR.spin(directionType::fwd,
                 dC.strafeSpeed * dC.frontStrafeSpeedMultiplier +
                     Controller1.Axis2.value() * dC.strafeStickMultiplier,
                 velocityUnits::pct);
  }
  // Right bumper strafe right
  if (Controller1.ButtonR1.pressing()) {
    DriveBL.spin(directionType::rev,
                 dC.strafeSpeed -
                     Controller1.Axis3.value() * dC.strafeStickMultiplier,
                 velocityUnits::pct);
    DriveFL.spin(directionType::fwd,
                 dC.strafeSpeed * dC.frontStrafeSpeedMultiplier +
                     Controller1.Axis3.value() * dC.strafeStickMultiplier,
                 velocityUnits::pct);
    DriveBR.spin(directionType::fwd,
                 dC.strafeSpeed +
                     Controller1.Axis2.value() * dC.strafeStickMultiplier,
                 velocityUnits::pct);
    DriveFR.spin(directionType::rev,
                 dC.strafeSpeed * dC.frontStrafeSpeedMultiplier -
                     Controller1.Axis2.value() * dC.strafeStickMultiplier,
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
    IndexerTop.spin(forward, 127, vex::pct);
    IndexerLow.spin(forward, 127, vex::pct);
    dC.enableIndex = false; // disable auto indexing
  } else if (Controller2.ButtonL2
                 .pressing()) { // Simple indexer down on bottom left bumper
    IndexerTop.spin(reverse, 127, vex::pct);
    IndexerLow.spin(reverse, 127, vex::pct);
    dC.enableIndex = false; // disable auto indexing
  } else {
    IndexerTop.stop(vex::hold);
    IndexerLow.stop(vex::hold);
  }
}

void DriveClass::cIndex() { // Automatic index
  if (dC.enableIndex) {
    if (!dC.position3) { // Always try to fill position 3
      IndexerTop.spin(forward, 80, pct);
      IndexerLow.spin(forward, 80, pct);
    }
    if (dC.position3 &&
        !dC.position2) { // If position 3 is filled, fill position2
      IndexerLow.spin(forward, 80, pct);
    }
  }
}

void DriveClass::openIntake() {
  // PID open on bottom right
  // bumper Left Intake
  double leftError = -dC.leftPotDesired + potL.angle(deg);
  IntakeL.spin(reverse, 100, pct); //Run left intake
  if (leftError < dC.potRange2) { //inside small range
    dC.leftBrake = true;
  }
  if (leftError < dC.potRange1 && dC.leftBrake) { //inside large range and left brake
    IntakeL.stop(hold);
  }
  if (leftError >= dC.potRange1) { //out of large range
    dC.leftBrake = false;
  }
  // Right Intake
  double rightError = -dC.rightPotDesired + potR.angle(deg);
  IntakeR.spin(reverse, 100, pct); //Run right intake
  if (rightError < dC.potRange2) { //inside small range
    dC.rightBrake = true;
  }
  if (rightError < dC.potRange1 && dC.rightBrake) { //inside large range and rightBrake
    IntakeR.stop(hold);
  }
  if (rightError >= dC.potRange1) { //out of large range
    dC.rightBrake = false;
  }
  dC.enableIndex = false;
}

void DriveClass::intake() {
  // Intake on top right bumper
  if (Controller2.ButtonR1.pressing()) {
    IntakeL.spin(forward, 100, vex::pct);
    IntakeR.spin(forward, 100, vex::pct);
    dC.leftIntakeTotalError = 0;
    dC.rightIntakeTotalError = 0;
    dC.enableIndex = true;
    dC.doIntake = false;
  } else if (Controller2.ButtonR2.pressing()) {
    openIntake();
    dC.doIntake = false;
  } else if (Controller2.ButtonUp.pressing()) { // Auto intake
    intakeSense();
  } else {
    dC.leftIntakeTotalError = 0;
    dC.rightIntakeTotalError = 0;
    IntakeL.stop(hold);
    IntakeR.stop(hold);
    dC.doIntake = false;
  }
}

void DriveClass::indexSense() { // Sets index ball position variables
  if (LinePosition1.pressing()) { //Position 1
    dC.position1 = true;
    Brain.Screen.drawCircle(300, 100, 50, green); //Visualisation
  } else {
    dC.position1 = false;
    Brain.Screen.drawCircle(300, 100, 50, black);
  }

  if (LinePosition2.pressing()) { //Position 2
    dC.position2 = true;
    Brain.Screen.drawCircle(200, 100, 50, green);
  } else {
    dC.position2 = false;
    Brain.Screen.drawCircle(200, 100, 50, black);
  }

  if (LinePosition3.pressing()) { //Position 3
    dC.position3 = true;
    Brain.Screen.drawCircle(100, 100, 50, green);
  } else {
    dC.position3 = false;
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
        dC.doIntake = true;
        dC.enableIndex = true;
      } else if (VisionSensor.largestObject.centerY > 80 &&
                 VisionSensor.largestObject.centerY <= 190) { // If the object is far, open intake
        openIntake();
        dC.doIntake = false;
      }
      if (dC.doIntake && position1) { //Stop intaking once the ball enters position 1
        dC.doIntake = false;
      }
      if (dC.doIntake) { //Run the intakes if doIntake is true
        IntakeL.spin(forward, 100, vex::pct);
        IntakeR.spin(forward, 100, vex::pct);
        dC.leftIntakeTotalError = 0;
        dC.rightIntakeTotalError = 0;
        dC.enableIndex = true;
      }
    }
  }
}

void DriveClass::score() { // Score macro
  // Score 1
  if (Controller1.ButtonL2.pressing() && dC.scoreNum == 0) {
    dC.scoreNum = 1;
    dC.indexRotation = IndexerTop.position(degrees);
  } else if (Controller1.ButtonR2.pressing() && dC.scoreNum == 0) {
    dC.scoreNum = 2;
    dC.indexRotation = IndexerTop.position(degrees);
  }

  if (dC.scoreNum == 1) {
    if (IndexerTop.position(degrees) < dC.indexRotation + 600) {
      IndexerTop.spin(forward, 100, pct);
    } else {
      dC.enableIndex = true;
      dC.scoreNum--;
    }
  }

  if (dC.scoreNum == 2) {
    // Run single shot
    if (IndexerTop.position(degrees) < dC.indexRotation + 600) {
      IndexerTop.spin(forward, 100, pct);
    } else if (!dC.position3) {
      // cIndex until position3
      dC.enableIndex = true;
      cIndex();
    } else if (dC.position3) {
      // Reset indexRotation
      dC.indexRotation = IndexerTop.position(degrees);
      // Run single shot again
      dC.scoreNum--;
    }
  }
}

void DriveClass::resetScoreNum() { dC.scoreNum = 0; }

void DriveClass::checkPosition1() { // Run auto index for some time if position1
                                    // is activated
  if (dC.position1) {
    // cIndex for timeToIndex seconds
    dC.t = Brain.timer(seconds) + dC.timeToIndex;
  }
  if (Brain.timer(seconds) < dC.t) {
    dC.enableIndex = true;
  }
}