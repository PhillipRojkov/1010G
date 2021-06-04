#include "DriveClass.h"

void DriveClass::runTankMecanumBase() {
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
    DriveFR.spin(vex::directionType::rev, Controller1.Axis2.value(),
                 vex::velocityUnits::pct);
  } else {
    DriveFR.stop(brake);
    DriveBR.stop(brake);
  }
  // Left bumper strafe left
  if (Controller1.ButtonL1.pressing()) {
    DriveBL.spin(directionType::fwd,
                 strafeSpeed * (2 - strafeWeighting) +
                     Controller1.Axis3.value() * strafeStickMultiplier,
                 velocityUnits::pct);
    DriveFL.spin(directionType::rev,
                 strafeSpeed * strafeWeighting -
                     Controller1.Axis3.value() * strafeStickMultiplier,
                 velocityUnits::pct);
    DriveBR.spin(directionType::rev,
                 strafeSpeed * (2 - strafeWeighting) -
                     Controller1.Axis2.value() * strafeStickMultiplier,
                 velocityUnits::pct);
    DriveFR.spin(directionType::fwd,
                 strafeSpeed * strafeWeighting +
                     Controller1.Axis2.value() * strafeStickMultiplier,
                 velocityUnits::pct);
  }
  // Right bumper strafe right
  if (Controller1.ButtonR1.pressing()) {
    DriveBL.spin(directionType::rev,
                 strafeSpeed * (2 - strafeWeighting) -
                     Controller1.Axis3.value() * strafeStickMultiplier,
                 velocityUnits::pct);
    DriveFL.spin(directionType::fwd,
                 strafeSpeed * strafeWeighting +
                     Controller1.Axis3.value() * strafeStickMultiplier,
                 velocityUnits::pct);
    DriveBR.spin(directionType::fwd,
                 strafeSpeed * (2 - strafeWeighting) +
                     Controller1.Axis2.value() * strafeStickMultiplier,
                 velocityUnits::pct);
    DriveFR.spin(directionType::fwd,
                 strafeSpeed * strafeWeighting -
                     Controller1.Axis2.value() * strafeStickMultiplier,
                 velocityUnits::pct);
  }
}

void DriveClass::runArcadeMecanumBase() {
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

void DriveClass::runTankBase() {
  // Left stick
  if (abs(Controller1.Axis3.value()) > controllerDeadZone) {
    DriveFL.spin(fwd, Controller1.Axis3.value(), pct);
    DriveBL.spin(fwd, Controller1.Axis3.value(), pct);
  } else {
    DriveFL.stop(brake);
    DriveBL.stop(brake);
  }
  // Right stick
  if (abs(Controller1.Axis2.value()) > controllerDeadZone) {
    DriveBR.spin(fwd, Controller1.Axis2.value(), pct);
    DriveFR.spin(fwd, Controller1.Axis2.value(), pct);
  } else {
    DriveFR.stop(brake);
    DriveBR.stop(brake);
  }
}

void DriveClass::runArcadeBase(bool dualStick) {
  if (dualStick) { //Forward/backward on right stick, turn on left stick
    DriveFL.spin(forward, Controller1.Axis2.value() + Controller1.Axis4.value(), vex::pct);
    DriveBL.spin(forward, Controller1.Axis2.value() + Controller1.Axis4.value(), vex::pct);
    DriveFR.spin(forward, Controller1.Axis2.value() - Controller1.Axis4.value(), vex::pct);
    DriveBR.spin(forward, Controller1.Axis2.value() - Controller1.Axis4.value(), vex::pct);
  } else { //All movement on right stick
    DriveFL.spin(forward, Controller1.Axis2.value() + Controller1.Axis1.value(), vex::pct);
    DriveBL.spin(forward, Controller1.Axis2.value() + Controller1.Axis1.value(), vex::pct);
    DriveFR.spin(forward, Controller1.Axis2.value() - Controller1.Axis1.value(), vex::pct);
    DriveBR.spin(forward, Controller1.Axis2.value() - Controller1.Axis1.value(), vex::pct);
  }
}

void DriveClass::index() {
  // Simple indexer up on partner top left bumper
  if (Controller2.ButtonL1.pressing()) {
    IndexerTop.spin(forward, 100, vex::pct);
    IndexerLow.spin(forward, 80, vex::pct);
    enableIndex = false; // disable auto indexing
  } else if (Controller2.ButtonL2
                 .pressing()) { // Simple indexer down on partner bottom left bumper
    IndexerTop.spin(reverse, 100, vex::pct);
    IndexerLow.spin(reverse, 100, vex::pct);
    enableIndex = false; // disable auto indexing
  } else { //Auto index
    enableIndex = true;
    cIndex();
  }
}

void DriveClass::cIndex() { // Automatic index
  if (enableIndex) {
    if (!position3) { // Always try to fill position 3
      IndexerTop.spin(forward, 35, pct);
      IndexerLow.spin(forward, 100, pct);
    }
    if (position3 && !position2) { // If position 3 is filled, fill position2
      IndexerLow.spin(forward, 70, pct);
      IndexerTop.stop(hold);
    }
    if (position3 && position2) { // Hold balls when both position 2 and 3 are filled
      IndexerLow.stop(hold);
      IndexerTop.stop(hold);
    }
  }
}

void DriveClass::openIntake() {
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
  enableIndex = false;
}

void DriveClass::newIntake() {
  if (Controller2.ButtonR1.pressing()) { // Intake on partner top right bumper
    IntakeL.spin(forward, 100, vex::pct);
    IntakeR.spin(forward, 100, vex::pct);
    enableIndex = true; //Auto index
  } else if (Controller2.ButtonR2.pressing()) { // Open on partner bottom right bumper
    IntakeL.spin(reverse, 100, pct);
    IntakeR.spin(reverse, 100, pct);
  } else if (Controller1.ButtonL1.pressing()) { // Slow outtake on main top right bumper
    IntakeL.spin(reverse, 20, pct);
    IntakeR.spin(reverse, 20, pct);
  }
   else {
    IntakeL.stop(hold); //Hold the intakes in the open position
    IntakeR.stop(hold);
  }
}

void DriveClass::intake() {
  if (Controller2.ButtonR1.pressing()) { // Intake on partner top right bumper
    IntakeL.spin(forward, 100, vex::pct);
    IntakeR.spin(forward, 100, vex::pct);
    leftIntakeTotalError = 0; //Reset opening PID values
    rightIntakeTotalError = 0;
    enableIndex = true; //Auto index
    doIntake = false; //Disable auto intake
    leftIntakeOpen = false;
    rightIntakeOpen = false;
    leftIntakeLogic = false;
    rightIntakeLogic = false;
  } else if (Controller2.ButtonR2.pressing()) { // Open on partner bottom right bumper
    openIntake();
    doIntake = false; //Disable auto intake
  } else if (Controller2.ButtonUp.pressing()) { // Auto intake on partner up button
    intakeSense();
    leftIntakeOpen = false;
    rightIntakeOpen = false;
    leftIntakeLogic = false;
    rightIntakeLogic = false;
  } else {
    leftIntakeTotalError = 0; //Reset opening PID values
    rightIntakeTotalError = 0;
    IntakeL.stop(hold); //Hold the intakes in the open position
    IntakeR.stop(hold);
    doIntake = false; //Disable auto intake
    leftIntakeOpen = false;
    rightIntakeOpen = false;
    leftIntakeLogic = false;
    rightIntakeLogic = false;
  }
}

void DriveClass::indexSense() {
  if (LinePosition2.value(pct) < 60 && LinePosition2.value(pct) > 1) { // Position 2
    position2 = true;
    //Brain.Screen.drawCircle(200, 100, 50, green); // Vertical screen
    Brain.Screen.drawCircle(240, 208, 40, green); // Horizontal screen
  } else {
    position2 = false;
    //Brain.Screen.drawCircle(200, 100, 50, black); // Vertical screen
    Brain.Screen.drawCircle(240, 208, 40, black); // Horizontal screen
  }

  if (LinePosition3.isObjectDetected() && LinePosition3.objectDistance(mm) < 45) { // Position 3
    position3 = true;
    //Brain.Screen.drawCircle(100, 100, 50, green); // Vertical screen
    Brain.Screen.drawCircle(240, 106, 40, green); // Horizontal screen
  } else {
    position3 = false;
    //Brain.Screen.drawCircle(100, 100, 50, black); // Vertical screen
    Brain.Screen.drawCircle(240, 106, 40, black); // Horizontal screen
  }
}

void DriveClass::intakeSense() { //Auto intake
  // Red code
  for (int i = 0; i < 2; i++) {
    if (i == 0) {
      VisionSensor.takeSnapshot(SIG_2); // Red signature
    } else {
      VisionSensor.takeSnapshot(SIG_1); // Blue signature
    }
    if (VisionSensor.largestObject.exists &&
        VisionSensor.largestObject.width >
            100) { // If the object exists and is relatively close
      if (VisionSensor.largestObject.centerY >
          190) { // If the object is near, close until position1 is clicked
        doIntake = true;
        enableIndex = true;
      } else if (VisionSensor.largestObject.centerY > 80 &&
                 VisionSensor.largestObject.centerY <=
                     190) { // If the object is far, open intake
        openIntake();
        doIntake = false;
      }
      if (doIntake &&
          position1) { // Stop intaking once the ball enters position 1
        doIntake = false;
      }
      if (doIntake) { // Run the intakes if doIntake is true
        IntakeL.spin(forward, 100, vex::pct);
        IntakeR.spin(forward, 100, vex::pct);
        leftIntakeTotalError = 0;
        rightIntakeTotalError = 0;
        enableIndex = true;
      }
    }
  }
}

void DriveClass::score() {
  // Score 1
  if (Controller1.ButtonL2.pressing() && scoreNum == 0) {
    scoreNum = 1;
    indexRotation = IndexerTop.position(degrees);
  } else if (Controller1.ButtonR2.pressing() && scoreNum == 0) {
    scoreNum = 2;
    indexRotation = IndexerTop.position(degrees);
  }

  if (scoreNum == 1) { //Single shot
    if (IndexerTop.position(degrees) < indexRotation + 300) {
      IndexerTop.spin(forward, 420, pct);
      IndexerLow.stop(hold);
      enableIndex = false;
    } else {
      enableIndex = true;
      scoreNum--;
    }
  }

  if (scoreNum == 2) { //Double shot
    if (IndexerTop.position(degrees) < indexRotation + 300 + 450) {
      IndexerTop.spin(forward, 420, pct);
      enableIndex = false;
    } else {
      scoreNum = 0;
      enableIndex = true;
    }
    if (IndexerTop.position(degrees) < indexRotation + 150) {
      IndexerLow.stop(hold);
    } else {
      IndexerLow.spin(forward, 80, pct);
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