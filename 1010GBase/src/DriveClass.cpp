#include "vex.h"

using namespace vex;

class DriveClass { // Holds all functions used for user control
  // Drive parameters
  int controllerDeadZone = 10;   // Stick dead zone (stick values from 0 - 100)
  double strafeMultiplier = 0.6; // Strafe slowdown multiplier

  //pot parameters
  int leftPotMax = 220; //pot values range from 0 to leftPotMax
  //When the pot value is greater than leftPotMax, intakeL is out
  int rightPotMin = 190; //pot values range from 360 to rightPotMin
  //When the pot value is less than rightPotMax, intakeR is out

  // Indexer parameters
  bool position1;
  bool position2;
  bool position3;

  int scoreNum = 0;

  int indexRotation;

  double timeToIndex = 1.2;
  double t = 0;

public:
  bool enableIndex = false;

  void runTankBase() { // Linear tank drive
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
                   80 + Controller1.Axis3.value() * strafeMultiplier,
                   velocityUnits::pct);
      DriveFL.spin(directionType::rev,
                   80 - Controller1.Axis3.value() * strafeMultiplier,
                   velocityUnits::pct);
      DriveBR.spin(directionType::rev,
                   80 - Controller1.Axis2.value() * strafeMultiplier,
                   velocityUnits::pct);
      DriveFR.spin(directionType::fwd,
                   80 + Controller1.Axis2.value() * strafeMultiplier,
                   velocityUnits::pct);
    }
    // Right bumper strafe right
    if (Controller1.ButtonR1.pressing()) {
      DriveBL.spin(directionType::rev,
                   80 - Controller1.Axis3.value() * strafeMultiplier,
                   velocityUnits::pct);
      DriveFL.spin(directionType::fwd,
                   80 + Controller1.Axis3.value() * strafeMultiplier,
                   velocityUnits::pct);
      DriveBR.spin(directionType::fwd,
                   80 + Controller1.Axis2.value() * strafeMultiplier,
                   velocityUnits::pct);
      DriveFR.spin(directionType::rev,
                   80 - Controller1.Axis2.value() * strafeMultiplier,
                   velocityUnits::pct);
    }
  }

  void runArcadeBase() { // Linear mecanum drive in arcade style
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

  void index() {
    // Simple indexer up on top left bumper
    if (Controller2.ButtonL1.pressing()) {
      IndexerTop.spin(forward, 127, vex::pct);
      IndexerLow.spin(forward, 127, vex::pct);
      enableIndex = false;
    } else if (Controller2.ButtonL2
                   .pressing()) { // Simple indexer down on bottom left bumper
      IndexerTop.spin(reverse, 127, vex::pct);
      IndexerLow.spin(reverse, 127, vex::pct);
      enableIndex = false;
    } else {
      IndexerTop.stop(vex::hold);
      IndexerLow.stop(vex::hold);
    }
  }

  void cIndex() {  
    if (enableIndex) {
      if (!position3) {
        IndexerTop.spin(forward, 80, pct);
        IndexerLow.spin(forward, 80, pct);
      }
      if (position3 && !position2) {
        IndexerLow.spin(forward, 80, pct);
      }
    }
  }

  void intake() {
    // Intake on top right bumper
    if (Controller2.ButtonR1.pressing()) {
      IntakeL.spin(forward, 100, vex::pct);
      IntakeR.spin(forward, 100, vex::pct);
      enableIndex = true;
    } else if (Controller2.ButtonR2.pressing()) { // Simple outtake/open on bottom right bumper
      if (potL.angle(deg) < leftPotMax) { //If the leftPot value < leftPotMax, outtake leftIntake
        IntakeL.spin(reverse, 100, vex::pct);
      } else { //Stop the intake once leftPotValue >= leftPotMax
        IntakeL.stop(hold);
      }

      if (potR.angle(deg) > rightPotMin) { //If the leftPot value < leftPotMax, outtake leftIntake
        IntakeR.spin(reverse, 100, vex::pct);
      } else { //Stop the intake once leftPotValue >= leftPotMax
        IntakeR.stop(hold);
      }
      enableIndex = false;
    } else {
      IntakeL.stop(hold);
      IntakeR.stop(hold);
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

  void score() {
    //Score 1
    if (Controller2.ButtonX.pressing() && scoreNum == 0) {
      scoreNum = 1;
      indexRotation = IndexerTop.position(degrees);
    } else if (Controller2.ButtonY.pressing() && scoreNum == 0) {
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
     //Run single shot
     if (IndexerTop.position(degrees) < indexRotation + 600) {
        IndexerTop.spin(forward, 100, pct);
      } else if (!position3){
        //cIndex until position3
        enableIndex = true;
        cIndex();
      } else if (position3) {
        //Reset indexRotation
        indexRotation = IndexerTop.position(degrees);
        //Run single shot again
        scoreNum--;
      }
    }
  }

  void resetScoreNum() {
    scoreNum = 0;
  }

  void checkPosition1() {
    if (position1) {
      //cIndex for 3 seconds
      t = Brain.timer(seconds) + timeToIndex;
    }
    if (Brain.timer(seconds) < t) {
      enableIndex = true;
    }
  }
};