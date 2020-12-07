#include "vex.h"

using namespace vex;

class DriveClass { // Holds all functions used for user control
  // Drive parameters
  int controllerDeadZone = 10;   // Stick dead zone (stick values from 0 - 100)
  double strafeMultiplier = 0.6; // Strafe slowdown multiplier

  // Indexer parameters
  bool position1;
  bool position2;
  bool position3;
  int linePos1Pct = 71;
  int linePos2Pct = 69;
  bool goingTo3 = false;

  int scoreNum = 0;
  bool scoring = false;

  int indexRotation;

  double st;

  bool i;

public:
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
      IndexerL.spin(forward, 127, vex::pct);
      IndexerR.spin(forward, 127, vex::pct);
    } else if (Controller2.ButtonL2
                   .pressing()) { // Simple indexer down on bottom left bumper
      IndexerL.spin(reverse, 127, vex::pct);
      IndexerR.spin(reverse, 127, vex::pct);
    } else {
      IndexerL.stop(vex::hold);
      IndexerR.stop(vex::hold);
    }
  }

  void cIndex() {
    if (Controller2.ButtonA.pressing()) {
      if (!position2) {
        IndexerL.spin(forward, 80, pct);
        IndexerR.spin(forward, 80, pct);
      } else if (position2 && !goingTo3) {
          IndexerL.stop(hold);
          IndexerR.stop(hold);
        }
      if (position3 && !position2) {
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
      if (position1 && !position2) {
        IndexerL.spin(forward, 80, pct);
        IndexerR.spin(forward, 80, pct);
      }
    }
  }

  void intake() {
    // Intake on top right bumper
    if (Controller2.ButtonR1.pressing()) {
      IntakeL.spin(forward, 100, vex::pct);
      IntakeR.spin(forward, 100, vex::pct);
    } else if (Controller2.ButtonR2.pressing()) { // Simple outtake on bottom right bumper
      IntakeL.spin(reverse, 100, vex::pct);
      IntakeR.spin(reverse, 100, vex::pct);
    } else {
      IntakeL.stop(vex::hold);
      IntakeR.stop(vex::hold);
    }
  }

  void indexSense() { // Sets index ball position variables
    if (LinePosition1.value(pct) <= linePos1Pct) {
      position1 = true;
      Brain.Screen.drawCircle(300, 100, 50, orange);
    } else {
      position1 = false;
      Brain.Screen.drawCircle(300, 100, 50, purple);
    }

    if (LinePosition2.value(pct) <= linePos2Pct) {
      position2 = true;
      Brain.Screen.drawCircle(200, 100, 50, orange);
    } else {
      position2 = false;
      Brain.Screen.drawCircle(200, 100, 50, purple);
    }

    if (LinePosition3.pressing()) {
      position3 = true;
      Brain.Screen.drawCircle(100, 100, 50, orange);
    } else {
      position3 = false;
      Brain.Screen.drawCircle(100, 100, 50, purple);
    }
  }

  void score() {
    //Score 1
    if (Controller2.ButtonX.pressing() && !scoring) {
      scoreNum = 1;
      scoring = true;
      indexRotation = IndexerL.position(degrees);
    } else if (Controller2.ButtonY.pressing() && !scoring) {
      scoring = true;
      scoreNum = 2;
      indexRotation = IndexerL.position(degrees);
      i = true;
    } else if (Controller2.ButtonB.pressing() && !scoring) {
      scoring = true;
      scoreNum = 3;
      indexRotation = IndexerL.position(degrees);
    }

    if (scoreNum == 1) {
      if (IndexerL.position(degrees) < indexRotation + 600) {
        IndexerL.spin(forward, 100, pct);
        IndexerR.spin(forward, 100, pct);
      } else {
        scoreNum = 0;
        scoring = false;
      }
    }

    if (scoreNum == 2) {
      if (IndexerL.position(degrees) < indexRotation + 800 && i) {
        IndexerL.spin(forward, 100, pct);
        IndexerR.spin(forward, 100, pct);
        st = Brain.timer(sec);
      } else {
        i = false;
      } 
      if (IndexerL.position(degrees) < indexRotation + 1400 && !i) {
        if (Brain.timer(sec) > st + 0.2) {
          IndexerL.spin(forward, 100, pct);
          IndexerR.spin(forward, 100, pct);
        }
      }
      if (IndexerL.position(degrees) >= indexRotation + 1400) {
        scoreNum = 0;
        scoring = false;
      }
    }

    if (scoreNum == 3) {
      if (IndexerL.position(degrees) < indexRotation + 900) {
        IndexerL.spin(forward, 100, pct);
        IndexerR.spin(forward, 100, pct);
      } else {
        scoreNum = 0;
        scoring = false;
      }
    }
  }
};