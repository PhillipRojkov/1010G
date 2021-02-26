#include "AutoMasters.h"

void AutoMasters::runFlipout() {
  autoFunctions.flipout();
}

void AutoMasters::newSkillsNew() {
  // Flipout
  autoFunctions.flipout();
  autoFunctions.openIntakeTo();
  wait(700, msec);
  autoFunctions.dumbForward(200, 100, 100, 40);
  autoFunctions.intake(100);
  autoFunctions.dumbForward(100, 50, 50, 50);
  // Goal 1
  autoFunctions.autoForward(350, 100, 100, 90);
  autoFunctions.autoTurnTo(-135);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(800, 100, 100, 90);
  autoFunctions.shoot();
  // Goal 2
  autoFunctions.autoBackward(260, 100, 100, 90);
  autoFunctions.autoTurnTo(0);
  autoFunctions.openIntake();
  autoFunctions.autoForward(1190, 100, 100, true, 90);
  autoFunctions.autoTurnTo(-90);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(250, 100, 1, 90);
  wait(800, msec);
  autoFunctions.shoot();
  // Goal 3
  autoFunctions.autoBackward(450, 50, 50, 90);
  autoFunctions.autoTurnTo(0);
  autoFunctions.openIntake();
  autoFunctions.autoForward(1400, 100, 600, true, 90);
  autoFunctions.autoTurnTo(-45);
  autoFunctions.autoStrafeLeft(600, 100, 100, 80);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(600, 100, 100, 90);
  autoFunctions.shoot();
  // Goal 4
  autoFunctions.autoBackward(1200, 100, 100, 90);
  autoFunctions.autoTurnTo(90);
  autoFunctions.openIntake();
  autoFunctions.autoForward(680, 100, 300, true, 90);
  autoFunctions.autoTurnTo(0);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(820, 100, 1, 90);
  wait(800, msec);
  autoFunctions.shoot();
  // Goal 5
  autoFunctions.autoBackward(70, 40, 30, 90);
  autoFunctions.autoTurnTo(90);
  autoFunctions.openIntake();
  autoFunctions.autoForward(1100, 100, 100, true, 90);
  autoFunctions.autoTurnTo(45);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(400, 100, 100, 90);
  autoFunctions.shoot();
  // Goal 6
  autoFunctions.autoBackward(250, 100, 100, 90);
  autoFunctions.autoTurnTo(180);
  autoFunctions.openIntake();
  autoFunctions.autoForward(1245, 100, 100, true, 90);
  autoFunctions.autoTurnTo(90);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(250, 100, 100, 90);
  wait(800, msec);
  autoFunctions.shoot();
  // Goal 7
  autoFunctions.autoBackward(450, 50, 50, 90);
  autoFunctions.autoTurnTo(180);
  autoFunctions.openIntake();
  autoFunctions.autoForward(1300, 100, 400, true, 90);
  autoFunctions.autoTurnTo(135);
  autoFunctions.autoStrafeLeft(500, 100, 100, 80);
  autoFunctions.autoForward(600, 100, 100, 90);
  autoFunctions.intakeBrake();
  autoFunctions.shoot();
  // Goal 8
  autoFunctions.autoBackward(250, 100, 100, 90);
  autoFunctions.autoTurnTo(270);
  autoFunctions.autoForward(1200, 100, 100, 90);
  autoFunctions.autoTurnTo(180);
  autoFunctions.autoForward(250, 100, 100, 90);
  wait(800, msec);
  autoFunctions.shoot();
  // Goal centre
  autoFunctions.autoBackward(200, 100, 100, 90);
  autoFunctions.autoTurnTo(0);
  autoFunctions.openIntake();
  autoFunctions.autoForward(500, 100, 100, true, 90);
  autoFunctions.autoStrafeLeft(400, 100, 100, 80);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(400, 100, 200, 90);
  autoFunctions.autoBackward(400, 100, 200, 90);
  autoFunctions.autoStrafeRight(400, 100, 100, 80);
  autoFunctions.openIntake();
  autoFunctions.autoForward(300, 100, 100, 90);
  wait(800, msec);
  autoFunctions.shoot();
  wait(200, msec);
  autoFunctions.autoBackward(200, 100, 100, 90);
}

void AutoMasters::rightHome() {
  // Flipout
  autoFunctions.flipout();
  autoFunctions.openIntake();
  wait(200, msec);
  // Goal 1
  autoFunctions.dumbForward(40, 20, 40, 100);
  autoFunctions.intake(100);
  autoFunctions.autoTurnTo(25);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(100, 50, 1, 100);
  autoFunctions.alignTurnRight(100, 42);
  autoFunctions.shoot();
  // Goal 2
  autoFunctions.autoBackward(600, 50, 100, 100);
  autoFunctions.autoTurnTo(0);
  autoFunctions.autoBackward(1100, 50, 100, 100);
  autoFunctions.autoTurnTo(90);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(350, 50, 150, 100);
  wait(400, msec);
  autoFunctions.shoot();
  // Goal 3
  autoFunctions.autoBackward(250, 50, 50, 100);
  autoFunctions.autoTurnTo(180);
  autoFunctions.autoForward(900, 50, 100, 100);
  autoFunctions.autoTurnTo(135);
  autoFunctions.openIntake();
  autoFunctions.intake(100);
  autoFunctions.autoForward(200, 50, 1, 100);
  autoFunctions.autoForward(200, 1, 50, 100);
  wait(300, msec);
  autoFunctions.autoForward(150, 1, 100, 100);
  autoFunctions.intakeBrake();
  autoFunctions.shoot();
  autoFunctions.autoBackward(200, 1, 1, 100);
}

void AutoMasters::rightTwoAndMiddle() {
  // Flipout
  autoFunctions.flipout();
  autoFunctions.openIntake();
  wait(200, msec);
  // Goal 1
  autoFunctions.dumbForward(40, 20, 40, 100);
  autoFunctions.intake(100);
  autoFunctions.autoTurnTo(25);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(100, 50, 1, 100);
  autoFunctions.alignTurnRight(100, 42);
  autoFunctions.shoot();
  // Goal 2
  autoFunctions.autoBackward(600, 50, 100, 100);
  autoFunctions.autoTurnTo(0);
  autoFunctions.autoBackward(1100, 50, 100, 100);
  autoFunctions.autoTurnTo(90);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(350, 50, 150, 100);
  wait(400, msec);
  autoFunctions.shoot();
  // Goal middle
  autoFunctions.autoBackward(200, 50, 50, 100);
  autoFunctions.autoTurnTo(0);
  autoFunctions.autoBackward(600, 100, 100, 100);
  autoFunctions.autoTurnTo(-90);
  autoFunctions.autoForward(800, 100, 100, 80);
  autoFunctions.autoStrafeRight(700, 100, 100, 100);
  autoFunctions.autoBackward(500, 50, 100, 100);
}

void AutoMasters::rightTwoAndSide() {
  // Flipout
  autoFunctions.flipout();
  autoFunctions.openIntake();
  wait(200, msec);
  // Goal 1
  autoFunctions.dumbForward(40, 20, 40, 100);
  autoFunctions.intake(100);
  autoFunctions.autoTurnTo(25);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(100, 50, 1, 100);
  autoFunctions.alignTurnRight(100, 42);
  autoFunctions.shoot();
  // Goal 2
  autoFunctions.autoBackward(600, 50, 100, 100);
  autoFunctions.autoTurnTo(0);
  autoFunctions.autoBackward(1100, 50, 100, 100);
  autoFunctions.autoTurnTo(90);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(350, 50, 150, 100);
  wait(400, msec);
  autoFunctions.shoot();
  // Goal side
  autoFunctions.indexerBrake();
  autoFunctions.intake(100);
  wait(700, msec);
  // dumbBackward(50, 10, 10, 40);
  while (!autoFunctions.position1 && !autoFunctions.position2 &&
         !autoFunctions.position3) {
    autoFunctions.indexSense();
    wait(10, msec);
  }
  autoFunctions.intakeBrake();
  autoFunctions.autoBackward(350, 50, 50, 100);
  autoFunctions.intake(100);
  autoFunctions.autoTurnTo(-15);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(1500, 50, 150, 100);
  autoFunctions.autoTurnTo(-75);
  autoFunctions.brakeDrive();
  autoFunctions.autoForward(500, 100, 100, 100);
  autoFunctions.shoot();
  autoFunctions.autoBackward(400, 1, 50, 100);
}