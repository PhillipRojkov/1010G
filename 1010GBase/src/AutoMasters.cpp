#include "AutoMasters.h"
#include "Odometry.h"

Odometry odometry;
AutoFunctions autoFunctions; //instance of AutoFunctions

void AutoMasters::runOdometry(){
  odometry.setXY();
  odometry.printCoordinates();
}

void AutoMasters::runFlipout() {
  autoFunctions.flipout();
}

bool runIndexer = false;

void indexThread() {
  while (true) {
    if (runIndexer) {
      autoFunctions.cIndex();
    } 
    wait (10, msec);
  }
}

bool i = true;
double timeToIntake = 0;
void intakeThread() {
  while (true) {
  if(Brain.timer(sec) < timeToIntake) {
    i = true;
    autoFunctions.intake(100);
  } else if (i) {
    autoFunctions.intakeBrake();
    i = false;
  }
  if (Brain.timer(sec) > timeToIntake) {
    timeToIntake = Brain.timer(sec);
  }
  wait(10, msec);
  }
}

void AutoMasters::skills() {
  // Flipout
  autoFunctions.flipout();
  runIndexer = true;
  thread autoIndexThread(indexThread); //start auto index thread
  thread intakesThread(intakeThread);
  //Goal 1
  odometry.driveToPoint(-4, 15, -45, 100, 10, 4, 6); //Drive to ball
  timeToIntake += 1.5;
  wait(100, msec);
  odometry.driveToPoint(-28.8, 34, -85, 100, 30, 4, 6, 1, 0.1); //Drive to goal
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  wait(250, msec);
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 2
  autoFunctions.openDegrees(100, 90);
  odometry.driveToPoint(7, 56, 43, 100, 10, 10, 4); //Drive to ball
  timeToIntake += 1.2;
  wait(100, msec);
  odometry.driveToPoint(13, 69, -45, 100, 15, 10, 4, 1, 0.1); //Drive to goal
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  wait(250, msec);
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 3
  odometry.driveToPoint(40, 73, 45, 100); //Line up on ball
  autoFunctions.openDegrees(100, 90);
  odometry.driveToPoint(45, 78, 45, 100, 10, 10, 4); //On top of ball
  timeToIntake += 1.6;
  odometry.driveToPoint(49.2, 98.2, 0, 100, 10, 10, 6); //Line up on goal
  odometry.driveToPoint(49.6, 107, 0, 100, 15, 10, 6, 1.5, 0.1); //Drive to goal
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  wait(250, msec);
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 4
  odometry.driveToPoint(52, 61, 135, 100, 10, 10, 6); //Line up on ball
  autoFunctions.openDegrees(100, 90);
  odometry.driveToPoint(57, 56, 135, 100, 10, 10, 4); //On top of ball
  timeToIntake += 1.8;
  wait(100, msec);
  odometry.driveToPoint(82, 59, 45, 100, 10, 10, 8, 1, 0.1); //Line up on goal
  odometry.driveToPoint(82.5, 66, 45, 100, 15, 10, 8, 1, 0.1); //Drive to goal
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  wait(250, msec);
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 5
  odometry.driveToPoint(79.5, 63, 45, 100, 10, 1, 4); //Back up from goal
  odometry.driveToPoint(93, 53.5, 135, 100); //Line up on ball
  autoFunctions.openDegrees(100, 90);
  odometry.driveToPoint(99, 48, 135, 100); //On top of ball
  timeToIntake += 1.5;
  wait(200, msec);
  odometry.driveToPoint(118, 30.5, 90, 100, 10, 4, 6); //Line up on goal
  odometry.driveToPoint(123, 30, 90, 100, 15, 4, 6, 1.5, 0.14); //Drive to goal
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  wait(250, msec);
  autoFunctions.shoot();
  runIndexer = true;
  autoFunctions.openDegrees(100, 90);
  //Goal 6
  odometry.driveToPoint(84.5, 3, 225, 100); //Drive to ball
  timeToIntake += 1.2;
  wait(100, msec);
  odometry.driveToPoint(81, -5, 135, 100, 15, 4, 6, 1, 0.1); //Drive to goal
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  wait(250, msec);
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 7
  odometry.driveToPoint(54, -15, 225, 100); //Line up on ball
  autoFunctions.openDegrees(100, 90);
  odometry.driveToPoint(45, -24, 225, 100); //On top of ball
  timeToIntake += 1.5;
  wait(100, msec);
  odometry.driveToPoint(50, -38, 180, 100); //Line up on goal
  odometry.driveToPoint(49, -49, 180, 100, 15, 4, 6, 1, 0.1); //Drive to goal
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  wait(250, msec);
  autoFunctions.shoot();
  runIndexer = true;
  autoFunctions.openDegrees(100, 90);
  //Goal centre
  odometry.driveToPoint(43, -2, -45, 100); //Line up on ball
  timeToIntake += 1.5;
  odometry.driveToPoint(35, 4, -45, 100); //On top of ball
  odometry.driveToPoint(35, 11.5, 45, 100); //Line up poke
  autoFunctions.autoForward(100, 1, 1, 100);
  odometry.driveToPoint(42.5, 15.5, 350, 100); //Align for shoot
  runIndexer = false;
  wait(250, msec);
  autoFunctions.shoot();
  runIndexer = true;
  autoFunctions.autoBackward(100, 1, 1, 100);

  intakesThread.interrupt();
}

void AutoMasters::rightHome() {
  // Flipout
  autoFunctions.flipout();
  // Goal 1
  autoFunctions.autoForward(40, 20, 40, 100);
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

void AutoMasters::leftHome() {
  // Flipout
  autoFunctions.flipout();
  // Goal 1
  autoFunctions.intake(100);
  wait(200, msec);
  autoFunctions.autoForward(90, 30, 30, 50);
  autoFunctions.autoTurnTo(-45);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(50, 10, 10, 50);
  autoFunctions.shoot();
  // Goal 2
  autoFunctions.autoBackward(200, 50, 100, 100);
  autoFunctions.intake(100);
  autoFunctions.autoTurnTo(0);
  autoFunctions.autoBackward(340, 100, 100, 100);
  autoFunctions.intakeBrake();
  autoFunctions.autoTurnTo(-90);
  autoFunctions.autoForward(110, 50, 50, 100);
  wait(400, msec);
  autoFunctions.shoot();
  // Goal 3
  autoFunctions.autoBackward(80, 30, 30, 100);
  autoFunctions.autoTurnTo(-180);
  autoFunctions.autoForward(400, 100, 100, 70);
  wait(30, msec);
  autoFunctions.autoTurnTo(-135);
  autoFunctions.intake(100);
  autoFunctions.autoForward(160, 50, 50, 60);
  wait(600, msec);
  autoFunctions.intakeBrake();
  autoFunctions.autoForward(20, 1, 10, 50);
  autoFunctions.shoot();
  autoFunctions.autoBackward(70, 1, 1, 100);
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