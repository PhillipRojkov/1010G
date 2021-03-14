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
bool opening = false;
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
  if (opening) {
    autoFunctions.openIntake();
  }
  wait(10, msec);
  }
}

void AutoMasters::skills() {
  // Flipout
  autoFunctions.flipout();
  wait(250, msec);
  runIndexer = true;
  thread autoIndexThread(indexThread); //start auto index thread
  thread intakesThread(intakeThread);
  //Goal 1
  odometry.driveToPoint(-4, 15, -45, 100, 10, 4, 6); //Drive to ball
  timeToIntake += 1.8;
  wait(700, msec);
  odometry.driveToPoint(-28.6, 33.7, -85, 100, 30, 4, 6); //Drive to goal
  autoFunctions.timeOutDrive(0.7, 70);
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 2
  autoFunctions.autoBackward(30, 10, 10, 80);
  autoFunctions.openDegrees(100, 90);
  odometry.driveToPoint(7, 56, 43, 100, 20, 10, 8); //Drive to ball
  timeToIntake += 1.2;
  wait(100, msec);
  odometry.driveToPoint(13, 69, -45, 100, 15, 10, 4, 1.2, 0.12); //Drive to goal
  autoFunctions.timeOutDrive(0.5, 50);
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 3
  odometry.driveToPoint(39, 72, 45, 90, 20, 8, 6); //Line up on ball
  autoFunctions.openDegrees(100, 90);
  odometry.driveToPoint(50.4, 83.9, 45, 100, 15, 10, 4); //On top of ball
  timeToIntake += 1.6;
  odometry.driveToPoint(49.2, 98.2, 0, 100, 25, 10, 7, 1.5, 0.12); //Line up on goal
  autoFunctions.timeOutDrive(0.9, 70); //Drive to goal
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 4
  autoFunctions.autoBackward(200, 70, 70, 100);
  odometry.driveToPoint(52, 61, 135, 90, 15, 10, 6, 2, 0.15); //Line up on ball
  autoFunctions.openDegrees(100, 90);
  odometry.driveToPoint(57, 56, 135, 100, 15, 10, 4, 1.5, 0.12); //On top of ball
  timeToIntake += 1.8;
  wait(100, msec);
  odometry.driveToPoint(71.7, 59.4, 45, 100, 15, 10, 8); //Line up on goal
  autoFunctions.timeOutDrive(0.8, 90); //Drive to goal
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 5
  autoFunctions.autoBackward(100, 40, 40, 70);
  odometry.driveToPoint(93.5, 49.2, 135, 100, 15, 2, 4); //Line up on ball
  autoFunctions.openDegrees(100, 90);
  odometry.driveToPoint(98.2, 45, 135, 100, 15, 4, 4); //On top of ball
  timeToIntake += 1.5;
  wait(100, msec);
  odometry.driveToPoint(116.8, 31, 90, 90); //Line up on goal
  autoFunctions.timeOutDrive(1, 70); //Drive to goal
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 6
  autoFunctions.autoBackward(100, 20, 20, 80); //Back up from goal
  odometry.driveToPoint(94.8, 11.8, 225, 90, 15, 10, 5); //Line up on ball
  autoFunctions.openDegrees(100, 90);
  odometry.driveToPoint(89.5, 6, 225, 100, 10, 4, 4); //Drive to ball
  timeToIntake += 1.2;
  wait(100, msec);
  odometry.driveToPoint(82.7, -0.5, 135, 100, 15, 10, 6); //Line up on goal
  autoFunctions.timeOutDrive(0.8, 70);
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 7
  /*autoFunctions.autoBackward(100, 40, 40, 70);
  odometry.driveToPoint(54, -13, 225, 100, 18, 10, 4); //Line up on ball
  autoFunctions.openDegrees(100, 90);
  odometry.driveToPoint(46.5, -20.5, 225, 100); //On top of ball
  timeToIntake += 1.5;
  wait(100, msec);
  odometry.driveToPoint(52, -22, 180, 100, 15, 1.5, 0.12); //Line up on goal
  autoFunctions.timeOutDrive(1.4, 70);
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  autoFunctions.openDegrees(100, 90);
  //Goal centre
  odometry.driveToPoint(46, 0.5, -45, 100); //Line up on ball
  timeToIntake += 1.5;
  odometry.driveToPoint(35.5, 8, -45, 100); //On top of ball
  odometry.driveToPoint(36, 12, 45, 100, 20, 4, 6); //Line up poke
  autoFunctions.autoForward(100, 1, 1, 100);
  autoFunctions.autoBackward(100, 1, 1, 100);
  autoFunctions.openDegrees(100, 135);
  autoFunctions.autoForward(90, 1, 1, 100);
  wait(100, msec);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  autoFunctions.autoBackward(100, 1, 1, 100);*/
  autoFunctions.autoBackward(100, 40, 40, 70);
  autoFunctions.openDegrees(100, 90);
  odometry.driveToPoint(76.6, 4, -45, 100); //Line up on ball
  odometry.driveToPoint(71, 10.2, -45, 90); //On top of ball
  timeToIntake += 1.5;
  odometry.driveToPoint(66, 22.3, -45, 100); //Line up poke
  autoFunctions.autoForward(100, 1, 1, 100);
  autoFunctions.autoBackward(100, 1, 1, 100);
  odometry.driveToPoint(63.5, 19, -45, 90); //Line up shoot
  autoFunctions.openDegrees(100, 160);
  wait(200, msec);
  autoFunctions.autoForward(200, 1, 1, 100);
  wait(100, msec);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;

  autoIndexThread.interrupt();
  intakesThread.interrupt();
}

void AutoMasters::rightHome() {
  // Flipout
  autoFunctions.flipout();
  wait(250, msec);
  runIndexer = true;
  thread autoIndexThread(indexThread); //start auto index thread
  thread intakesThread(intakeThread);
  //Goal 1
  odometry.driveToPoint(-16.8, 35, -90, 100, 18, 4, 6); //Drive to ball
  autoFunctions.autoForward(80, 20, 20, 100);
  timeToIntake += 1.4;
  wait(1000, msec);
  autoFunctions.timeOutDrive(0.6, 90); //Drive to goal
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 2
  autoFunctions.autoBackward(200, 70, 70, 100);
  //autoFunctions.openIntake();
  // odometry.driveToPoint(-4, 35, -90, 100, 135, 2, 5); //Turn
  autoFunctions.autoTurnTo(-225);
  opening = true;
  odometry.driveToPoint(44, -30, -225, 100, 18, 2, 5); //Drive to ball
  odometry.driveToPoint(44, -34.7, -180, 100, 18, 2, 5); //Drive to ball
  autoFunctions.autoForward(20, 10, 10, 100);
  opening = false;
  timeToIntake += 1.5;
  autoFunctions.autoForward(40, 20, 20, 100);
  wait(800, msec);
  autoFunctions.timeOutDrive(0.5, 90); //Drive to goal
  intakesThread.interrupt();
  autoFunctions.openDegrees(100, 45);
  runIndexer = false;
  autoIndexThread.interrupt();
  autoFunctions.shoot();
  runIndexer = true;
  autoFunctions.autoBackward(150, 50, 50, 90);
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