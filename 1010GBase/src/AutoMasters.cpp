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

double currentTime = 0;

bool i = true;
bool opening = false;
double timeToIntake = 0;
double intakeDelay = 0; // += a delay in seconds
void intakeThread() {
  while (true) {
  if(Brain.timer(sec) < timeToIntake && Brain.timer(sec) > intakeDelay) {
    i = true;
    autoFunctions.intake(100);
  } else if (i) {
    autoFunctions.intakeBrake();
    i = false;
  }
  if (Brain.timer(sec) > timeToIntake) {
    timeToIntake = Brain.timer(sec);
  }
  if (Brain.timer(sec) > intakeDelay) {
    intakeDelay = Brain.timer(sec);
  }
  if (opening) {
    autoFunctions.openIntake();
  }
  wait(10, msec);
  }
}

void cornerDescore() {
  timeToIntake += 1;
  autoFunctions.dumbBackward(40, 1, 1, 20);
  wait(200, msec);
  timeToIntake = 0;
  runIndexer = false;
  wait(10, msec);
  autoFunctions.dumbBackward(50, 20, 20, 80);
  autoFunctions.outdex(100);
  autoFunctions.openDegrees(100, 135);
}

void AutoMasters::skills() {
  // Flipout
  autoFunctions.flipout();
  wait(250, msec);
  runIndexer = true;
  thread autoIndexThread(indexThread); //start auto index thread
  thread intakesThread(intakeThread);
  wait(10, msec);
  //Goal 1
  timeToIntake += 0.4;
  wait(400, msec);
  odometry.driveToPoint(-4.3, 17.6, -45, 100); //Drive to ball
  timeToIntake += 15;
  wait(700, msec);
  odometry.driveToPoint(-28.3, 30.9, -90, 90); //Drive to goal
  autoFunctions.timeOutDrive(0.4, 70);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Descore
  cornerDescore();
  //Goal 2
  odometry.driveToPoint(8.4, 58, 45, 100, 12, 4, 4.5, 14, 1, 0.07); //Drive to ball
  runIndexer = true;
  timeToIntake += 15;
  wait(100, msec);
  odometry.driveToPoint(12.6, 64.7, -45, 100); //Drive to goal
  autoFunctions.timeOutDrive(0.3, 50);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Descore
  timeToIntake += 1;
  autoFunctions.autoBackward(100, 1, 1, 20);
  timeToIntake = 0;
  runIndexer = false;
  wait(10, msec);
  autoFunctions.outdex(100);
  autoFunctions.openDegrees(100, 135);
  //Goal 3
  odometry.driveToPoint(14.1, 62, -60, 100, 12, 4, 4, 12, 1.5, 0.1); //Turn for descore
  odometry.driveToPoint(51, 81.6, 45, 100, 10, 6, 3); //On top of ball
  runIndexer = true;
  timeToIntake += 15;
  wait(100, msec);
  odometry.driveToPoint(49.7, 103, 0, 100); //Line up on goal
  autoFunctions.timeOutDrive(0.4, 70); //Drive to goal
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Descore
  cornerDescore();
  //Goal 4
  autoFunctions.dumbBackward(100, 10, 10, 100);
  odometry.driveToPoint(58.5, 50.3, 135, 100, 10, 10, 3); //On top of ball
  runIndexer = true;
  timeToIntake += 15;
  wait(100, msec);
  odometry.driveToPoint(83.6, 61.7, 45, 100); //Line up on goal
  autoFunctions.timeOutDrive(0.3, 70); //Drive to goal
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Descore
  timeToIntake += 1;
  autoFunctions.dumbBackward(100, 1, 1, 20);
  timeToIntake = 0;
  runIndexer = false;
  wait(10, msec);
  autoFunctions.outdex(100);
  autoFunctions.openDegrees(100, 135);
  //Goal 5
  odometry.driveToPoint(80.1, 61.5, 25, 100, 12, 4, 4, 12, 1.5, 0.1); //Turn for descore
  odometry.driveToPoint(96.2, 40.2, 135, 100, 10, 20, 3); //On top of ball
  runIndexer = true;
  timeToIntake += 15;
  wait(100, msec);
  odometry.driveToPoint(120.9, 25.3, 90, 100, 13, 4, 2, 9, 1, 0.07); //Line up on goal
  autoFunctions.timeOutDrive(0.5, 70); //Drive to goal
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Descore
  cornerDescore();
  //Goal 6
  autoFunctions.dumbBackward(150, 20, 20, 80); //Back up from goal
  odometry.driveToPoint(84.1, 1.9, 225, 100); //On top of ball
  runIndexer = true;
  timeToIntake += 15;
  wait(100, msec);
  odometry.driveToPoint(78.7, -5.7, 135, 100, 15, 10, 6); //Line up on goal
  autoFunctions.timeOutDrive(0.4, 70);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Descore
  timeToIntake += 1;
  autoFunctions.autoBackward(80, 1, 1, 20);
  timeToIntake = 0;
  runIndexer = false;
  wait(10, msec);
  autoFunctions.outdex(100);
  autoFunctions.openDegrees(100, 135);
  //Goal centre
  odometry.driveToPoint(67.7, 7, -45, 100, 12, 20, 4); //On top of ball
  runIndexer = true;
  timeToIntake += 1.7;
  odometry.driveToPoint(59.6, 21.4, -45, 100); //Line up poke
  wait(100, msec);
  autoFunctions.autoForward(130, 1, 1, 100); //Triple poke
  odometry.driveToPoint(59.6, 21.4, -45, 100); //Line up poke
  autoFunctions.autoForward(130, 1, 1, 100);
  autoFunctions.autoBackward(60, 1, 1, 100);
  odometry.driveToPoint(59.6, 21.4, -45, 100); //Line up poke
  autoFunctions.autoForward(130, 1, 1, 100);
  autoFunctions.autoBackward(80, 1, 1, 100);
  odometry.driveToPoint(56.2, 19.2, -45, 100);//Line up shoot
  autoFunctions.openDegrees(100, 180);
  wait(200, msec);
  autoFunctions.autoForward(160, 1, 1, 100);
  autoFunctions.brakeDrive();
  wait(100, msec);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Goal 7
  autoFunctions.autoBackward(100, 40, 40, 70);
  odometry.driveToPoint(35.4, -23.3, -150, 100); //On top of ball
  timeToIntake += 15;
  wait(100, msec);
  odometry.driveToPoint(40.2, -45.4, -180, 100, 10, 1.5, 0.12); //Line up on goal
  autoFunctions.timeOutDrive(0.6, 70);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Descore
  cornerDescore();

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