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

double pX;
double pY;
double pH;
void pursuitThread() {
  while (true) {
    odometry.driveToPoint(pX, pY, pH, 100, 6, 3, 4, 10, 0.01, 0.0001);
    wait(10, msec);
  }
}

void cornerDescore() {
  runIndexer = false;
  wait(10, msec);
  autoFunctions.indexerBrake();
  IndexerLow.spin(forward, 20, pct);
  timeToIntake += 1;
  autoFunctions.dumbBackward(40, 1, 1, 20);
  wait(200, msec);
  timeToIntake = 0;
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
  odometry.driveToPoint(-28.3, 30.9, -90, 100); //Drive to goal
  autoFunctions.timeOutDrive(0.4, 70);
  runIndexer = false;
  autoFunctions.shoot();
  //Descore
  cornerDescore();
  //Goal 2
  odometry.driveToPoint(8.4, 58, 45, 100, 12, 4, 4.5, 14, 1, 0.07); //Drive to ball
  runIndexer = true;
  timeToIntake += 1.5;
  wait(100, msec);
  odometry.driveToPoint(12.6, 64.7, -45, 100); //Drive to goal
  autoFunctions.timeOutDrive(0.3, 50);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  //Descore
  timeToIntake += 2;
  autoFunctions.dumbBackward(100, 1, 1, 20);
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
  odometry.driveToPoint(55.5, 52.9, 135, 100, 10, 10, 3); //On top of ball
  runIndexer = true;
  timeToIntake += 2;
  wait(200, msec);
  odometry.driveToPoint(81.1, 60.5, 45, 100); //Line up on goal
  autoFunctions.timeOutDrive(0.4, 80); //Drive to goal
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
  odometry.driveToPoint(97.2, 41.9, 135, 100, 10, 20, 3); //On top of ball
  runIndexer = true;
  timeToIntake += 15;
  wait(100, msec);
  odometry.driveToPoint(116.9, 28.5, 90, 100, 13, 4, 2, 9, 1, 0.07); //Line up on goal
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
  timeToIntake += 2;
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
  odometry.driveToPoint(57.2, 18.2, -45, 100);//Line up shoot
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
  odometry.driveToPoint(35.4, -23.3, -150, 100, 10, 10, 4); //On top of ball
  timeToIntake += 15;
  wait(100, msec);
  odometry.driveToPoint(40.2, -45.4, -180, 100, 10, 1.5, 0.12); //Line up on goal
  autoFunctions.timeOutDrive(0.6, 70);
  runIndexer = false;
  autoFunctions.shoot();
  //Descore
  autoFunctions.index(40);
  autoFunctions.dumbBackward(150, 20, 20, 13);

  autoIndexThread.interrupt();
  intakesThread.interrupt();
}

void AutoMasters::rightHome() {
  
}

void AutoMasters::leftHome() {
  // Flipout
  autoFunctions.flipout();
  wait(250, msec);
  runIndexer = true;
  thread autoIndexThread(indexThread); //start auto index thread
  thread intakesThread(intakeThread);
  wait(10, msec);
  //Goal 1
  timeToIntake += 0.52;
  wait(50, msec);
  autoFunctions.autoForward(150, 30, 30, 100);
  odometry.driveToPoint(-27, 31.2, -90, 100, 10, 4, 6, 16, 1.6, 0.2); //On top of ball
  timeToIntake += 1.2;
  wait(600, msec);
  autoFunctions.timeOutDrive(0.4, 100);
  /*odometry.driveToPoint(-35.5, 30.2, -90, 100, 9, 4, 4, 10, 1, 0.05); //On goal
  //Pursuit stay at goal
  pX = -35.5;
  pY = 30.2;
  pH = -90;
  thread pursuitThreadA(pursuitThread); //Yes this is jank AF. Too bad!*/
  runIndexer = false;
  autoFunctions.shoot();
  /*//Cycle
  while(!autoFunctions.position3 && (!autoFunctions.position2 || !autoFunctions.position1)) {
    autoFunctions.indexSense();
    timeToIntake += 1;
    runIndexer = true;
    wait(10, msec);
  }
  runIndexer = false;
  timeToIntake = 0;
  autoFunctions.shoot();
  runIndexer = true;
  pursuitThreadA.interrupt();*/
  //Back out
  autoFunctions.openDegrees(100, 180);
  autoFunctions.dumbBackward(200, 50, 50, 100);
  autoFunctions.outdex(100);
  //Goal 3
  odometry.driveToPoint(44.5, -24, -225, 100, 12, 10, 6, 6, 1.8, 0.2); //Drive across field
  odometry.driveToPoint(44.5, -43.8, -180, 100, 10, 10, 5); //On top of ball
  timeToIntake += 1.5;
  wait(1200, msec);
  autoFunctions.timeOutDrive(0.4, 100);
  /*odometry.driveToPoint(47.9, -47.8, -180, 100, 9, 4, 4, 10, 1, 0.05); //On top of goal
  //Pursuit stay at goal
  pX = 47.9;
  pY = -47.8;
  pH = -180;
  thread pursuitThreadB(pursuitThread); //Now you see why this is jank. Too bad!*/
  runIndexer = false;
  autoIndexThread.interrupt();
  intakesThread.interrupt();
  autoFunctions.shoot();
  /*
  //Cycle
  while(!autoFunctions.position3 && (!autoFunctions.position2 || !autoFunctions.position1)) {
    autoFunctions.indexSense();
    timeToIntake += 1;
    runIndexer = true;
    wait(10, msec);
  }
  runIndexer = false;
  timeToIntake = 0;
  autoFunctions.shoot();
  pursuitThreadA.interrupt();
  */
  //Back out
  autoFunctions.openDegrees(100, 180);
  autoFunctions.dumbBackward(100, 40, 40, 100);
  autoFunctions.outdex(100);
}

void AutoMasters::leftTwoAndMiddle() {
  // Flipout
  autoFunctions.flipout();
  wait(100, msec);
  runIndexer = true;
  thread autoIndexThread(indexThread); //start auto index thread
  thread intakesThread(intakeThread);
  wait(10, msec);
  //Goal 1
  timeToIntake += 0.5;
  wait(50, msec);
  autoFunctions.autoForward(140, 10, 30, 100);
  odometry.driveToPoint(-27, 31.2, -90, 100, 10, 4, 6, 16, 1.9, 0.2); //On top of ball
  timeToIntake += 1.3;
  wait(500, msec);
  autoFunctions.timeOutDrive(0.4, 100);
  /*odometry.driveToPoint(-35.5, 30.2, -90, 100, 9, 4, 4, 10, 1, 0.05); //On goal
  //Pursuit stay at goal
  pX = -35.5;
  pY = 30.2;
  pH = -90;
  thread pursuitThreadA(pursuitThread); //Yes this is jank AF. Too bad!*/
  runIndexer = false;
  autoFunctions.shoot();
  /*//Cycle
  while(!autoFunctions.position3 && (!autoFunctions.position2 || !autoFunctions.position1)) {
    autoFunctions.indexSense();
    timeToIntake += 1;
    runIndexer = true;
    wait(10, msec);
  }
  runIndexer = false;
  timeToIntake = 0;
  autoFunctions.shoot();
  runIndexer = true;
  pursuitThreadA.interrupt();*/
  //Back out
  timeToIntake = 0;
  autoFunctions.openDegrees(100, 160);
  autoFunctions.dumbBackward(200, 30, 10, 100);
  autoFunctions.outdex(100);
  //Grab middle ball
  odometry.driveToPoint(13.4, 52.2, 45, 100, 10, 4, 5, 10, 1.5, 0.2); //Align for ball
  runIndexer = true;
  timeToIntake += 1;
  wait(100, msec);
  //Middle
  odometry.driveToPoint(30.8, 34.7, 45, 100, 12, 4, 6, 16, 1.2, 0.1); //Strafe to push ball
  //Side
  autoFunctions.autoBackward(150, 30, 10, 100);
  timeToIntake += 2;
  autoFunctions.autoTurnTo(130);
  odometry.driveToPoint(76.2, -24.4, 45, 100, 10, 1.1, 8, 18, 1.8, 0.2);
  autoFunctions.timeOutDrive(0.55, 100);
  runIndexer = false;
  autoIndexThread.interrupt();
  intakesThread.interrupt();
  autoFunctions.shoot();
  autoFunctions.autoBackward(150, 1, 50, 100);
}

void AutoMasters::rightTwoAndSide() {
  
}