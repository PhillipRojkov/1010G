#include "AutoMasters.h"
#include "Odometry.h"

Odometry odometry;
AutoFunctions autoFunctions; //instance of AutoFunctions

void AutoMasters::runOdometry(){
  odometry.setXY();
  //odometry.printCoordinates();
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

void AutoMasters::LRTAuto() {
  
  // Flipout
  autoFunctions.flipout();
  wait(250, msec);
  runIndexer = true;
  thread autoIndexThread(indexThread); //start auto index thread
  thread intakesThread(intakeThread);
  wait(10, msec);

/*
  autoFunctions.autoForward(520, 1, 420, 75);
  autoFunctions.autoTurnTo(-90);
  //autoFunctions.intake(100);
  autoFunctions.autoForward(500, 1, 420, 80);
  wait(100, msec);
  //autoFunctions.intakeBrake();
  autoFunctions.autoForward(50, 10, 10, 80);
  runIndexer = false;
  //autoFunctions.shoot();

  runIndexer = false;
  autoFunctions.indexerBrake();

  autoFunctions.autoBackward(400, 20, 350, 80);
  autoFunctions.autoTurnTo(-45);
  autoFunctions.autoBackward(700, 20, 600, 80);
  autoFunctions.autoTurnTo(-135);
  autoFunctions.autoBackward(540, 20, 400, 90);

  autoFunctions.autoForward(50, 20, 20, 80);
  autoFunctions.turnRange = 2;
  autoFunctions.autoTurnTo(-195);
  autoFunctions.autoForward(1050, 20, 400, 100);
*/
  //Goal 1
  runIndexer = false;
  odometry.pursuit(0, 35, 100);
  //autoFunctions.intake(100);
  odometry.pursuit(-35.3, 31.3, 100, 2.25, 1.5, 0.03);
  //wait(50, msec);
  autoFunctions.intakeBrake();
  //autoFunctions.autoForward(30, 1, 1, 50);
  autoFunctions.timeOutDrive(0.15, 90);
  autoFunctions.intake(-18);
  //autoFunctions.shoot();

  //Middle
  autoFunctions.autoBackward(150, 1, 1, 50);
  runIndexer = false;
  autoFunctions.indexerBrake();
  odometry.pursuit(17.5, 7, -100);
  autoFunctions.intakeBrake();
  autoFunctions.autoTurnTo(-123);
  autoFunctions.timeOutDrive(0.7, -80);
  autoFunctions.autoForward(5, 1, 1, 100);
  odometry.pursuit(28, 14, 100, 2.5, 1.2, 0.035);
  autoFunctions.turnRange = 2;
  autoFunctions.autoTurnTo(-190);

  //Goal 2
  //odometry.pursuit(45, -43, 100);
  autoFunctions.autoForward(600, 20, 1, 100);
  autoFunctions.timeOutDrive(1, 70);

  autoIndexThread.interrupt();
  intakesThread.interrupt();
}

void AutoMasters::skills() {
  // Flipout
  autoFunctions.flipout();
  wait(250, msec);
  runIndexer = true;
  thread autoIndexThread(indexThread); //start auto index thread
  thread intakesThread(intakeThread);
  wait(10, msec);
  

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
  wait(10, msec);
  
  autoIndexThread.interrupt();
  intakesThread.interrupt();
}

void AutoMasters::leftHome() {
  // Flipout
  autoFunctions.flipout();
  wait(250, msec);
  runIndexer = true;
  thread autoIndexThread(indexThread); //start auto index thread
  //thread intakesThread(intakeThread);
  wait(10, msec);
  
  //Goal 1
  odometry.pursuit(0, 35, 100);
  autoFunctions.intake(100);
  odometry.pursuit(-35.3, 31.3, 100, 2.25, 1.5, 0.03);
  wait(50, msec);
  autoFunctions.intakeBrake();
  autoFunctions.timeOutDrive(0.25, 90);
  autoFunctions.intake(-5);
  runIndexer = false;
  autoFunctions.shoot();

  //Goal 3
  autoFunctions.dumbBackward(200, 100, 100, 100);
  runIndexer = true;
  autoFunctions.intake(100);
  autoFunctions.autoTurnTo(135);
  odometry.pursuit(52, -24, 100, 2.5, 4, 0.05);
  autoFunctions.autoTurnTo(180);
  autoFunctions.autoForward(300, 100, 100, 80);
  wait(400, msec);
  autoFunctions.intake(-5);
  autoFunctions.timeOutDrive(0.2, 80);
  autoFunctions.dumbDrive(1, 5);
  runIndexer = false;
  autoFunctions.shoot();

  autoFunctions.intake(-50);
  autoFunctions.autoBackward(200, 10, 100, 100);

  autoIndexThread.interrupt();
  //intakesThread.interrupt();
}

void AutoMasters::leftTwoAndMiddle() {
  // Flipout
  autoFunctions.flipout();
  wait(250, msec);
  runIndexer = true;
  thread autoIndexThread(indexThread); //start auto index thread
  //thread intakesThread(intakeThread);
  wait(10, msec);

  autoFunctions.intake(100);
  autoFunctions.autoForward(90, 40, 40, 80);
  DriveFR.spin(fwd, 90, pct);
  DriveBR.spin(fwd, 90, pct);
  wait(70, msec);

  autoFunctions.dumbDrive(1, 40);
  wait(400, msec);
  autoFunctions.dumbDrive(1, 2);
  runIndexer = false;
  autoFunctions.shoot();
  runIndexer = true;
  wait(500, msec);
  autoFunctions.intakeBrake();
  runIndexer = false;
  autoFunctions.doubleShot(900);
  wait(10, msec);
  autoFunctions.outdex(100);
  autoFunctions.intake(-5);
  autoFunctions.dumbBackward(200, 100, 100, 100);
  autoFunctions.intake(-100);
  wait(200, msec);
  autoFunctions.indexerBrake();
  runIndexer = true;
  autoFunctions.intake(100);

  odometry.pursuit(51.9, -5.1, 100, 2.1, 1.5, 0.05);
  autoFunctions.dumbBackward(9, 4, 5, 100);
  odometry.pursuit(58, -27.8, 100, 2.5, 2, 0.045, 0.49 * (180 / M_PI));

  DriveFR.spin(fwd, -65, pct);
  DriveBR.spin(fwd, -65, pct);
  DriveFL.spin(fwd, -100, pct);
  DriveBL.spin(fwd, -100, pct);
  wait(200, msec);
  autoFunctions.brakeDrive();

  odometry.pursuit(17, -47.5, 100, 4, 5, 0.055);
  autoFunctions.autoTurnTo(265);

  autoFunctions.dumbDrive(1, 70);
  wait(200, msec);
  autoFunctions.dumbDrive(1, 4);
  wait(200, msec);
  runIndexer = false;
  autoFunctions.doubleShot(1050);
  autoFunctions.intake(-50);

  autoFunctions.autoBackward(180, 1, 100, 100);

  autoIndexThread.interrupt();
  //intakesThread.interrupt();
}

void AutoMasters::rightTwoAndSide() {
  // Flipout
  autoFunctions.flipout();
  wait(250, msec);
  runIndexer = true;
  thread autoIndexThread(indexThread); //start auto index thread
  thread intakesThread(intakeThread);
  wait(10, msec);
  
  autoIndexThread.interrupt();
  intakesThread.interrupt();
}