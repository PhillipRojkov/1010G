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
  thread intakesThread(intakeThread);
  wait(10, msec);
  
  autoIndexThread.interrupt();
  intakesThread.interrupt();
}

void AutoMasters::LRTAuto() {
  // Flipout
  autoFunctions.flipout();
  wait(250, msec);
  runIndexer = true;
  thread autoIndexThread(indexThread); //start auto index thread
  thread intakesThread(intakeThread);
  wait(10, msec);

  //Goal 1
  odometry.pursuit(0.1, 40, 100);
  autoFunctions.intake(100);
  odometry.pursuit(-40, 40, 100);
  autoFunctions.intakeBrake();
  autoFunctions.shoot();

  //Middle
  odometry.pursuit(40, 30, -100);
  odometry.pursuit(50, 35, -100);

  //Goal 2
  autoFunctions.intake(100);
  odometry.pursuit(50, -50, 100);
  autoFunctions.intakeBrake();
  autoFunctions.shoot();
  autoFunctions.autoBackward(100, 10, 200, 100);

  autoIndexThread.interrupt();
  intakesThread.interrupt();
}

void AutoMasters::leftTwoAndMiddle() {
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