#include "vex.h"
#include "AutoFunctions.cpp"

using namespace vex;

AutoFunctions af;

class AutoMasters {
  //AutoFunctions
void autoForward(double degrees, double iDeg, double fDeg, double speed) { //Forward auto function. degrees > iDeg + fDeg
  af.autoForward(degrees, iDeg, fDeg, speed);
}

void autoBackward(double degrees, double iDeg, double fDeg, double speed) { //Backward auto function. degrees > iDeg + fDeg
  af.autoBackward(degrees, iDeg, fDeg, speed);
}

void autoTurnTo(double degrees) { //+degrees turns right, -degrees turns left
  af.autoTurnTo(degrees);
}

void autoStrafeLeft (double degrees, double iDeg, double fDeg, double speed) { //Strafe left auto function. degrees > iDeg + fDeg) {
  af.autoStrafeLeft(degrees, iDeg, fDeg, speed);
}

void autoStrafeRight (double degrees, double iDeg, double fDeg, double speed) { //Strafe right auto function. degrees > iDeg + fDeg) {
  af.autoStrafeRight(degrees, iDeg, fDeg, speed);
}

void intake(double speed) {
  af.intake(speed);
}

void outake(double speed) {
  af.outake(speed);
}

void intakeBrake() {
  af.intakeBrake();
}

void index(double speed) {
  af.index(speed);
}

void pIndex(double speed, double degrees) {
  af.pIndex(speed, degrees);
}

void outdex(double speed) {
  af.outdex(speed);
}

void pOutdex(double speed, double degrees) {
  af.pOutdex(speed, degrees);
}

void indexerBrake() {
  af.indexerBrake();
}

// -------------------------------------------------------------------------------
public:
//Master Functions
void autoCalibrate () { //Runs every single action
  autoForward(720, 180, 360, 100);

  autoTurnTo(-90);

  autoTurnTo(0);

  autoStrafeLeft(720, 180, 180, 100);

  autoStrafeRight(720, 180, 180, 100);

  autoBackward(720, 180, 180, 100);

  intake(100);
  wait(1000, msec);

  outake(100);
  wait(1000, msec);

  intakeBrake();

  index(127);
  wait(1000, msec);

  outdex(127);
  wait(1000, msec);

  indexerBrake();
}

void turnCalibrate () {
  autoTurnTo(-90);
  autoTurnTo(135);
  autoTurnTo(-135);
  autoTurnTo(180);
  autoTurnTo(-180);
  autoTurnTo(0);
}

void skillsAuto() { //Start red, left of middle
  intake(100);

  autoForward(570, 180, 180, 100);

  autoTurnTo(135);

  intakeBrake();

  autoForward(650, 180, 1, 100);
  
  //First goal
  pIndex(100, 1300);
  pOutdex(100, 300);
  indexerBrake();

  autoBackward(150, 70, 70, 90);

  intake(100);

  autoTurnTo(0);

  autoForward(1350, 180, 600, 100);

  autoTurnTo(90);

  pIndex(100, 360);
  indexerBrake();

  intakeBrake();

  autoForward(110, 90, 1, 100);
  
  //middle goal
  pIndex(100, 2000);
  indexerBrake();
  wait(500, msec);
  pIndex(100, 2000);
  indexerBrake();

  autoBackward(110, 90, 90, 100);

  intake(100);

  autoTurnTo(-7);

  autoForward(1400, 180, 500, 100);

  autoBackward(200, 100, 100, 100);

  autoTurnTo(45);

  pIndex(100, 360);
  indexerBrake();

  intakeBrake();

  autoForward(720, 180, 1, 100);

  //Back goal
  pIndex(100, 1000);
  pOutdex(100, 300);
  indexerBrake();

  autoBackward(680, 180, 180, 100);
}

void redLeftCorner() {

}

void blueLeftCorner() {

}
};