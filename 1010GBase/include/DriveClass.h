#pragma once
#include "vex.h"

class DriveClass {
  public:
  // Drive parameters
  int controllerDeadZone = 10; // Stick dead zone (stick values from 0 - 100)
  int strafeSpeed = 100;
  double strafeStickMultiplier =
      0.6; // Strafe slowdown multiplier applied to sticks
  double frontStrafeSpeedMultiplier =
      0.65; // Slow down front wheels in strafe in order to keep straight

  // Intake parameters
  // pot parameters
  int leftPotDesired = 220; // pot values range from 0 to leftPotDesired
  // When the pot value is greater than leftPotDesired, intakeL is out
  int rightPotDesired = 115; // pot values range from 360 to rightPotDesired
  // When the pot value is less than rightPotDesired, intakeR is out
  int potRange1 = 5;
  int potRange2 = 3;
  bool leftBrake = false;
  bool rightBrake = false;
  bool doIntake = false;
  // intake kP
  double intakekP = 4;
  double intakekI = 0.01;
  double intakekD = 0;
  double leftIntakeTotalError = 0;
  double rightIntakeTotalError = 0;
  double leftIntakePrevError = 0;
  double rightIntakePrevError = 0;

  // Indexer parameters
  bool position1;
  bool position2;
  bool position3;
  int scoreNum = 0;
  int indexRotation;
  double timeToIndex = 1.2;
  double t = 0;

  bool enableIndex = false;

  void runTankBase();

  void runArcadeBase();

  void index();

  void cIndex();

  void openIntake();

  void intake();

  void indexSense();

  void intakeSense();

  void score();

  void resetScoreNum();

  void checkPosition1();
};