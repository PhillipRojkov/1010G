#pragma once
#include "vex.h"

class DriveClass {
private:
  // Drive parameters
  int controllerDeadZone = 17; // Stick dead zone (stick values from 0 - 100)
  int strafeSpeed = 100;
  double strafeStickMultiplier =
      0.6; // Strafe slowdown multiplier applied to sticks
  double strafeWeighting =
      1; // Larger strafeWeighting increases front speed, decreases rear speed. 1 is equal

  // Intake parameters
  // pot parameters
  int leftPotDesired = 30;   // min
  int rightPotDesired = 105; // min
  int potRange1 = 7;
  int potRange2 = 2;
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

public:
  bool enableIndex = false;

  void runTankBase(); // Linear tank drive with mecanum on bumpers
  void runArcadeBase(); // Linear mecanum drive in arcade style
  void index(); // Indexer override on partner left bumpers
  void cIndex(); // Automatic index
  void openIntake(); // Open intakes on partner bottom right bumper
  void intake(); // Intake/ open/ auto intake, on top right bumper/ bottom right bumper/ up button
  void indexSense(); // Sets index ball position variables
  void intakeSense(); // Automatically intake balls
  void score(); // Score macros for one and two balls (driver bottom left bumper, bottom right bumper)
  void resetScoreNum(); // Reset score macro
  void checkPosition1(); // Auto intake when a ball is in position1
};