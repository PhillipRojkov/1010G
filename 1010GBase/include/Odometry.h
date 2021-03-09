#pragma once
#include "EncoderOdometry.h"
#include "InertialNavigation.h"

class Odometry {
private:
  EncoderOdometry encoderOdometry;
  InertialNavigation inertialNavigation;
public:
  double x;
  double y;

  double PI = 3.14159265359;

  //Turning PID
  //(180/PI) is included because the terms on which these coefficients operate are in radians
  //Changing to degrees makes for parity between other turn functions
  double turnkP = 0.5 * (180/PI);
  double turnkD = 1.5 * (180/PI);
  double turnkI = 0 * (180/PI);

  double turnCompletionPoint = 4; //At what point in the translation should the turn be completed (1 is for at the end, 2 is for at the midpoint, 4 is at the quarterpoint, etc.)

  //Drive PID
  double drivekP = 4;
  double drivekD = 0;
  double drivekI = 0;

  double minDriveSpeed = 10;

  void setXY();

  void printCoordinates();

  void driveToPoint(double dX, double dY, double dH, double maxSpeed);
};