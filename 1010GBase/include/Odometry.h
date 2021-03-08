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

  void setXY();

  void printCoordinates();

  void driveToPoint(double dX, double dY, double dH);
};