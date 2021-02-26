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

  void finishCalibrating();

  void setXY();
};