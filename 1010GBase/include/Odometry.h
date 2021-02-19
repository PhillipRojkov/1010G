#pragma once
#include "EncoderOdometry.h"
#include "InertialNavigation.h"

class Odometry {
public:
  double x;
  double y;

  void finishCalibrating();

  void setXY();
};