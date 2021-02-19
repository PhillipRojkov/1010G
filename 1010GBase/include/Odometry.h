#pragma once
#include "InertialNavigation.h"
#include "EncoderOdometry.h"

class Odometry {
  public: 
  double x;
  double y;

  void setXY();

  void finishCalibrating();
};