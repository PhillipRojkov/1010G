#pragma once
#include "vex.h"
#include <fstream> //Used for SD card functions

class EncoderOdometry {
  public:
  double encoderX;
  double encoderY;

  void computeLocation();
};