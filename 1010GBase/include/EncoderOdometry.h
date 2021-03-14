#pragma once
#include "vex.h"
#include <fstream> //Used for SD card functions

class EncoderOdometry {
public:
  double PI = 3.14159265359;

  double encoderX;
  double encoderY;

  double wheelRadius = 2.75 / 2;
  double offsetL = 4.375; //distance from tracking centre to left odometry wheel
  double offsetR = 4.375; //distance from tracking centre to right odometry wheel
  double offsetS = 1.5; //distance from trakcing centre to middle odometry wheel
  double theta = 0;

  double prevEncoderS = 0;
  double prevEncoderL = 0;
  double prevEncoderR = 0;
  double prevTheta = 0;

  double deltaS;
  double deltaL;
  double deltaR;

  double deltaTheta;
  double deltaX;
  double deltaY;

  void computeLocation();
};