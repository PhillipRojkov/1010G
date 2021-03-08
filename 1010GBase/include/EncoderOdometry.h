#pragma once
#include "vex.h"
#include <fstream> //Used for SD card functions

class EncoderOdometry {
public:
  double PI = 3.14159265359;

  double encoderX;
  double encoderY;

  double wheelRadius = 2.75 / 2;
  double sL = 4.5; //distance from tracking centre to left odometry wheel
  double sR = 4.5; //distance from tracking centre to right odometry wheel
  double sM = 0.75; //distance from trakcing centre to middle odometry wheel
  double theta = 0;

  double prevEncoderM = 0;
  double prevEncoderL = 0;
  double prevEncoderR = 0;
  double prevTheta = 0;

  double deltaM;
  double deltaL;
  double deltaR;

  double deltaTheta;

  void computeLocation();
};