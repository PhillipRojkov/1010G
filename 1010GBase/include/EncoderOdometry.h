#pragma once
#include "vex.h"
#include <fstream> //Used for SD card functions

class EncoderOdometry {
public:
// Essentially VEX Inertial sensor is kinda garbage, ours picks up ~5 
// deg of error for every 720 degrees of rotation. Multiply IMU heading
// by this variable to remove the undershoot. Why can't vex just calibrate
// the sensor from the factory? beats me.
// APRIL 1 2020, this value is now 1 because changing the mounting
// orientation of the sensor fixed it. Imagine having good sensors
// APRIL 2 2020, the previous sensor completely crapped out,
// 15 degrees of error over 360 degrees of rotation. Replaced sensor
// It now works better
  double constantOfBadGyroL = 1.0082;
  double constantOfBadGyroR = 1.0091;

// Variables to counter gyro drift cause vex gyro bad lol
// Units are degrees / second
  double gyroDriftL = 0.002;
  double gyroDriftR = -0.0014;

  double diagonalMvmtOffsetX = -0.04; //Offset applied to deltaX when the robot is travelling -0.04
  //at 45 degrees relative to local forward
  double diagonalMvmtOffsetY = 0.04; //Offset applied to deltaX when the robot is travelling 0.04
  //at 45 degrees relative to local forward

  double PI = 3.14159265359;

  double encoderX;
  double encoderY;

  double wheelRadiusL = 2.775 / 2;
  double wheelRadiusR = 2.775 / 2;
  double middleWheelRadius = 2.775 / 2;
  double offsetL = 4; //distance from tracking centre to left odometry wheel
  double offsetR = 4; //distance from tracking centre to right odometry wheel
  double offsetS = 2.5; //distance from trakcing centre to middle odometry wheel
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

  double deltaT;
  double prevTime = 0;

  void computeLocation();

  void writeToSD();   // Write debug data to the sd card
};