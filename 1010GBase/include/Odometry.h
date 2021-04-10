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
  double gyroDriftL = 0.001;
  double gyroDriftR = -0.0012;

  double PI = 3.14159265359;

  //Turning PID
  //(180/PI) is included because the terms on which these coefficients operate are in radians
  //Changing to degrees makes for parity between other turn functions
  double turnkP = 1 * (180/PI);
  double turnkD = 0.5 * (180/PI);
  double turnkI = 0 * (180/PI);

  double defaultTurnCompletionPoint = 4;
  double defaultMinDriveSpeed = 12;
  double defaultMinStrafeSpeed = 20;

  double defaultPositionError = 1.1; //Distance (inches) the robot needs to be from target position to end function
  double defaultTurnError = 0.07; //Rotation (rad) the robot needs to be from target heading to end function

  //Drive PID
  double defaultDrivekP = 6;
  double drivekD = 0;
  double drivekI = 0;

  double defaultStrafekP = 14;

  void setXY();

  void printCoordinates();

  void driveToPoint(double dX, double dY, double dH, double maxSpeed, double minDriveSpeed, double turnCompletionPoint, double drivekP, double strafekP, double positionError, double turnError);
  void driveToPoint(double dX, double dY, double dH, double maxSpeed, double minDriveSpeed, double turnCompletionPoint, double drivekP);
  void driveToPoint(double dX, double dY, double dH, double maxSpeed);
};