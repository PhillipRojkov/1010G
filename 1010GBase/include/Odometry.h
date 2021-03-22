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

  double constantOfBadGyro = 1.006; //what?

  double PI = 3.14159265359;

  //Turning PID
  //(180/PI) is included because the terms on which these coefficients operate are in radians
  //Changing to degrees makes for parity between other turn functions
  double turnkP = 1 * (180/PI);
  double turnkD = 1.2 * (180/PI);
  double turnkI = 0 * (180/PI);

  double defaultTurnCompletionPoint = 4;
  double defaultMinDriveSpeed = 13;

  double defaultPositionError = 1; //Distance (inches) the robot needs to be from target position to end function
  double defaultTurnError = 0.06; //Rotation (rad) the robot needs to be from target heading to end function

  //Drive PID
  double defaultDrivekP = 4;
  double drivekD = 0;
  double drivekI = 0;

  double defaultStrafekP = 12;

  void setXY();

  void printCoordinates();

  void driveToPoint(double dX, double dY, double dH, double maxSpeed, double minDriveSpeed, double turnCompletionPoint, double drivekP, double strafekP, double positionError, double turnError);
  void driveToPoint(double dX, double dY, double dH, double maxSpeed, double minDriveSpeed, double turnCompletionPoint, double drivekP);
  void driveToPoint(double dX, double dY, double dH, double maxSpeed);
};