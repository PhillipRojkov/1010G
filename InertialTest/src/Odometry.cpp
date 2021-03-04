#include "Odometry.h"

void Odometry::setXY() {
  //encoderOdometry.computeLocation();
  inertialNavigation.computeLocation();

  x = inertialNavigation.inertialX;
  y = inertialNavigation.inertialY;
}

void Odometry::printCoordinates() {
  // Print X and Y field position
  Brain.Screen.setCursor(1, 2);
  Brain.Screen.print(x);
  Brain.Screen.setCursor(1, 18);
  Brain.Screen.print(y);
  // Print IMU rotation
  Brain.Screen.setCursor(4, 2);
  Brain.Screen.print(IMU.rotation());
}