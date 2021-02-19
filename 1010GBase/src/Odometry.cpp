#include "Odometry.h"

EncoderOdometry encoderOdometry;
InertialNavigation inertialNavigation;

void Odometry::setXY() {
  encoderOdometry.computeLocation();
  inertialNavigation.computeLocation();

  x = (encoderOdometry.encoderX + inertialNavigation.inertialX) / 2;
}

void Odometry::finishCalibrating() {
  inertialNavigation.calibrating = false;
}