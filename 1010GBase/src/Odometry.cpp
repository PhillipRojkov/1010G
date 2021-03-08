#include "Odometry.h"

void Odometry::setXY() {
  encoderOdometry.computeLocation();
  /*inertialNavigation.computeLocation();

  x = inertialNavigation.inertialX;
  y = inertialNavigation.inertialY;*/
  x = encoderOdometry.encoderX;
  y = encoderOdometry.encoderY;
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
  //Print odometry heading (radians)
  Brain.Screen.setCursor(5, 2);
  Brain.Screen.print(encoderOdometry.theta);
}

void Odometry::driveToPoint(double dX, double dY, double dH) {
  double h = IMU.rotation() * (PI / 180);

  double deltaX = dX - x;
  double deltaY = dY - y;
  double deltaH = dH - h;

  double speed = 80;

  while (sqrt(pow(deltaX, 2) + pow(deltaY, 2)) > 1) {
    deltaX = dX - x;
    deltaY = dY - y;
    deltaH = dH - h;

    double DirectionOfMovement = atan(deltaY / deltaX);

    double P1 = -cos(DirectionOfMovement + PI / 4) / cos(PI / 4);
    double P2 = -cos(DirectionOfMovement + 3 * PI / 4) / cos(PI / 4);

    double s = fmax(P1, P2) / speed;

    double fL = P2/s;
    double fR = P1/s;
    double bL = P1/s;
    double bR = P2/s;

    DriveFL.spin(forward, fL, pct);
    DriveFR.spin(forward, fR, pct);
    DriveBL.spin(forward, bL, pct);
    DriveBR.spin(forward, bR, pct);

    if (fabs(fL) < 2) {
      DriveFL.stop(brake);
    }
    if (fabs(fR) < 2) {
      DriveFR.stop(brake);
    }
    if (fabs(bL) < 2) {
      DriveBL.stop(brake);
    }
    if (fabs(bR) < 2) {
      DriveBR.stop(brake);
    }
  }
}