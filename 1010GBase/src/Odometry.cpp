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
  Brain.Screen.setCursor(1, 9);
  Brain.Screen.print(y);
  // Print IMU rotation
  Brain.Screen.setCursor(1, 18);
  Brain.Screen.print(IMU.rotation());
}

 //At what point in the translation should the turn be completed (1 is for at the end, 2 is for at the midpoint, 4 is at the quarterpoint, etc.)
void Odometry::driveToPoint(double dX, double dY, double dH, double maxSpeed, double minDriveSpeed, double turnCompletionPoint, double drivekP) {

  dH *= (PI / 180); //Convert to radians

  double h = IMU.rotation() * (PI / 180); //heading in radians

  double deltaX = dX - x;
  double deltaY = dY - y;
  double deltaH = dH - h;

  double distanceLeft = sqrt(pow(deltaX, 2) + pow(deltaY, 2)); //scalar point to point distance remaining
  double initialDistance = distanceLeft; //Calculate only at the beginning of this function call
  double percentTurn =  (initialDistance - distanceLeft) / initialDistance; //Starts at zero, approaches one as the robot gets closer to (dX, dY)
  double initialHeading = h;

  double newDesiredHeading = initialHeading + (dH - initialHeading) * percentTurn; //gets closer to dH as percentTurn approaches 1

  double speed;

  double turnIntegral = 0;

  double prevDeltaH = deltaH;

  // Run when the robot is far away from desired point and heading
  while (distanceLeft > 1 || fabs(dH - h) > 0.05) {
    speed = distanceLeft * drivekP;
    if (speed > maxSpeed) { //clamp speed between maxSpeed and minSpeed
      speed = maxSpeed;
    } 
    if (speed < minDriveSpeed) {
      speed = minDriveSpeed;
    }

    h = IMU.rotation() * (PI / 180);
    deltaX = dX - x;
    deltaY = dY - y;

    percentTurn = fabs((initialDistance - distanceLeft) / initialDistance); //Starts at zero, approaches one as the robot gets closer to (dX, dY)
    percentTurn = fmin(percentTurn, 1 / turnCompletionPoint);
    newDesiredHeading = initialHeading + (dH - initialHeading) * turnCompletionPoint * percentTurn; //gets closer to dH as percentTurn approaches 1
    deltaH = newDesiredHeading - h; //Turn error
    turnIntegral += deltaH;
    double turnDerivative = deltaH - prevDeltaH;
    prevDeltaH = deltaH;

    distanceLeft = sqrt(pow(deltaX, 2) + pow(deltaY, 2)); //scalar point to point distance remaining

    double DirectionOfMovement;

    //If statements avoid divide by zero problem for moving along the x and y axis
    if (fabs(deltaX) == 0) {
      if (deltaY > 0) {
        DirectionOfMovement = 0;
      } else {
        DirectionOfMovement = PI;
      }
    } else if (fabs(deltaY) == 0) {
      if (deltaX > 0) {
        DirectionOfMovement = PI/2;
      } else {
        DirectionOfMovement = -PI/2;
      }
    } else {
      //Calculate Direction of Movement using inverse tan
      if (deltaX > 0 && deltaY > 0) { //Quadrant 1
        DirectionOfMovement = atan(deltaX / deltaY);
      } else if (deltaX < 0 && deltaY > 0) { //Quadrant 2
        DirectionOfMovement = atan(deltaX / deltaY);
      } else if (deltaX < 0 && deltaY < 0) { //Quadrant 3
        DirectionOfMovement = -PI + atan(deltaX / deltaY);
      } else { //Quadrant 4
        DirectionOfMovement = PI + atan(deltaX / deltaY);
      }
      DirectionOfMovement -= h; // Make local to robot orientation
    }

    double P1 = -cos(DirectionOfMovement + 3 * PI / 4) / cos(PI / 4); //Diagonal set 1 percentage
    double P2 = -cos(DirectionOfMovement + 5 * PI / 4) / cos(PI / 4); //Diagonal set 2 percentage

    double s = fmax(fabs(P1), fabs(P2)) / speed; //speed limiter
    double fL = P1/s + deltaH * turnkP + turnIntegral * turnkI + turnDerivative * turnkD;
    double fR = P2/s - deltaH * turnkP - turnIntegral * turnkI - turnDerivative * turnkD;
    double bL = P2/s + deltaH * turnkP + turnIntegral * turnkI + turnDerivative * turnkD;
    double bR = P1/s - deltaH * turnkP - turnIntegral * turnkI - turnDerivative * turnkD; 

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
  DriveFL.stop(brake);
  DriveFR.stop(brake);
  DriveBL.stop(brake);
  DriveBR.stop(brake);
}