#include "Odometry.h"

double theta = 0;
void Odometry::setXY() {
  encoderOdometry.computeLocation();
  /*inertialNavigation.computeLocation();

  x = inertialNavigation.inertialX;
  y = inertialNavigation.inertialY;*/
  x = encoderOdometry.encoderX;
  y = encoderOdometry.encoderY;
  theta = encoderOdometry.theta;
}

void Odometry::printCoordinates() {
  // Print X and Y field position
  Brain.Screen.setCursor(1, 2);
  Brain.Screen.print(x);
  Brain.Screen.setCursor(1, 9);
  Brain.Screen.print(y);
  // Print IMU rotations
  Brain.Screen.setCursor(1, 18);
  Brain.Screen.print((IMUR.rotation() + Brain.timer(sec) * gyroDriftR) * constantOfBadGyroR);
  Brain.Screen.setCursor(2, 18);
  Brain.Screen.print((IMUL.rotation() + Brain.timer(sec) * gyroDriftL) * constantOfBadGyroL);
  Brain.Screen.setCursor(3, 18);
  Brain.Screen.print(theta * (180/PI));
}

 //Turn Completion point: At what point in the translation should the turn be completed (1 is for at the end, 2 is for at the midpoint, 4 is at the quarterpoint, etc.)
void Odometry::driveToPoint(double dX, double dY, double dH, double maxSpeed, double minDriveSpeed, double turnCompletionPoint, double drivekP, double strafekP, double positionError, double turnError) {
  dH *= (PI / 180); //Convert to radians

  double h = ((IMUL.rotation() + Brain.timer(sec) * gyroDriftL) * constantOfBadGyroL + (IMUR.rotation() + Brain.timer(sec) * gyroDriftR) * constantOfBadGyroR)/2 * (PI / 180); //heading in radians

  double deltaX = dX - x;
  double deltaY = dY - y;
  double deltaH = dH - h;

  double speed;
  double distanceLeft = sqrt(pow(deltaX, 2) + pow(deltaY, 2)); //scalar point to point distance remaining
  double initialDistance = distanceLeft; //Calculate only at the beginning of this function call

  double percentMoveCompletion = (initialDistance - distanceLeft) / initialDistance; //Starts at zero, approaches one as the robot gets closer to (dX, dY)
  double initialHeading = h;
  double newDesiredHeading = initialHeading + (dH - initialHeading) * percentMoveCompletion; //gets closer to dH as percentTurn approaches 1
  double turnIntegral = 0;
  double prevDeltaH = deltaH;

  // Run when the robot is far away from desired point and heading
  while (distanceLeft > positionError || fabs(dH - h) > turnError) {
    h = ((IMUL.rotation() + Brain.timer(sec) * gyroDriftL) * constantOfBadGyroL + (IMUR.rotation() + Brain.timer(sec) * gyroDriftR) * constantOfBadGyroR)/2 * (PI / 180);
    deltaX = dX - x;
    deltaY = dY - y;

    percentMoveCompletion = fabs((initialDistance - distanceLeft) / initialDistance); //Starts at zero, approaches one as the robot gets closer to (dX, dY)
    percentMoveCompletion = fmin(percentMoveCompletion, 1 / turnCompletionPoint);
    newDesiredHeading = initialHeading + (dH - initialHeading) * turnCompletionPoint * percentMoveCompletion; //gets closer to dH as percentTurn approaches 1
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

    //Set speed
    //Average kP - (strafe - drive)/2 * cos(2x)
    speed = distanceLeft * ((drivekP + strafekP)/2 - (strafekP - drivekP)/2 * cos(2*DirectionOfMovement));
    if (speed > maxSpeed) { //clamp speed between maxSpeed and minSpeed
      speed = maxSpeed;
    } 
    //Set minDriveSpeed
    //Average average minSpeed - (strafe - drive)/2 * cos(2x)
    double minSpeed = (minDriveSpeed + defaultMinStrafeSpeed)/2 - (defaultMinStrafeSpeed - minDriveSpeed)/2 * cos(2*DirectionOfMovement);
    if (speed < minSpeed) {
      speed = minSpeed;
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

void Odometry::driveToPoint(double dX, double dY, double dH, double maxSpeed) {
  driveToPoint(dX, dY, dH, maxSpeed, defaultMinDriveSpeed, defaultTurnCompletionPoint, defaultDrivekP, defaultStrafekP, defaultPositionError, defaultTurnError);
}

void Odometry::driveToPoint(double dX, double dY, double dH, double maxSpeed, double minDriveSpeed, double turnCompletionPoint, double drivekP) {
  driveToPoint(dX, dY, dH, maxSpeed, minDriveSpeed, turnCompletionPoint, drivekP, defaultStrafekP, defaultPositionError, defaultTurnError);
}