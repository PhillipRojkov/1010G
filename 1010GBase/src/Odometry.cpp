#include "Odometry.h"

double theta = 0;
void Odometry::setXY() {
  encoderOdometry.computeLocation();

  x = encoderOdometry.encoderX;
  y = encoderOdometry.encoderY;
  theta = encoderOdometry.theta;
}

void Odometry::pursuit(double dX, double dY, double speed, double drivekP, double positionError, double turnError) {
  //Set up delta values
  double deltaX = dX - x;
  double deltaY = dY - y;

  //Initial position
  double initialX = x;
  double initialY = y;
  
  double distanceLeft = sqrt(pow(deltaX, 2) + pow(deltaY, 2)); //scalar point to point distance remaining
    //Desired heading for driving to point
    double DirectionOfMovement;
    //If statements avoid divide by zero problem for moving directly along the x and y axis
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
    }

  double prevError = 0;
  double turnIntegral = 0;
  //Turn to face desired position
  while (fabs(DirectionOfMovement - theta) > turnError) {
    double error = DirectionOfMovement - theta;
    double derivative = error - prevError;
    turnIntegral += error;

    DriveFL.spin(fwd, error * turnkP + derivative * turnkD + turnIntegral * turnkI, pct);
    DriveBL.spin(fwd, error * turnkP + derivative * turnkD + turnIntegral * turnkI * turnkP, pct);
    DriveFR.spin(fwd, -(error * turnkP + derivative * turnkD + turnIntegral * turnkI * turnkP), pct);
    DriveBR.spin(fwd, -(error * turnkP + derivative * turnkD + turnIntegral * turnkI * turnkP), pct);
  }
  //Find slope of line from initial position to desired position
  double iSlope = (dY - initialY) / (dX - initialX);
  double s = speed;
  double prevTrackingError = 0;
  double trackingIntegral = 0;
  while (distanceLeft > positionError) {
    //Slope of line from current position to desired position
    double fSlope = (dY - y) / (dX - x);
    double trackingError = (fSlope - iSlope);
    double trackingDerivative = trackingError - prevTrackingError;
    prevTrackingError = trackingError;
    trackingIntegral += trackingError;
    
    if (fabs(distanceLeft * drivekP) > speed) {
      s = speed;
    } else {
      s = distanceLeft * drivekP;
    }

    double trackingkP = defaultTrackingkP;
    DriveFL.spin(forward, s + trackingError * trackingkP + trackingDerivative * trackingkD + trackingIntegral * trackingkI, pct);
    DriveBL.spin(forward, s + trackingError * trackingkP + trackingDerivative * trackingkD + trackingIntegral * trackingkP, pct);
    DriveBR.spin(forward, s - (trackingError * trackingkP + trackingDerivative * trackingkD + trackingIntegral * trackingkI), pct);
    DriveBL.spin(forward, s - (trackingError * trackingkP + trackingDerivative * trackingkD + trackingIntegral * trackingkI), pct);
  }
  DriveFL.stop(brake);
  DriveBL.stop(brake);
  DriveFR.stop(brake);
  DriveBR.stop(brake);
}

void Odometry::pursuit(double dX, double dY, double speed, double drivekP) {
  pursuit(dX, dY, speed, drivekP, defaultPositionError, defaultTurnError);
}

void Odometry::pursuit(double dX, double dY, double speed) {
  pursuit(dX, dY, speed, defaultDrivekP, defaultPositionError, defaultTurnError);
}

 //Turn Completion point: At what point in the translation should the turn be completed (1 is for at the end, 2 is for at the midpoint, 4 is at the quarterpoint, etc.)
void Odometry::driveToPoint(double dX, double dY, double dH, double maxSpeed, double minDriveSpeed, double turnCompletionPoint, double drivekP, double strafekP, double positionError, double turnError) {
  dH *= (PI / 180); //Convert dH to radians

  double h = ((IMUL.rotation() + Brain.timer(sec) * gyroDriftL) * constantOfBadGyroL + (IMUR.rotation() + Brain.timer(sec) * gyroDriftR) * constantOfBadGyroR)/2 * (PI / 180); //heading in radians

  //Set up delta values
  double deltaX = dX - x;
  double deltaY = dY - y;
  double deltaH = dH - h;

  double speed;
  double distanceLeft = sqrt(pow(deltaX, 2) + pow(deltaY, 2)); //scalar point to point distance remaining
  double initialDistanceLeft = distanceLeft; //Calculated only at the beginning of this function call

  double percentMoveCompletion = (initialDistanceLeft - distanceLeft) / initialDistanceLeft; //Starts at zero, approaches one as the robot gets closer to (dX, dY)
  double initialHeading = h;
  double newDesiredHeading = initialHeading + (dH - initialHeading) * percentMoveCompletion; //gets closer to dH as percentTurn approaches 1
  double turnIntegral = 0;
  double prevDeltaH = deltaH;

  // Run when the robot is far away from desired point and heading
  while (distanceLeft > positionError || fabs(dH - h) > turnError) {
    h = ((IMUL.rotation() + Brain.timer(sec) * gyroDriftL) * constantOfBadGyroL + (IMUR.rotation() + Brain.timer(sec) * gyroDriftR) * constantOfBadGyroR)/2 * (PI / 180); //heading in radians
    deltaX = dX - x;
    deltaY = dY - y;

    percentMoveCompletion = fabs((initialDistanceLeft - distanceLeft) / initialDistanceLeft); //Starts at 0, approaches 1 as the robot gets closer to its desired position
    percentMoveCompletion = fmin(percentMoveCompletion, 1 / turnCompletionPoint);
    newDesiredHeading = initialHeading + (dH - initialHeading) * turnCompletionPoint * percentMoveCompletion; //gets closer to dH as percentTurn approaches 1
    // Turning PID values
    deltaH = newDesiredHeading - h; //Turn error
    turnIntegral += deltaH;
    double turnDerivative = deltaH - prevDeltaH;
    prevDeltaH = deltaH;

    distanceLeft = sqrt(pow(deltaX, 2) + pow(deltaY, 2)); //scalar point to point distance remaining

    // Calculate local direction of movement
    // Traveling straight is 0 degrees, travelling to the right is 90 degrees, etc.
    double DirectionOfMovement;
    //If statements avoid divide by zero problem for moving directly along the x and y axis
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
    }
    DirectionOfMovement -= h; // Make local to robot orientation

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
    //Set motor speeds
    double fL = P1/s + deltaH * turnkP + turnIntegral * turnkI + turnDerivative * turnkD;
    double fR = P2/s - deltaH * turnkP - turnIntegral * turnkI - turnDerivative * turnkD;
    double bL = P2/s + deltaH * turnkP + turnIntegral * turnkI + turnDerivative * turnkD;
    double bR = P1/s - deltaH * turnkP - turnIntegral * turnkI - turnDerivative * turnkD; 

    DriveFL.spin(forward, fL, pct);
    DriveFR.spin(forward, fR, pct);
    DriveBL.spin(forward, bL, pct);
    DriveBR.spin(forward, bR, pct);

    //Brake motors if they are travelling very slow (assists with 45 degree directionOfMovement)
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
  } //End of while loop
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

