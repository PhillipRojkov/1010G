#include "Odometry.h"

double theta = 0;
void Odometry::setXY() {
  encoderOdometry.computeLocation();
  encoderOdometry.printCoordinates();

  x = encoderOdometry.encoderX;
  y = encoderOdometry.encoderY;
  theta = encoderOdometry.theta; // In radians
}

void Odometry::pursuit(double dX, double dY, double speed, double drivekP, double positionError, double turnError, double turnkP) {
  //Set up delta values
  double deltaX = dX - x;
  double deltaY = dY - y;
    //Desired heading in radians for driving to point (dX, dY)
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
  
    //Stop the robot from doing all the 360s by calculating the shortest path to the turn
    if (DirectionOfMovement - theta > PI) {
      DirectionOfMovement -= PI;
    } else if (DirectionOfMovement - theta < -PI) {
      DirectionOfMovement += 2 * PI;
    }

  double prevError = 0;
  double turnIntegral = 0;
  double t = 0;
  //Turn to face desired position
  if (speed > 0) { //Driving forwards, robot turns to directionOfMovement
    while (t < turnMargin) {
      //Turning PID
      double error = DirectionOfMovement - theta;
      double derivative = error - prevError;
      prevError = error;
      turnIntegral += error;

      //double kD = ((slowTurnkP + turnkP)/2 - (turnkP - slowTurnkP)/2 * cos(2*error)); 
      double kD = 0;
      if (fabs(error) < 50 * (PI/180)) {
        kD = shortTurnkD;
      } else {
        kD = turnkD;
      }

      DriveFL.spin(fwd, error * turnkP + derivative * kD + turnIntegral * turnkI, pct);
      DriveBL.spin(fwd, error * turnkP + derivative * kD + turnIntegral * turnkI, pct);
      DriveFR.spin(fwd, -(error * turnkP + derivative * kD + turnIntegral * turnkI), pct);
      DriveBR.spin(fwd, -(error * turnkP + derivative * kD + turnIntegral * turnkI), pct);

      if (fabs(DirectionOfMovement - theta) <= turnError) {
        t += 10;
      } else {
        t = 0;
      }
      wait(10, msec);
    }
  } else { //Driving backwards, robot faces in opposite direction of directionOfMovement
    while (t < turnMargin) {
      //Turning PID
      double error = DirectionOfMovement - (theta + PI);
      double derivative = error - prevError;
      prevError = error;
      turnIntegral += error;

      double kD;
      if (fabs(error) < 50 * (PI/180)) {
        kD = shortTurnkD;
      } else {
        kD = turnkD;
      }

      DriveFL.spin(fwd, error * turnkP + derivative * kD + turnIntegral * turnkI, pct);
      DriveBL.spin(fwd, error * turnkP + derivative * kD + turnIntegral * turnkI * turnkP, pct);
      DriveFR.spin(fwd, -(error * turnkP + derivative * kD + turnIntegral * turnkI * turnkP), pct);
      DriveBR.spin(fwd, -(error * turnkP + derivative * kD + turnIntegral * turnkI * turnkP), pct);

      if (fabs(DirectionOfMovement - (theta + PI)) <= turnError) {
        t += 10;
      } else {
        t = 0;
      }
      wait(10, msec);
    }
  }
  DriveFL.stop(brake);
  DriveBL.stop(brake);
  DriveFR.stop(brake);
  DriveBR.stop(brake);

  wait(100, msec);

  //Find slope of the line from initial position to desired position on the global cartesian plane
  //Usually relative to x, ie. ∆Y/∆X but can be relative to y, ie. ∆X/∆Y if ∆X = 0
  //If iSlope is relative to y, swapAxis = true, therefore fSlope will also be relative to y
  double iSlope = 0;
  bool swapAxis = dX == x; //True if dX == x (divide by zero problem exists)
  if (!swapAxis) { //Avoid divide by zero
    iSlope = (dY - y) / (dX - x);
  }
  // iSlope is 0 (relative to y) when dX == x, so no else statement needed

  double s = speed; //Speed variable
  double prevTrackingError = 0;
  double trackingIntegral = 0;
  /*distanceLeft = sqrt(pow(deltaX, 2) + pow(deltaY, 2)); //Scalar point to point distance remaining
  double prevDistanceLeft = distanceLeft;
  double distanceDerivative = distanceLeft - prevDistanceLeft;*/

  double c = 0;
  double d = 0;
  double perpDistanceLeft = 10;

  while (perpDistanceLeft > positionError) { //Movement loop
    //Slope of line from current position to desired position
    double fSlope = 0; //If checks below fail, fSlope = 0
    if (!swapAxis) { //iSlope is relative to x
      if (dX != x) { //Avoid divide by zero
        fSlope = (dY - y) / (dX - x);
      } 
    } else { //iSlope is relative to y
      if (dY != y) { //avoid divide by zero
        fSlope = (dX - x) / (dY - y);
      }
    }

    if (iSlope == 0 && swapAxis) {
      c = x + 0.001; //Janky fix so that the pow() function doesnt break
      d = dY;
    } else if (iSlope == 0 && !swapAxis) {
      c = dX;
      d = y + 0.001; //Janky fix so that the pow() function doesnt break
    } else {
      d = (-y/iSlope + x - iSlope * dY - dX) / (-iSlope - 1/iSlope);
      c = (d - y + iSlope * x) / iSlope;
    }

    perpDistanceLeft = sqrt(pow((c - x), 2) + pow((d - y), 2));

    //Tracking PID
    double trackingError = (fSlope - iSlope); //How far away is the robot from being on the correct course
    double trackingDerivative = trackingError - prevTrackingError;
    prevTrackingError = trackingError;
    trackingIntegral += trackingError;

    //Set speed varible s
    if (speed > 0) { //Driving forwards
      if (fabs(perpDistanceLeft * drivekP) > speed && perpDistanceLeft * drivekP > 0) {
        s = speed;
      } else {
        s = perpDistanceLeft * drivekP;
      }
    } else { //Driving backwards
      if (-fabs(perpDistanceLeft * drivekP) < speed && perpDistanceLeft * drivekP > 0) {
        s = speed;
      } else {
        s = -perpDistanceLeft * drivekP;
      }
    }

    double trackingkP = defaultTrackingkP;
    DriveFL.spin(forward, s + (trackingError * trackingkP + trackingDerivative * trackingkD + trackingIntegral * trackingkI), pct);
    DriveBL.spin(forward, s + (trackingError * trackingkP + trackingDerivative * trackingkD + trackingIntegral * trackingkP), pct);
    DriveFR.spin(forward, s - (trackingError * trackingkP + trackingDerivative * trackingkD + trackingIntegral * trackingkI), pct);
    DriveBR.spin(forward, s - (trackingError * trackingkP + trackingDerivative * trackingkD + trackingIntegral * trackingkI), pct);

    wait(10, msec);
  }

  DriveFL.stop(brake);
  DriveBL.stop(brake);
  DriveFR.stop(brake);
  DriveBR.stop(brake);
}

void Odometry::pursuit(double dX, double dY, double speed, double drivekP, double positionError, double turnError) {
  pursuit(dX, dY, speed, drivekP, defaultPositionError, turnError, defaultTurnkP);
}

void Odometry::pursuit(double dX, double dY, double speed, double drivekP) {
  pursuit(dX, dY, speed, drivekP, defaultPositionError, defaultTurnError, defaultTurnkP);
}

void Odometry::pursuit(double dX, double dY, double speed) {
  pursuit(dX, dY, speed, defaultDrivekP, defaultPositionError, defaultTurnError, defaultTurnkP);
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
    double fL = P1/s + deltaH * defaultTurnkP + turnIntegral * turnkI + turnDerivative * turnkD;
    double fR = P2/s - deltaH * defaultTurnkP - turnIntegral * turnkI - turnDerivative * turnkD;
    double bL = P2/s + deltaH * defaultTurnkP + turnIntegral * turnkI + turnDerivative * turnkD;
    double bR = P1/s - deltaH * defaultTurnkP - turnIntegral * turnkI - turnDerivative * turnkD; 

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

