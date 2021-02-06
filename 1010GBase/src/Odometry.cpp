#pragma once
#include "vex.h"
#include <fstream>

class Odometry {
public:
  bool calibrating = true;

  // Time keeping
  double prevTime = 0; // The Brain.timer(sec) value from the previous
                       // iteration

  // Constants
  double g = 9.80665; // graviational constant
  double PI = 3.14159265359;

  // Odometery return variables
  double h = 0; // Heading in degrees. Rotation clockwise is positive, does not
                // reset at 360
  double x = 0; // x position on the field (side to side from starting position)
                // in meters
  double y = 0; // y position on the field (forwards/backwards from starting
                // position) in meters

  // Variables used for calculating location
  double viX = 0;
  double viY = 0;
  double prevAccelX = 0;
  double prevAccelY = 0;

  double absAvgDriveRPM() { // Returns average of all drive speeds
    return (fabs(DriveBL.velocity(rpm)) + fabs(DriveBR.velocity(rpm)) +
            fabs(DriveFL.velocity(rpm)) + fabs(DriveFR.velocity(rpm))) /
           4;
  }

  // Sets X and Y equal to the global position coordinates
  void computeLocation() {
    double accelX = IMU.acceleration(xaxis); // SIDE VECTOR - Right is positive
    double accelY =
        IMU.acceleration(zaxis); // FORWARD VECTOR - Forward is positive

    // Cancel out vertical component from potential tipping
    accelX = accelX - sin(IMU.roll() * (PI / 180));
    accelY = accelY - sin(IMU.pitch() * (PI / 180));

    //Convert acceleration to m/s2
    accelX *= g;
    accelY *= g;

    //avgAccelX is the average acceleration between the last iteration and this one
    double avgAccelX = (accelX + prevAccelX) / 2;
    double avgAccelY = (accelY + prevAccelY) / 2;
    
        // Cancel noise
        if (fabs(avgAccelX) < 0.06 * g) {
          avgAccelX = 0.0;
        }
        if (fabs(avgAccelY) < 0.06 * g) {
          avgAccelY = 0.0;
        }
      
    // Create global x and y vectors
    double globalAccelX = accelX * cos(IMU.rotation() * (PI / 180)) +
                          accelY * sin(IMU.rotation() * (PI / 180));
    double globalAccelY = accelX * sin(IMU.rotation() * (PI / 180)) +
                          accelY * cos(IMU.rotation() * (PI / 180));

    double deltaT = (Brain.timer(sec) - prevTime); // time between calculations (seconds)
    prevTime = Brain.timer(sec);

    if (!calibrating) {
      viX += avgAccelX * deltaT;
      viY += avgAccelY * deltaT;
      x = viX;
      y = viY;
    }
    prevAccelX = accelX;
    prevAccelY = accelY;
    writeToSD();
  }

  void screenPrint() {
    // Debug
    // Print X and Y field position
    Brain.Screen.setCursor(1, 2);
    Brain.Screen.print(x);
    Brain.Screen.setCursor(1, 18);
    Brain.Screen.print(y);

    // Print IMU rotation
    Brain.Screen.setCursor(4, 2);
    Brain.Screen.print(IMU.rotation());
  }

  void writeToSD() { // Write debug data to the sd card
    if (Brain.SDcard.isInserted()) {
      std::ofstream file("pirate.txt", std::ofstream::app);
      file << Brain.timer(sec);
      file << ",";
      file << x;
      file << ",";
      file << y;
      file << "\n";
      file.close();
    }
  }
};