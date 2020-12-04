#include "vex.h"
#include <fstream>

class Odometry {
public:
  bool calibrating = true;

  double z = 0;

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

    
        // Cancel noise
        if (fabs(accelX) < 0.06) {
          accelX = 0.0;
        }
        if (fabs(accelY) < 0.06) {
          accelY = 0.0;
        }
      
    // Create global x and y vectors
    double globalAccelX = accelX * cos(IMU.rotation() * (PI / 180)) +
                          accelY * sin(IMU.rotation() * (PI / 180));
    double globalAccelY = accelX * sin(IMU.rotation() * (PI / 180)) +
                          accelY * cos(IMU.rotation() * (PI / 180));

    double deltaT = (Brain.timer(sec) - prevTime); // time between calculations (seconds)
    prevTime = Brain.timer(sec);

    if (!calibrating) {
      // Create initial VX and VY
     // viX += globalAccelX * deltaT;
     // viY += globalAccelY * deltaT;

      viX += accelX * deltaT;
      viY += accelY * deltaT;

      if (fabs(accelX) < 0.06 && fabs(prevAccelX) < 0.06) {
          viX = 0.0;
        }
        if (fabs(accelY) < 0.06 && fabs(prevAccelY) < 0.06) {
          viY = 0.0;
        }

/*
      if (absAvgDriveRPM() < 4) {
        viX = 0;
        viY = 0;
      }
*/
      // Use Kinematics Equation to conver to distance
     // x += viX * deltaT + 0.5 * globalAccelX * pow(deltaT, 2);
     // y += viY * deltaT + 0.5 * globalAccelY * pow(deltaT, 2);
      x += viX * deltaT;
      x = IMU.acceleration(zaxis) - sin(IMU.pitch() * (PI / 180));
     // y += viY * deltaT;
      y = IMU.acceleration(zaxis);
      z = viY;
    }

     //Average acceleration from last 2 calculations
   // accelX = (accelX + prevAccelX) / 2;
    prevAccelX = accelX;

   // accelY = (accelY + prevAccelY) / 2;
    prevAccelY = accelY;
    /*
        // Cancel noise on VX & VY
        if ((fabs(viX) < 0.4 && fabs(globalAccelX) < 0.06 * g) ||
            (fabs(globalAccelX) < 0.06 * g && absAvgDriveRPM() < 2)) {
          viX = 0.0;
        }
        if ((fabs(viY) < 0.4 && fabs(globalAccelY) < 0.06 * g) ||
            (fabs(globalAccelY) < 0.06 * g && absAvgDriveRPM() < 2)) {
          // Brain.Screen.drawCircle(300, 100, 100, red);
          viY = 0.0;
        } else {
          // Brain.Screen.drawCircle(300, 100, 100, green);
        }
        */
  
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
      file << ",";
      file << z;
      file << "\n";
      file.close();
    }
  }
};