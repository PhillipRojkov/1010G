#pragma once
#include "vex.h"
#include <fstream> //Used for SD card functions

class InertialNavigation {
public:
  // Time keeping
  double prevTime = 0; // The Brain.timer(sec) value from the previous
                       // iteration
  double deltaT; // Time between iterations

  // Constants
  double g = 9.80665; // graviational constant
  double PI = M_PI;

  // Odometery return variables
  double h = 0; // Heading in degrees. Rotation clockwise is positive, does not
                // reset at 360
  double inertialX = 0; // x position on the field (side to side from starting
                        // position) in meters
  double inertialY = 0; // y position on the field (forwards/backwards from
                        // starting position) in meters

  double testX = 0;
  double testY = 0;

  // Variables used for calculating location
  double accelX;
  double accelY;
  double globalAccelX;
  double globalAccelY;
  double viX = 0;        // Initial global x velocity
  double viY = 0;        // Initial global y velocitY
  double prevViX = 0; // Previous global x velocity
  double prevViY = 0; // Previous global y velocity
  double prevGlobalAccelX = 0; // Previous global x acceleration
  double prevGlobalAccelY = 0; // Previous global y acceleration

  void computeLocation(); // Compute x and y global positions

  void printCoordinates(); // Print the x, y positions and rotation to the screen
  void writeToSD();   // Write debug data to the sd card
};