#include "InertialNavigation.h"

void InertialNavigation::computeLocation() {
  accelX = -IMU.acceleration(xaxis); // SIDE VECTOR - Right is positive
  accelY = IMU.acceleration(yaxis); // FORWARD VECTOR - Forward is positive

  // Cancel out vertical component from potential tipping
  accelX = accelX + sin(IMU.pitch() * (PI / 180));
  accelY = accelY + sin(IMU.roll() * (PI / 180));

  // Convert acceleration to m/s2
  accelX *= g;
  accelY *= g;

  // Create global x and y vectors
  globalAccelX = accelX * cos(IMU.rotation() * (PI / 180)) + accelY * sin(IMU.rotation() * (PI / 180));
  globalAccelY = accelX * sin(IMU.rotation() * (PI / 180)) - accelY * cos(IMU.rotation() * (PI / 180));

  deltaT = Brain.timer(sec) - prevTime; // time between calculations (seconds)

  //Trapezoidal approximation for integrating global x and y velocity
  viX += (globalAccelX + prevGlobalAccelX) / 2 * deltaT;
  viY += (globalAccelY + prevGlobalAccelY) / 2 * deltaT;

  //Trapezoidal approximation for integrating global x and y position
  inertialX += (viX + prevViX) / 2 * deltaT;
  inertialY += (viY + prevViY) / 2 * deltaT;

  inertialX = globalAccelX;
  inertialY = globalAccelX;
  testX = viX;
  testY = viY;

  //Set previous velocity, acceleration and time variables
  prevViX = viX;
  prevViY = viY;
  prevGlobalAccelX = globalAccelX;
  prevGlobalAccelY = globalAccelY;
  prevTime = Brain.timer(sec);

  writeToSD();
}

void InertialNavigation::printCoordinates() {
  // Debug
  // Print X and Y field position
  Brain.Screen.setCursor(1, 2);
  Brain.Screen.print(inertialX);
  Brain.Screen.setCursor(1, 18);
  Brain.Screen.print(inertialY);
  // Print IMU rotation
  Brain.Screen.setCursor(4, 2);
  Brain.Screen.print(IMU.rotation());
}

void InertialNavigation::writeToSD() {
  if (Brain.SDcard.isInserted()) { // Write only if the card is inserted
    std::ofstream file(
        "pirate.txt",
        std::ofstream::app); // Create or open a file called pirate.txt
    // Write format:
    // time(sec),x(m),y(m),accelX(m/s^2),deltaT(s)
    file << Brain.timer(sec); // Write the current time to pirate.txt
    file << ",";
    file << inertialX; // Write the current x position
    file << ",";
    file << inertialY; // Write the current y position
    file << ",";
    file << testX;
    file << ",";
    file << testY;
    file << ",";
    file << IMU.rotation();
    file << ",";
    file << deltaT; //Write deltaT
    file << "\n";      // Create a new line
    file.close();      // Close the file
  }
}