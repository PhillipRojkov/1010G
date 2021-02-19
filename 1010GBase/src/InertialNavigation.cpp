#include "InertialNavigation.h"

void InertialNavigation::computeLocation() {
  double accelX = IMU.acceleration(xaxis); // SIDE VECTOR - Right is positive
  double accelY =
      IMU.acceleration(zaxis); // FORWARD VECTOR - Forward is positive

  // Cancel out vertical component from potential tipping
  accelX = accelX - sin(IMU.roll() * (PI / 180));
  accelY = accelY - sin(IMU.pitch() * (PI / 180));

  // Convert acceleration to m/s2
  accelX *= g;
  accelY *= g;

  // avgAccelX is the average acceleration between the last iteration and this
  // one
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

  double deltaT =
      (Brain.timer(sec) - prevTime); // time between calculations (seconds)
  prevTime = Brain.timer(sec);

  if (!calibrating) {
    viX += avgAccelX * deltaT;
    viY += avgAccelY * deltaT;
    inertialX = viX;
    inertialY = viY;
  }
  prevAccelX = accelX;
  prevAccelY = accelY;
  writeToSD();
}

void InertialNavigation::screenPrint() {
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
    // time(sec),x(m),y(m)
    file << Brain.timer(sec); // Write the current time to pirate.txt
    file << ",";
    file << inertialX; // Write the current x position
    file << ",";
    file << inertialY; // Write the current y position
    file << "\n";      // Create a new line
    file.close();      // Close the file
  }
}