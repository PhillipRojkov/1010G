#include "EncoderOdometry.h"

void EncoderOdometry::computeLocation() {
  //Used for logging time between computes
  deltaT = Brain.timer(sec) - prevTime;
  prevTime = Brain.timer(sec);

  double encoderLValue = encoderL.position(deg);
  double encoderRValue = encoderR.position(deg);
  double encoderSValue = encoderS.position(deg);

  deltaL = (encoderLValue - prevEncoderL) / 180 * PI * wheelRadiusL;
  deltaR = (encoderRValue - prevEncoderR) / 180 * PI * wheelRadiusR;
  deltaS = (encoderSValue - prevEncoderS) / 180 * PI * middleWheelRadius;

  prevEncoderL = encoderLValue;
  prevEncoderR = encoderRValue;
  prevEncoderS = encoderSValue;

  theta = ((IMUL.rotation() + Brain.timer(sec) * gyroDriftL) * constantOfBadGyroL + (IMUR.rotation() + Brain.timer(sec) * gyroDriftR) * constantOfBadGyroR)/2 * (PI / 180);
  deltaTheta = theta - prevTheta;

  //deltaTheta = (deltaL - deltaR) / (offsetL + offsetR);
  //theta += deltaTheta;

  double arcRadius = 0; // radius of the motion of the robot modeled as an arc
  double strafeRadius =
      0; // radius of the strafe motion of the robot modeled as an arc

  if (deltaTheta == 0) {
    deltaX = deltaS;
    deltaY = deltaR;
  } else {
    arcRadius = deltaR / deltaTheta + offsetR;
    strafeRadius = deltaS / deltaTheta + offsetS;

    deltaX = 2 * sin(deltaTheta / 2) * strafeRadius;
    deltaY = 2 * sin(deltaTheta / 2) * arcRadius;
  }

  double avgTheta = (theta + prevTheta) / 2;

  encoderX += deltaX * cos(avgTheta) + deltaY * sin(avgTheta);
  encoderY += deltaY * cos(avgTheta) - deltaX * sin(avgTheta);

  prevTheta = theta;

  writeToSD();
}

void EncoderOdometry::writeToSD() { //Write data to the sd card
 if (Brain.SDcard.isInserted()) { // Write only if the card is inserted
    std::ofstream file(
        "pirate.txt",
        std::ofstream::app); // Create or open a file called pirate.txt
    // Write format:
    // time(sec),x(m),y(m),accelX(m/s^2),deltaT(s)
    file << Brain.timer(sec); // Write the current time to pirate.txt
    file << ",";
    file << encoderX; // Write the current x position
    file << ",";
    file << encoderY; // Write the current y position
    file << ",";
    file << IMUL.rotation();
    file << ",";
    file << IMUR.rotation();
    file << ",";
    file << deltaT; //Write deltaT
    file << "\n";      // Create a new line
    file.close();      // Close the file
  }
}