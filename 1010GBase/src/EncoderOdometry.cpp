#include "EncoderOdometry.h"

void EncoderOdometry::computeLocation() {
  //Used for logging time between computes
  deltaT = Brain.timer(sec) - prevTime;
  prevTime = Brain.timer(sec);

  double encoderLValue = encoderL.position(deg);
  double encoderRValue = encoderR.position(deg);
  double encoderSValue = encoderS.position(deg);

  //Distance travelled by each encoder wheel
  deltaL = (encoderLValue - prevEncoderL) / 180 * PI * wheelRadiusL;
  deltaR = (encoderRValue - prevEncoderR) / 180 * PI * wheelRadiusR;
  deltaS = (encoderSValue - prevEncoderS) / 180 * PI * middleWheelRadius;

  prevEncoderL = encoderLValue;
  prevEncoderR = encoderRValue;
  prevEncoderS = encoderSValue;

  //Heading in radians
  theta = ((IMUL.rotation() + Brain.timer(sec) * gyroDriftL) * constantOfBadGyroL + (IMUR.rotation() + Brain.timer(sec) * gyroDriftR) * constantOfBadGyroR)/2 * (PI / 180);
  deltaTheta = theta - prevTheta;

  //Unused heading code using encoders
  //deltaTheta = (deltaL - deltaR) / (offsetL + offsetR);
  //theta += deltaTheta;

  double arcRadius = 0; // radius of the motion of the robot modeled as an arc
  double strafeRadius =
      0; // radius of the strafe motion of the robot modeled as an arc

  if (deltaTheta == 0) { //If statement to avoid divide by zero
    deltaX = deltaS;
    deltaY = deltaR;
  } else {
    //Calculate local deltaX and deltaY
    arcRadius = deltaR / deltaTheta + offsetR;
    strafeRadius = deltaS / deltaTheta + offsetS;

    deltaX = 2 * sin(deltaTheta / 2) * strafeRadius;
    deltaY = 2 * sin(deltaTheta / 2) * arcRadius;

    // Create multiplier value which is used to slightly increase deltaX and deltaY 
    // when travelling at 45 degrees to counter wheel slip which results in undershoot
    // along both axis
    // This may not be necessary for all odometry systems
    double dirOfMovement = PI/2;
    if (deltaY != 0) { //Avoid divide by zero problem
      dirOfMovement = atan2(deltaX, deltaY); //Direction of movement relative to orientation
    // 0 would be travelling forward, pi/2 would be travelling right, etc.
    }
    double multiplier = coefficientOfLmao * sin(4 * dirOfMovement - PI/2)/2 + 1 + coefficientOfLmao/2; //Ranges from 1 to 1 + coefficientOfLmao
    deltaX *= multiplier;
    deltaY *= multiplier;
  }

  //Average heading in radians
  double avgTheta = (theta + prevTheta) / 2;
  prevTheta = theta;

  //Set global x and y coordinates
  encoderX += deltaX * cos(avgTheta) + deltaY * sin(avgTheta);
  encoderY += deltaY * cos(avgTheta) - deltaX * sin(avgTheta);

  writeToSD(); //Write debug data to SD card
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