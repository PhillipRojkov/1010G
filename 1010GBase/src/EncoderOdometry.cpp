#include "EncoderOdometry.h"

void EncoderOdometry::computeLocation() {
  double encoderLValue = encoderL.position(deg);
  double encoderRValue = encoderR.position(deg);
  double encoderSValue = encoderS.position(deg);

  deltaL = (encoderLValue - prevEncoderL) / 180 * PI * wheelRadiusL;
  deltaR = (encoderRValue - prevEncoderR) / 180 * PI * wheelRadiusR;
  deltaS = (encoderSValue - prevEncoderS) / 180 * PI * middleWheelRadius;

  prevEncoderL = encoderLValue;
  prevEncoderR = encoderRValue;
  prevEncoderS = encoderSValue;

  theta = (IMU.rotation(deg) * constantOfBadGyro) * (PI / 180);
  deltaTheta = theta - prevTheta;

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
}