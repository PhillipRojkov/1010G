#include "EncoderOdometry.h"

void EncoderOdometry::computeLocation() {
  deltaL = (encoderL.position(deg) - prevEncoderL) / 180 * PI * wheelRadius;
  deltaR = (encoderR.position(deg) - prevEncoderR) / 180 * PI * wheelRadius;
  deltaS = (encoderS.position(deg) - prevEncoderS) / 180 * PI * wheelRadius;

  prevEncoderL = encoderL.position(deg);
  prevEncoderR = encoderR.position(deg);
  prevEncoderS = encoderS.position(deg);

  deltaTheta = (deltaL - deltaR) / (offsetL + offsetR);
  theta += deltaTheta;

  double arcRadius = 0;
  double strafeRadius = 0;
  if (fabs(deltaTheta) < 0.001) {
    arcRadius = offsetR;
    strafeRadius = offsetS;

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
  encoderY += deltaY * cos(avgTheta) + deltaX * sin(avgTheta);

  prevTheta = theta;
}