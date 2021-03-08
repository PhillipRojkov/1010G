#include "EncoderOdometry.h"

void EncoderOdometry::computeLocation() {
  deltaL = (encoderL.position(deg) - prevEncoderL) / 180 * PI * wheelRadius;
  deltaR = (encoderR.position(deg) - prevEncoderR) / 180 * PI * wheelRadius;
  deltaM = (encoderM.position(deg) - prevEncoderM) / 180 * PI * wheelRadius;;

  prevEncoderL = encoderL.position(deg);
  prevEncoderR = encoderR.position(deg);
  prevEncoderM = encoderM.position(deg);

  deltaTheta = (deltaL - deltaR) / (sL + sR);
  theta += deltaTheta;

  double R = 0;
  double RM = 0;
  if (deltaTheta == 0) {
    R = sR;
    RM = sM;
  } else {
    R = deltaR / deltaTheta + sR;
    RM = deltaM / deltaTheta + sM;
  }


  double deltaY = 2 * sin(deltaTheta / 2) * R;
  double deltaX = 2 * sin(deltaTheta / 2) * RM;

  double avgTheta = (theta + prevTheta) / 2;

  encoderX += deltaX * cos(avgTheta) + deltaY * sin(avgTheta);
  encoderY += deltaY * cos(avgTheta) + deltaX * sin(avgTheta);

  prevTheta = theta;
}