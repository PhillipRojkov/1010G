#pragma once
#include "AutoFunctions.h"
#include "vex.h"

class AutoMasters { // Holds complete autonomous runs
private:
  AutoFunctions autoFunctions; //instance of AutoFunctions
public:
  void runOdometry();
  // Flipout auto
  void runFlipout();
  // Skills autos
  void skills();
  // LRT autos
  void LRTAuto();
  // Match autos
  void rightHome();
  void leftHome();
  void leftTwoAndMiddle();
  void rightTwoAndSide();
};