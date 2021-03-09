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
  // Match autos
  void rightHome();
  void leftHome();
  void rightTwoAndMiddle();
  void rightTwoAndSide();

  bool runIndexer = false;;
};