#pragma once
#include "AutoFunctions.h"
#include "vex.h"

class AutoMasters { // Holds complete autonomous runs
private:
  AutoFunctions autoFunctions; //instance of AutoFunctions
public:
  // Flipout auto
  void runFlipout();
  // Skills autos
  void newSkillsNew();
  // Match autos
  void rightHome();
  void leftHome();
  void rightTwoAndMiddle();
  void rightTwoAndSide();

  void braker();
};