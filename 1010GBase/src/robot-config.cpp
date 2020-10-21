#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
//Controller
controller Controller1 = controller(primary);
//Drive motors
motor DriveFL = motor(PORT1, ratio18_1, true); //Front left drive - Reversed
motor DriveFR = motor(PORT2, ratio18_1, false); //Front right drive
motor DriveBL = motor(PORT3, ratio18_1, true); //Back left drive - Reversed
motor DriveBR = motor(PORT4, ratio18_1, false); //Back right drive
//Intake motors
motor IntakeL = motor(PORT5, ratio18_1, false); //Left intake
motor IntakeR = motor(PORT6, ratio18_1, true); //Right intake - Reversed
//Indexer motors
motor IndexerL = motor(PORT7, ratio18_1, false); //Left indexer
motor IndexerR = motor(PORT8, ratio18_1, true); //Right indexer - Reversed

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}