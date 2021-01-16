#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
// Controller
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
// Sensors
inertial IMU = inertial(PORT16);
limit LinePosition1 = limit(Brain.ThreeWirePort.A);
limit LinePosition2 = limit(Brain.ThreeWirePort.B);
limit LinePosition2R = limit(Brain.ThreeWirePort.D);
limit LinePosition3 = limit(Brain.ThreeWirePort.C);
pot potL = pot(Brain.ThreeWirePort.H);
pot potR = pot(Brain.ThreeWirePort.D);
distance DistanceSensor = distance(PORT13);
//vision VisionSensor = vision(PORT2, 50);
// Drive motors
motor DriveFL = motor(PORT20, ratio18_1, false); // Front left drive
motor DriveFR = motor(PORT8, ratio18_1, true);   // Front right drive - Reversed
motor DriveBL = motor(PORT19, ratio18_1, false); // Back left drive
motor DriveBR = motor(PORT9, ratio18_1, true);   // Back right drive - Reversed
// Intake motors
motor IntakeL = motor(PORT12, ratio18_1, false); // Left intake
motor IntakeR = motor(PORT2, ratio18_1, true);   // Right intake - Reversed
// Indexer motors
motor IndexerTop = motor(PORT11, ratio6_1, true); // Left indexer - Reversed
motor IndexerLow = motor(PORT1, ratio18_1, false); // Right indexer

// VEXcode generated functions

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}