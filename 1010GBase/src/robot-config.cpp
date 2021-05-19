#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
// Controller
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
// Sensors
inertial IMUL = inertial(PORT11);
inertial IMUR = inertial(PORT1);
line LinePosition2 = line(Brain.ThreeWirePort.E);
distance LinePosition3 = distance(PORT8);
line IntakeLineL = line(Brain.ThreeWirePort.F); //Deprecated
line IntakeLineR = line(Brain.ThreeWirePort.F); //Deprecated
distance DistanceSensor = distance(PORT1); //Deprecated
limit selector = limit(Brain.ThreeWirePort.F); //Deprecated
encoder encoderS = encoder(Brain.ThreeWirePort.C);
encoder encoderL = encoder(Brain.ThreeWirePort.A);
encoder encoderR = encoder(Brain.ThreeWirePort.G);
// Drive motors
motor DriveFL = motor(PORT13, ratio18_1, false); // Front left drive
motor DriveFR = motor(PORT18, ratio18_1, true); // Front right drive - Reversed
motor DriveBL = motor(PORT11, ratio18_1, true); // Back left drive - Reversed
motor DriveBR = motor(PORT20, ratio18_1, false); // Back right drive
// Intake motors
motor IntakeL = motor(PORT5, ratio18_1, false); // Left intake
motor IntakeR = motor(PORT6, ratio18_1, true);   // Right intake - Reversed
// Indexer motors
motor IndexerTop = motor(PORT7, ratio18_1, true); // Top indexer - Reversed
motor IndexerLow = motor(PORT4, ratio18_1, false);  // Lower indexer
// Vision
vex::vision::signature SIG_1 =
    vex::vision::signature(1, -3073, -1003, -2038, 853, 11915, 6384, 1, 0);
vex::vision::signature SIG_2 =
    vex::vision::signature(2, 1343, 8793, 5068, -913, 15, -450, 1, 0);
vex::vision::signature SIG_3 =
    vex::vision::signature(3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 =
    vex::vision::signature(4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 =
    vex::vision::signature(5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 =
    vex::vision::signature(6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 =
    vex::vision::signature(7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision VisionSensor = vex::vision(vex::PORT17, 62, SIG_1, SIG_2, SIG_3,
                                       SIG_4, SIG_5, SIG_6, SIG_7);
// VEXcode generated functions

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}