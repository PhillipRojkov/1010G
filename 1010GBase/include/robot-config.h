using namespace vex;

extern brain Brain;

// VEXcode devices
// Controller
extern controller Controller1;
extern controller Controller2;
// Sensors
extern inertial IMU;
extern limit LinePosition1;
extern limit LinePosition2;
extern limit LinePosition2R;
extern limit LinePosition3;
extern pot potL;
extern pot potR;
extern distance DistanceSensor;
// Drive
extern motor DriveFL;
extern motor DriveFR;
extern motor DriveBL;
extern motor DriveBR;
// Intakes
extern motor IntakeL;
extern motor IntakeR;
// Indexers
extern motor IndexerTop;
extern motor IndexerLow;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);