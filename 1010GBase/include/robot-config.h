using namespace vex;


extern brain Brain;

// VEXcode devices
// Controller
extern controller Controller1;
extern controller Controller2;
// Sensors
extern inertial IMU;
extern line LinePosition1;
extern line LinePosition2;
extern line LinePosition3L;
extern line LinePosition3T;
extern pot potL;
extern pot potR;
extern distance DistanceSensor;
extern limit selector;
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
//Vision
extern vision VisionSensor;
extern vision::signature SIG_1;
extern vision::signature SIG_2;
extern vision::signature SIG_3;
extern vision::signature SIG_4;
extern vision::signature SIG_5;
extern vision::signature SIG_6;
extern vision::signature SIG_7;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);