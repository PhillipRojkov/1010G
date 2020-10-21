using namespace vex;

extern brain Brain;

// VEXcode devices
//Controller
extern controller Controller1;
//Drive
extern motor DriveFL;
extern motor DriveFR;
extern motor DriveBL;
extern motor DriveBR;
//Intakes
extern motor IntakeL;
extern motor IntakeR;
//Indexers
extern motor IndexerL;
extern motor IndexerR;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );