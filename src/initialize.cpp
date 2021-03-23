#include "initialize.h"
#include "vex.h"
#include "config.h"
#include "hardware.h"

using namespace vex;
using namespace Hardware;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void Init::vexcodeInit(void)
{
  // Initialize the robot configuration variables
  Config::initConfig();

  inertia.calibrate();
  while(inertia.isCalibrating()){wait(5,msec);} //don't move robot during this time

  Hardware::v5_brain.Screen.print("Robot Code Initialized.");
  Hardware::master.Screen.print("Gyro Calibrated.");
  Hardware::partner.Screen.print("Controller Initialized.");
}