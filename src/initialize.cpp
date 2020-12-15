#include "initialize.h"
#include "vex.h"
#include "config.h"
#include "hardware.h"

using namespace vex;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void Init::vexcodeInit(void)
{
  // Initialize the robot configuration variables
  Config::initConfig();

  Hardware::v5_brain.Screen.print("Robot Code Initialized.");
}