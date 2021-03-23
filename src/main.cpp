/*
 * main.cpp
 * Created 4/3/2020
 * 
 * Contains the main initialization and loop, where all the robot code is running.
 * Full VexCode Text documentation available at: https://api.vexcode.cloud/v5/
 * 
 */

#include "vex.h"
#include "initialize.h"
#include "competition/autonomous.h"
#include "competition/opcontrol.h"

using namespace Hardware;
using namespace vex;

competition comp;

/**
 * The Main loop controlling the robot program. 
 * 
 * ==== WARNING ====
 * Robot Code should NOT go below!
 * 
 * Put robot code instead in:
 *  competition/opcontrol.cpp
 *  competition/autonomous.cpp
 */
int main()
{
  Init::vexcodeInit();

  comp.autonomous(Auto::autonomous);
  comp.drivercontrol(OpControl::opcontrol);

  while (true)
  {
    vexDelay(10);
  }

  Hardware::v5_brain.Screen.print("Code exited the control loops. Restart code.");
}
