#include "competition/opcontrol.h"
#include <iostream>
using namespace Hardware;

// -- TIME OUT --

// Using a timer from the vex api, a task will determine whether or not a
// specified amount of miliseconds has passed since a function began.
// To use the time out, begin the task at the start of a function with:
//    task time_out_task = task(&timeOut).
// The global variable:
//    time_out 
// can be used to tell whether or not the function has used up its time.

timer t = timer();

// Represents whether or not a function has used up its time
bool time_out = false;

/*
 * Should be used as a task at the start of a function or loop
 * @return task functions need to return an int for some
 *  reason, so just ignore the return statement
 */
int timeOut() {
  // reset
  time_out = false;
  t.clear();

  // Note: may change to a thread so I can send this an arg for how much time
  // the function / loop can run. For now, the default is 500 ms
  while(t.time(timeUnits::msec) < 500) {}

  time_out = true;
  return 1;
}

/**
 * Code for the Driver Control period is executed below.
 */
void OpControl::opcontrol()
{
  // OpControl Init

  // OpControl Loop
  while (true)
  {
    mec_drive.drive(master.Axis3.position(), master.Axis4.position(), master.Axis1.position());

    // -- USER CONTROL --
    if(master.ButtonL2.pressing()) {
      intake.spin(directionType::fwd, 100, velocityUnits::pct);
      front_rollers.spin(directionType::fwd, 50, velocityUnits::pct);
    }
    else {
      intake.stop();
      front_rollers.stop();
    }

    if(master.ButtonR2.pressing()) {
      flywheel.spin(directionType::fwd, 13, voltageUnits::volt);
    }
    else {
      flywheel.stop(brakeType::coast);
    }

    vexDelay(10); // Small delay to allow time-sensitive functions to work properly.
  }
}