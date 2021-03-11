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

// flag to be used when bottom roller is running in other functions
bool bottom_running = false;
task check_indexer_task;

int checkIndexer() {
  bottom_running = true;

  while(indexer.objectDistance(distanceUnits::mm) > 50) {}
  bottom_roller.stop(brakeType::brake);

  bottom_running = false;
  return 0;
}

void startIndexerChecking() {
  check_indexer_task = task(&checkIndexer);
  wait(10, timeUnits::msec);
}

void endIndexerChecking() {
  check_indexer_task.stop();
  wait(20, timeUnits::msec);
}

/*
 * Initially used for color sorting, will probably be removed soon
 */
void score() {
  wait(100, timeUnits::msec);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);

  task time_out_task = task(&timeOut);
  wait(10, timeUnits::msec);  // wait for time_out to be set to false again

  bottom_roller.stop();
  top_roller.stop();
}

/*
 * Function called when scoring in user control
 * (also initially created for color sorting but this seems more useful)
 */
void userScore() {
  bottom_running = true;
  bottom_roller.spin(directionType::fwd, 13, voltageUnits::volt);
  top_roller.spin(directionType::fwd, 13, voltageUnits::volt);

  while(master.ButtonL1.pressing()) {}

  bottom_roller.stop();
  top_roller.stop();
  bottom_running = false;
}

/**
 * Code for the Driver Control period is executed below.
 */
void OpControl::opcontrol()
{
  // Auto::autonomous();
  // OpControl Init
  
  master.ButtonL1.pressed(&userScore);

  // OpControl Loop
  while (true)
  {
    tank_drive.drive_tank(master.Axis3.position() / 100.0, master.Axis2.position() / 100.0);
    //tank_drive.drive_arcade(master.Axis3.position() / 100.0, master.Axis1.position() / 100.0);

    if(master.ButtonR1.pressing()) {  // intake
      startIndexerChecking();
      while(bottom_running && master.ButtonR1.pressing()) {
        intake.spin(directionType::fwd, 13, voltageUnits::volt);
        bottom_roller.spin(directionType::fwd, 5, voltageUnits::volt);
      }
      endIndexerChecking();
    }
    else if(master.ButtonR2.pressing()) { // de-intake
      intake.spin(directionType::rev, 13, voltageUnits::volt);
      bottom_roller.spin(directionType::rev, 10, voltageUnits::volt);
    }
    else {
      intake.stop();
      if(!bottom_running)
        bottom_roller.stop();
    }

    vexDelay(10); // Small delay to allow time-sensitive functions to work properly.
  }
}