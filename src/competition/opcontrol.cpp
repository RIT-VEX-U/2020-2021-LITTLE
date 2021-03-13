#include "competition/opcontrol.h"
#include <iostream>
using namespace Hardware;

// flag to be used when bottom roller is running in other functions
bool bottom_running = false;

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


// -- INDEXING --

// Check if an object has reached a certain point in the robot's intake
// system through looping in a task until the distance threshhold has
// been met.
// To use indexing, begin the task by calling:
//    startIndexerChecking()
// If the threshhold has not been met and you wish to break out of the loop, call:
//    endIndexerChecking()

// global ref to a task so it can be stopped anywhere in the file
task check_indexer_task;

/*
 * Run as a task
 * Continuously checks if an object is within 50mm of the distance sensor
 * If there is an object there, stop the bottom rollers
 */
int checkIndexer() {
  bottom_running = true;

  while(indexer.objectDistance(distanceUnits::mm) > 50) {}
  bottom_roller.stop(brakeType::brake);

  bottom_running = false;
  return 0;
}

void startIndexerChecking() {
  check_indexer_task = task(&checkIndexer);
  // give bottom_running flag time to be set
  wait(10, timeUnits::msec);
}

void endIndexerChecking() {
  check_indexer_task.stop();
  bottom_running = false;
  // give bottom_running flag time to be set
  wait(10, timeUnits::msec);
}


// -- SCORING --

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

  // OpControl Init
  master.ButtonL1.pressed(&userScore);

  // OpControl Loop
  while (true)
  {
    tank_drive.drive_tank(master.Axis3.position() / 100.0, master.Axis2.position() / 100.0);

    // intake
    if(master.ButtonR1.pressing()) {
      // intakes are unaffected by indexing
      intake.spin(directionType::fwd, 13, voltageUnits::volt);

      // indexing
      startIndexerChecking();
      while(bottom_running && master.ButtonR1.pressing()) {
        // this is gross but I couldn't quickly think of a better way to keep driving while indexing :(
        tank_drive.drive_tank(master.Axis3.position() / 100.0, master.Axis2.position() / 100.0);
        bottom_roller.spin(directionType::fwd, 13, voltageUnits::volt);
      }
      bottom_roller.stop();
      endIndexerChecking();
    }
    // de-intake
    else if(master.ButtonR2.pressing()) {
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