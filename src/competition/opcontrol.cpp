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

/*
 * Initially used for color sorting, will probably be removed soon
 */
void score() {
  wait(100, timeUnits::msec);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  mid_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);

  task time_out_task = task(&timeOut);
  wait(10, timeUnits::msec);  // wait for time_out to be set to false again

  while(scored.objectDistance(distanceUnits::mm) > 100 && !time_out) {}

  bottom_roller.stop();
  mid_roller.stop();
  top_roller.stop();
}

/*
 * Function called when scoring in user control
 * (also initially created for color sorting but this seems more useful)
 */
void userScore() {
  bottom_running = true;
  
  bottom_roller.spin(directionType::fwd, 13, voltageUnits::volt);
  mid_roller.spin(directionType::fwd, 13, voltageUnits::volt);
  top_roller.spin(directionType::fwd, 13, voltageUnits::volt);

  while(master.ButtonL1.pressing()) {}

  bottom_roller.stop();
  mid_roller.stop();
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
  
  //master.ButtonL1.pressed(&userScore);

  // OpControl Loop
  while (true)
  {
    int rPower  = master.Axis2.value();
		int lPower  = master.Axis3.value();

    //throttle user input –– dampened input value
    double outputL = pow(lPower,7)/pow(127,6);
    double outputR = pow(rPower,7)/pow(127,6);

    tank_drive.drive_tank(outputL, outputR, volt); //percentage of max output
    //tank_drive.drive_arcade(master.Axis3.position() / 100.0, master.Axis1.position() / 100.0);


    if(master.ButtonR1.pressing()) { //intake
      intake.spin(directionType::fwd, 13, voltageUnits::volt);
      bottom_roller.spin(directionType::fwd, 13, voltageUnits::volt);
      mid_roller.spin(directionType::fwd, 13, voltageUnits::volt);
      top_roller.spin(directionType::fwd, -13, voltageUnits::volt);

    }else if(master.ButtonR2.pressing()) { //outtake
      intake.spin(directionType::fwd, -13, voltageUnits::volt);
      bottom_roller.spin(directionType::fwd, -13, voltageUnits::volt);
      mid_roller.spin(directionType::fwd, -13, voltageUnits::volt);
      top_roller.spin(directionType::fwd, -13, voltageUnits::volt);

    }else if(master.ButtonL1.pressing()) {
      intake.spin(directionType::fwd, 13, voltageUnits::volt);
      bottom_roller.spin(directionType::fwd, -13, voltageUnits::volt);
      mid_roller.spin(directionType::fwd, 13, voltageUnits::volt);
      top_roller.spin(directionType::fwd, 13, voltageUnits::volt);

    }else {
      intake.stop();

      if(!bottom_running)
        bottom_roller.stop();
    }

    vexDelay(10); // Small delay to allow time-sensitive functions to work properly.
  }
}