#include "competition/opcontrol.h"
// AUTON TESTING: TO BE REMOVED
#include "competition/autonomous.h"
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

// -- COLOR FILTERING --

// NOTE: The values that can be seen when running the optical sensor
// directly through the brain are the values returned by the HUE()
// function, NOT color().
//
// ***ONLY hue() is to be used to retrieve "color" values from the
// optical sensor

// Color filtering has 2 modes:
// Store Mode:
//    The robot can hold 2 balls at once and will wait for user
//    input before attempting to score or eject
// Auto Mode: (typically used on goals)
//    The robot will automatically eject or score a ball, depending
//    on its color
bool store_mode = true;

// STORE MODE:

// Keeps track of the colors of both stored balls (using vals from checkColorRange)
//int ball_hues[2];
// Keeps track of how many balls are currently being stored
//int num_balls = 0;

// BOTH MODES:

// The current "hue" value that the optical sensor is returning
// Will be continuously updated within its own task
int curr_hue;

void changeModes() {
  store_mode = !store_mode;
  const char *mode_msg = store_mode ? "STORE MODE" : "AUTO MODE";
  master.Screen.clearScreen();
  master.Screen.setCursor(1, 1);
  master.Screen.print(mode_msg);
}

/* 
 * Taking the current "hue" value from the optical sensor
 * Will be run in its own task
 * @return task functions need to return an int for some
 *  reason, so just ignore the return statement
 */
int opticSample() {
  while(true) {
    curr_hue = optic.hue();
    wait(10, timeUnits::msec);
  }
  return 1;
}

/*
 * Check if the value given by the optical sensor's
 * hue() function is within the range of red or blue
 * @param hue_value - the value returned by optic.hue()
 * @return 1 = red, 2 = blue, 0 = not in range of either
 */
int checkColorRange(int hue_value) {
  // check if hue is within red range
  if(hue_value <= 40) return 1;
  // check if hue is within blue range
  else if(90 <= hue_value) return 2;
  
  return 0;
}

// Note: The direction of the top roller determines whether the ball moves 
// towards the flywheel or gets ejected

bool front_running = false;

void eject() {
  front_running = true;
  // rollers spin in order to eject
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  top_roller.spin(directionType::rev, 100, velocityUnits::pct);
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);

  task time_out_task = task(&timeOut);
  wait(10, timeUnits::msec);  // wait for time_out to be set to false again
  
  while(ejection.objectDistance(distanceUnits::mm) > 100 && !time_out) {}

  front_rollers.stop();
  bottom_roller.stop();
  top_roller.stop();

  front_running = false;

  //if(num_balls > 0) num_balls--;
}

void score() {
  front_running = true;

  // rollers + flywheel spin in order to score
  flywheel.spin(directionType::fwd, 50, velocityUnits::pct);
  wait(100, timeUnits::msec);
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);

  task time_out_task = task(&timeOut);
  wait(10, timeUnits::msec);  // wait for time_out to be set to false again

  while(scored.objectDistance(distanceUnits::mm) > 100 && !time_out) {}

  front_rollers.stop();
  //flywheel.stop();
  bottom_roller.stop();
  top_roller.stop();
  front_running = false;
  //indexer.spinTo(0, rotationUnits::rev);

  //if(num_balls > 0) num_balls--;
}

void userScore() {
  front_running = true;

  // rollers + flywheel spin in order to score
  flywheel.spin(directionType::fwd, 530, velocityUnits::rpm);
  wait(800, timeUnits::msec);

  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  top_roller.spin(directionType::fwd, 50, velocityUnits::pct);

  while(master.ButtonL1.pressing()) {}

  front_rollers.stop();
  flywheel.stop(brakeType::coast);
  bottom_roller.stop();
  top_roller.stop();
  front_running = false;
}

void ejectOrScore(int color_range) {
  switch(color_range) {
    // red
    case 1:
      score();
      break;
    // blue
    case 2:
      eject();
      break;
  }
}

/**
 * Code for the Driver Control period is executed below.
 */
void OpControl::opcontrol()
{
  // Auto::autonomous();
  // OpControl Init

  // optical sensor is more consistent when the led is on full power
  optic.setLight(ledState::on);

  // Make a task out of the function opticSample
  // Tasks are similar to threads, they are executed in parallel
  //  with the main process
  task optic_sample = task(&opticSample);

  // Variables for color filtering
  // Used for taken the average of the hues read
  int hue_total = 0, reads = 0;

  master.ButtonB.pressed(&eject);
  master.ButtonL1.pressed(&userScore);
  master.ButtonDown.pressed(&changeModes);

  // OpControl Loop
  while (true)
  {
    // -- USER CONTROL --
    tank_drive.drive_tank(master.Axis3.position() / 100.0, master.Axis2.position() / 100.0);
    //tank_drive.drive_arcade(master.Axis3.position() / 100.0, master.Axis1.position() / 100.0);

    if(master.ButtonR1.pressing()) {
      if(!store_mode) {
        flywheel.spin(directionType::fwd, 530, velocityUnits::rpm);
        wait(800, timeUnits::msec);
      }

      front_rollers.spin(directionType::fwd, 50, velocityUnits::pct);
      intake.spin(directionType::fwd, 100, velocityUnits::pct);
       
    }
    else {
      intake.stop();
      if(master.ButtonA.pressing()) {
        front_rollers.spin(directionType::rev, 50, velocityUnits::pct);
      }
      else if(!front_running) front_rollers.stop();

      if(!store_mode) {
        flywheel.stop(brakeType::coast);
      }
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