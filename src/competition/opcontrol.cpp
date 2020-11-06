#include "competition/opcontrol.h"
#include <iostream>

using namespace Hardware;

// -- TESTING FUNCTIONS: TO BE REMOVED --

/*
 * Used for printing encoder values to the
 * controller
 */
void printToController(float lf, float rr) {
  master.Screen.clearScreen();
  master.Screen.setCursor(1, 1);
  master.Screen.print(lf);
  master.Screen.setCursor(2, 1);
  master.Screen.print(rr);
}

// -- OPTICAL SENSOR --

// NOTE: The values that can be seen when running the optical sensor
// directly through the brain are the values returned by the HUE()
// function, NOT color().
//
// ***ONLY hue() is to be used to retrieve "color" values from the
// optical sensor

// The current "hue" value that the optical sensor is returning
// Will be continuously updated within its own task
int curr_hue;

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

void eject() {
  // rollers spin in order to eject
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  top_roller.spin(directionType::rev, 100, velocityUnits::pct);

  // give the ball time to be ejected
  // TODO: WILL BE REPLACED BY DISTANCE SENSOR
  wait(500, timeUnits::msec);
  bottom_roller.stop();
  top_roller.stop();
}

void score() {
  // rollers + flywheel spin in order to score
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  flywheel.spin(directionType::fwd, 50, velocityUnits::pct);

  // give the ball time to be scored
  // TODO: WILL BE REPLACED BY DISTANCE SENSOR
  wait(500, timeUnits::msec);
  flywheel.stop();
  bottom_roller.stop();
  top_roller.stop();
}

// -- OPTICAL SENSOR TESTING: TO BE REMOVED --

// stores up to 20 hue values taken from the optic sensor
int hues[40];
// index that can currently be stored in
int color_index = 0;

/*
 * Print the next value in the hues array to the controller
 */
void printNextColor() {
  // check that color_index is not at the end of the array before incrementing
  if(color_index < 39) color_index++;
  master.Screen.clearScreen();
  master.Screen.setCursor(1, 1);
  master.Screen.print(color_index);
  master.Screen.setCursor(2, 1);

  int curr_hue = hues[color_index];
  int hue_range = checkColorRange(curr_hue);

  switch(hue_range) {
    case 1: 
      master.Screen.print("red");
      break;
    case 2:
      master.Screen.print("blue");
      break;
    default:
      master.Screen.print("no color detected");
      break;
  }

  master.Screen.print(curr_hue);
}

/*
 * Print the previous value in the hues array to the controller
 */
void printPrevColor() {
  // check that color_index is not at the beginning of the array 
  // before incrementing
  if(color_index > 0) color_index--;
  master.Screen.clearScreen();
  master.Screen.setCursor(1, 1);
  master.Screen.print(color_index);
  master.Screen.setCursor(2, 1);

  int curr_hue = hues[color_index];
  int hue_range = checkColorRange(curr_hue);

  switch(hue_range) {
    case 1: 
      master.Screen.print("red");
      break;
    case 2:
      master.Screen.print("blue");
      break;
    default:
      master.Screen.print("no color detected");
      break;
  }

  master.Screen.setCursor(3, 1);
  master.Screen.print(hues[color_index]);
}

/**
 * Code for the Driver Control period is executed below.
 */
void OpControl::opcontrol()
{
  // OpControl Init

  // optical sensor is more consistent when the led is on full power
  optic.setLight(ledState::on);

  // Make a task out of the function opticSample
  // Tasks are similar to threads, they are executed in parallel
  //  with the main process
  task optic_sample = task(&opticSample);

  //
  int hue_total = 0, reads = 0;

  // OpControl Loop
  while (true)
  {
    mec_drive.drive(master.Axis3.position(), master.Axis4.position(), master.Axis1.position());

    // -- TEMP OP CONTROL --
    // NOTE: only front_rollers and intake are user-controlled, the other rollers
    // are controlled by the limit switch and optical sensor inputs
    if(master.ButtonR2.pressing()) {
      front_rollers.spin(directionType::fwd, 50, velocityUnits::pct);
      intake.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    else {
      front_rollers.stop();
      intake.stop();
    }

    // -- COLOR FILTERING --

    if(limit_switch.pressing()) {
      hue_total += curr_hue;
      reads++;
    }
    else if(reads > 0) {
      int avg = hue_total / reads;
      int color_range = checkColorRange(avg);

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

      // reset color filtering variables
      hue_total = 0, reads = 0;
    }

    vexDelay(10); // Small delay to allow time-sensitive functions to work properly.
  }

  // -- OPTICAL SENSOR TESTING: TO BE REMOVED --
  // optic_sample.stop();

  // master.ButtonLeft.pressed(&printPrevColor);
  // master.ButtonRight.pressed(&printNextColor);
}