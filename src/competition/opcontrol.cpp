#include "competition/opcontrol.h"

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

// -- OPTICAL SENSOR TESTING: TO BE REMOVED --

// NOTE: The values that can be seen when running the optical sensor
// directly through the brain are the values returned by the HUE()
// function, NOT color().
//
// ***ONLY hue() is to be used to retrieve "color" values from the
// optical sensor

// stores up to 20 hue values taken from the optic sensor
int hues[20];
// index that can currently be stored in
int color_index = 0;

/*
 * Check if the value given by the optical sensor's
 * hue() function is within the range of red or blue
 * @param hue_value - the value returned by optic.hue()
 * @return 1 = red, 2 = blue, 0 = not in range of either
 */
int checkColorRange(int hue_value) {
  // check if hue is within red range
  if(10 <= hue_value && hue_value <= 30) return 1;
  // check if hue is within blue range
  else if(190 <= hue_value && hue_value <= 230) return 2;
  
  return 0;
}

/*
 * Print the next value in the hues array to the controller
 */
void printNextColor() {
  // check that color_index is not at the end of the array before incrementing
  if(color_index < 19) color_index++;
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

  master.Screen.print(hues[color_index]);
} 

/*
 * Determines whether the ball is of a color we want to score
 * or eject and runs the roller motors accordingly
 * Currently, the function EJECTS BLUES and SCORES REDS
 */
void ejectOrScore() {
  // give the ball time to reach the optical sensor before reading
  wait(200, timeUnits::msec);
  int curr_hue = optic.hue();

  // The direction of the top roller, determines whether the ball moves 
  // towards the flywheel or gets ejected (default: toward flywheel, fwd)
  directionType up_or_out = directionType::fwd;

  int hue_range = checkColorRange(curr_hue);

  switch(hue_range){
    // blue: eject
    case 2:
      up_or_out = directionType::rev;
      break;
    // red: flywheel
    case 1:
    default:
      up_or_out = directionType::fwd;
      break;
  }

  // rollers spin in order to either eject or score
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  top_roller.spin(up_or_out, 100, velocityUnits::pct);

  // give the ball time to be ejected or scored
  // TODO: WILL BE REPLACED BY LIMIT SWITCH(ES)
  wait(1000, timeUnits::msec);
  bottom_roller.stop();
  top_roller.stop();
}

/**
 * Code for the Driver Control period is executed below.
 */
void OpControl::opcontrol()
{
  // OpControl Init

  // optical sensor is more consistent when the led is on full power
  optic.setLight(ledState::on);

  // when the limit switch is activated, ejectOrScore() is called ONCE
  // NOTE: This is different from pressing(), which will continue to
  // call the function until the switch/button is released
  limit_switch.pressed(&ejectOrScore);

  // OpControl Loop
  while (true)
  {
    mec_drive.drive(master.Axis3.position(), master.Axis4.position(), master.Axis1.position());

    // -- TEMP OP CONTROL --
    // NOTE: only front_rollers and intake are user-controlled, the other rollers
    // are controlled by the limit switch and optical sensor inputs
    if(master.ButtonR2.pressing()) {
      front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
      intake.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    else {
      front_rollers.stop();
      intake.stop();
    }

    vexDelay(20); // Small delay to allow time-sensitive functions to work properly.
  }
}