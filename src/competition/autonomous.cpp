#include "competition/autonomous.h"
#include <iostream>

using namespace Hardware;

// -- HELPER FUNCTIONS --

// double curr_rotation;

// int inertiaSample() {
//   while(true) {
//     curr_rotation = inertia.rotation();
//     std::cout<< "rotation: " << curr_rotation << "\n";
//     wait(10, timeUnits::msec);
//   }
//   return 1;
// }

void intake_ball() {
  intake.spin(directionType::fwd, 100, velocityUnits::pct);
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);

  while(!limit_switch.pressing()) {
    lf.spin(directionType::fwd, 50, velocityUnits::pct);
    rf.spin(directionType::fwd, 50, velocityUnits::pct);
    lr.spin(directionType::fwd, 50, velocityUnits::pct);
    rr.spin(directionType::fwd, 50, velocityUnits::pct);
  }

  lf.stop();
  rf.stop();
  lr.stop();
  rr.stop();

  intake.stop();
  front_rollers.stop();
  front_rollers.stop();
  bottom_roller.stop();
  top_roller.stop();
}

/**
 * DRIVING TEST AUTO:
 */
void driveTest() {
  while(!tank_drive.drive_forward(30, 0.5)) {}
  tank_drive.stop();
  wait(500, timeUnits::msec);
  while(!tank_drive.turn_degrees(90, 0.1)) {}
  tank_drive.stop();
  wait(500, timeUnits::msec);
  while(!tank_drive.drive_forward(34, 0.5)) {}
  tank_drive.stop();
  wait(500, timeUnits::msec);
  while(!tank_drive.turn_degrees(90, 0.1)) {}
  tank_drive.stop();
}

/**
 * 
 */
void cycleCornerTower() {
  while(!tank_drive.drive_forward(17, 0.5)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  while(!tank_drive.turn_degrees(-85, 0.1)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  // prep for ball intake
  intake.spin(directionType::fwd, 100, velocityUnits::pct);
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);

  while(!tank_drive.drive_forward(25, 1.0)) {
    vexDelay(20);
  }
  tank_drive.stop();
  intake.stop();
  front_rollers.stop();

  while(!tank_drive.turn_degrees(-45, 0.1)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  while(!tank_drive.drive_forward(22, 0.5)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);
}

/**
 * Code for the autonomous period is executed below.
 */
void Auto::autonomous()
{
  //Autonomous Init
  inertia.calibrate();
  while(inertia.isCalibrating()) {}

  cycleCornerTower();
  
  // Don't think I need a loop right now, maybe it will become less linear later...

  // //Autonomous Loop
  // while (true)
  // {
  //   vexDelay(20); // Small delay to allow time-sensitive functions to work properly.
  // }
}