#include "competition/autonomous.h"
#include <iostream>

using namespace Hardware;

void intake_ball() {
  intake.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);

  lf.stop();
  rf.stop();
  lb.stop();
  rb.stop();

  intake.stop();
  bottom_roller.stop();
  top_roller.stop();
}

/**
 * DRIVING TEST AUTO:
 */
void driveTest() {
  //flipout

  while(!tank_drive.drive_forward(10, 13)) {} //move forward at max speed
  //while(!tank_drive.turn_degrees(90, 0.1)) {} 
  //while(!tank_drive.drive_forward(34, 0.5)) {
    //  intake.spin(directionType::fwd, 100, velocityUnits::pct); //intake ball
 // }
  //intake.stop();
  //while(!tank_drive.turn_degrees(90, 0.1)) {}

}


/**
 * 
 
void scoreCornerTower() {
  while(!tank_drive.drive_forward(15, 0.5)){
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  while(!tank_drive.turn_degrees(-90, 0.1)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  // prep for ball intake
  intake.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::rev, 50, velocityUnits::pct);

  // while(!tank_drive.drive_forward(25, 1.0)) {
  //   vexDelay(20);
  // }

  while(lf.rotation(rotationUnits::rev) < (27 / (PI * 4.0 * 3))) {
    tank_drive.drive_tank(1.0, 1.0, volt);
  }
  tank_drive.stop();
  bottom_roller.stop();
  wait(700, timeUnits::msec);
  intake.stop();
  

  inertia.resetRotation();
  wait(100, timeUnits::msec);

  while(!tank_drive.turn_degrees(-23, 0.1)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  while(!tank_drive.drive_forward(12, 0.5)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  inertia.resetRotation();
  tank_drive.drive_tank(0.25, 0.25, volt);
  wait(800, timeUnits::msec);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  //intake.spin(directionType::fwd, 100, velocityUnits::pct);

  tank_drive.stop();
  top_roller.stop();
  bottom_roller.stop();
  intake.stop();

  // how much the robot rotated while driving into the tower
  // 7.0 - 8.0 seems to give the best result, so correct it if it's less or more than this
  double tower_rotate = inertia.rotation();
  wait(100, timeUnits::msec);

  intake.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::rev, 50, velocityUnits::pct);
  wait(500, timeUnits::msec);
  while(!tank_drive.drive_forward(-12, 0.5)) {
    // prep next ball
    vexDelay(20);
  }

  intake.stop();
  bottom_roller.stop();
  tank_drive.stop();

  while(!tank_drive.turn_degrees(121 - tower_rotate, 0.1)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);
}

void scoreSideTower() {
  while(!tank_drive.drive_forward(21, 0.5)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(500, timeUnits::msec);
  lf.resetRotation();

  // prep for ball intake
  intake.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::rev, 50, velocityUnits::pct);

  while(lf.rotation(rotationUnits::rev) < (27 / (PI * 4.0 * 3))) {
    tank_drive.drive_tank(1.0, 1.0);
  }
  tank_drive.stop();
  bottom_roller.stop();
  wait(700, timeUnits::msec);
  intake.stop();

  // while(!tank_drive.drive_forward(48, 0.5)) {
  //   vexDelay(20);
  // }
  // tank_drive.stop();
  // wait(100, timeUnits::msec);

  while(!tank_drive.turn_degrees(-90, 0.1)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  while(!tank_drive.drive_forward(8, 0.5)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  inertia.resetRotation();
  tank_drive.drive_tank(0.2, 0.2, volt);
  wait(800, timeUnits::msec);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  //intake.spin(directionType::fwd, 100, velocityUnits::pct);

  tank_drive.stop();
  top_roller.stop();
  bottom_roller.stop();
  intake.stop();

  double tower_rotate = inertia.rotation();
  wait(100, timeUnits::msec);

  // eject descored blue
  top_roller.spin(directionType::rev, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, 50, velocityUnits::pct);

  wait(300, timeUnits::msec);

  top_roller.stop();
  bottom_roller.stop();

  // prep next red + descore blue
  intake.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::rev, 50, velocityUnits::pct);
  wait(500, timeUnits::msec);
  intake.stop();
  bottom_roller.stop();

  while(!tank_drive.drive_forward(-12, 0.5)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  while(!tank_drive.turn_degrees(90 - tower_rotate, 0.1)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  while(!tank_drive.drive_forward(42, 0.5)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  while(!tank_drive.turn_degrees(-30, 0.1)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);

  inertia.resetRotation();
  tank_drive.drive_tank(0.2, 0.2);
  wait(800, timeUnits::msec);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  //intake.spin(directionType::fwd, 100, velocityUnits::pct);

  wait(400, timeUnits::msec);

  tank_drive.stop();
  top_roller.stop();
  bottom_roller.stop();
  intake.stop();

  intake.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::rev, 50, velocityUnits::pct);
  wait(500, timeUnits::msec);
  intake.stop();
  bottom_roller.stop();

  while(!tank_drive.drive_forward(-12, 0.5)) {
    vexDelay(20);
  }
  tank_drive.stop();
  wait(100, timeUnits::msec);
}
*/




/**
 * Code for the autonomous period is executed below.
 */
void Auto::autonomous()
{
  //Autonomous Init
  //intake.spin(directionType::rev, 200, velocityUnits::rpm);
  //wait(500, timeUnits::msec);
  //intake.stop();


  inertia.calibrate();
  while(inertia.isCalibrating()) {
  }

  //scoreCornerTower();
  //scoreSideTower();
  driveTest();
  // Don't think I need a loop right now, maybe it will become less linear later...

  // //Autonomous Loop
  // while (true)
  // {
  //   vexDelay(20); // Small delay to allow time-sensitive functions to work properly.
  // }
}