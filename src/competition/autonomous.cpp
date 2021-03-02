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
void scoreCornerTower() {
  while(!tank_drive.drive_forward(15, 0.5)) {
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
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::rev, 50, velocityUnits::pct);

  // while(!tank_drive.drive_forward(25, 1.0)) {
  //   vexDelay(20);
  // }

  while(lf.rotation(rotationUnits::rev) < (27 / (PI * 4.0 * 3))) {
    tank_drive.drive_tank(1.0, 1.0);
  }
  tank_drive.stop();
  front_rollers.stop();
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

  flywheel.spin(directionType::fwd, 550, velocityUnits::rpm);
  inertia.resetRotation();
  tank_drive.drive_tank(0.25, 0.25);
  wait(800, timeUnits::msec);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  front_rollers.spin(directionType::fwd, 50, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  //intake.spin(directionType::fwd, 100, velocityUnits::pct);

  wait(800, timeUnits::msec);
  double flywheel_rpm = flywheel.velocity(velocityUnits::rpm);
  std::cout<< "\nflywheel rpm: " << flywheel_rpm << std::endl;


  flywheel.stop(brakeType::coast);
  tank_drive.stop();
  top_roller.stop();
  front_rollers.stop();
  bottom_roller.stop();
  intake.stop();

  // how much the robot rotated while driving into the tower
  // 7.0 - 8.0 seems to give the best result, so correct it if it's less or more than this
  double tower_rotate = inertia.rotation();
  wait(100, timeUnits::msec);

  intake.spin(directionType::fwd, 100, velocityUnits::pct);
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::rev, 50, velocityUnits::pct);
  wait(500, timeUnits::msec);
  while(!tank_drive.drive_forward(-12, 0.5)) {
    // prep next ball
    vexDelay(20);
  }

  intake.stop();
  front_rollers.stop();
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
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::rev, 50, velocityUnits::pct);

  while(lf.rotation(rotationUnits::rev) < (27 / (PI * 4.0 * 3))) {
    tank_drive.drive_tank(1.0, 1.0);
  }
  tank_drive.stop();
  front_rollers.stop();
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

  flywheel.spin(directionType::fwd, 550, velocityUnits::rpm);
  inertia.resetRotation();
  tank_drive.drive_tank(0.2, 0.2);
  wait(800, timeUnits::msec);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  //intake.spin(directionType::fwd, 100, velocityUnits::pct);

  wait(400, timeUnits::msec);
  double flywheel_rpm = flywheel.velocity(velocityUnits::rpm);
  std::cout<< "\nflywheel rpm: " << flywheel_rpm << std::endl;

  flywheel.stop(brakeType::coast);
  tank_drive.stop();
  top_roller.stop();
  front_rollers.stop();
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
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::rev, 50, velocityUnits::pct);
  wait(500, timeUnits::msec);
  intake.stop();
  front_rollers.stop();
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

  flywheel.spin(directionType::fwd, 550, velocityUnits::rpm);
  inertia.resetRotation();
  tank_drive.drive_tank(0.2, 0.2);
  wait(800, timeUnits::msec);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  //intake.spin(directionType::fwd, 100, velocityUnits::pct);

  wait(400, timeUnits::msec);
  flywheel_rpm = flywheel.velocity(velocityUnits::rpm);
  std::cout<< "\nflywheel rpm: " << flywheel_rpm << std::endl;

  flywheel.stop(brakeType::coast);
  tank_drive.stop();
  top_roller.stop();
  front_rollers.stop();
  bottom_roller.stop();
  intake.stop();

  intake.spin(directionType::fwd, 100, velocityUnits::pct);
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::rev, 50, velocityUnits::pct);
  wait(500, timeUnits::msec);
  intake.stop();
  front_rollers.stop();
  bottom_roller.stop();

  while(!tank_drive.drive_forward(-12, 0.5)) {
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
  front_rollers.spin(directionType::rev, 500, velocityUnits::rpm);
  intake.spin(directionType::rev, 200, velocityUnits::rpm);
  wait(500, timeUnits::msec);
  front_rollers.stop();
  intake.stop();


  inertia.calibrate();
  while(inertia.isCalibrating()) {
    flywheel.spin(directionType::fwd, 530, velocityUnits::rpm);
  }
  flywheel.stop(brakeType::coast);

  scoreCornerTower();
  scoreSideTower();
  // Don't think I need a loop right now, maybe it will become less linear later...

  // //Autonomous Loop
  // while (true)
  // {
  //   vexDelay(20); // Small delay to allow time-sensitive functions to work properly.
  // }
}