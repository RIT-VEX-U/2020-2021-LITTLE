#include "hardware.h"

// Initialize Hardware below
// Form: [class] Hardware::[name](parameters);

// -- BRAIN & CONTROLLER(S) --
brain Hardware::v5_brain;
controller Hardware::master(controllerType::primary);
controller Hardware::partner(controllerType::partner);

// -- SENSORS --
inertial Hardware::inertia(PORT16);
distance Hardware::indexer(PORT1);
distance Hardware::lowerIndexer(PORT10); 

// -- MOTORS --
motor Hardware::lf(PORT14, gearSetting::ratio6_1), Hardware::lr(PORT13, gearSetting::ratio6_1, true), Hardware::lr2(PORT2, gearSetting::ratio6_1, true),
      Hardware::rf(PORT18, gearSetting::ratio6_1, true), Hardware::rr(PORT17, gearSetting::ratio6_1), Hardware::rr2(PORT9, gearSetting::ratio6_1);

//MecanumDrive Hardware::mec_drive(lf, rf, lr, rr)

// WARNING: NOT TUNED! These are filler values
TankDrive::tankdrive_config_t tank_config = {
  (PID::pid_config_t) {
    // p, i, d, f –– 3.9 works well just for p and everything, 5.7 ands .16pd?
    6.3, 0, 0.115, 0,
    // deadband, on_target_time
    0.05, .08, //stay on target for 80 msec
  },
  (PID::pid_config_t) {
    // p, i, d, f -- .167
    .165, 0, 0.007, 0,
    // deadband, on_target_time
    1, .1, 
  
  },
  // wheel diam
  3.25,
  // wheel : motor ratio
  1.6667
};
motor_group left_motors = {Hardware::lf, Hardware::lr, Hardware::lr2};
motor_group right_motors = {Hardware::rf, Hardware::rr, Hardware::rr2};
TankDrive Hardware::tank_drive(left_motors, right_motors, Hardware::inertia, tank_config);

motor Hardware::intakeLeft(PORT11);
motor Hardware::intakeRight(PORT19, true);
motor_group Hardware::intake(Hardware::intakeLeft, Hardware::intakeRight);

motor Hardware::bottom_roller1(PORT12, gearSetting::ratio6_1);
motor Hardware::bottom_roller2(PORT5, gearSetting::ratio6_1);
motor Hardware::mid_roller(PORT7, gearSetting::ratio6_1, true);
motor_group Hardware::bottom_rollers(bottom_roller1, Hardware::bottom_roller2, Hardware::mid_roller);
motor Hardware::top_roller(PORT15, gearSetting::ratio6_1);

// End Hardware Initialization




