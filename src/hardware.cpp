#include "hardware.h"

// Initialize Hardware below
// Form: [class] Hardware::[name](parameters);

// -- BRAIN & CONTROLLER(S) --
brain Hardware::v5_brain;
controller Hardware::master(controllerType::primary);
controller Hardware::partner(controllerType::partner);

// -- SENSORS --
inertial Hardware::inertia(PORT16);
distance Hardware::indexer(PORT10);

// -- MOTORS --
motor Hardware::lf(PORT14, gearSetting::ratio18_1), Hardware::rf(PORT18, gearSetting::ratio18_1, true),
      Hardware::lr(PORT13, gearSetting::ratio18_1, true), Hardware::lr2(PORT2, gearSetting::ratio18_1, true),
      Hardware::rr(PORT17, gearSetting::ratio18_1), Hardware::rr2(PORT9, gearSetting::ratio18_1);

//MecanumDrive Hardware::mec_drive(lf, rf, lr, rr);

// WARNING: NOT TUNED! These are filler values
TankDrive::tankdrive_config_t tank_config = {
  (PID::pid_config_t) {
    // p, i, d, f
    1.0, 0, 0, 0,
    // deadband, on_target_time
    0.05, 0
  },
  (PID::pid_config_t) {
    // p, i, d, f
    0.0005, 0, 0, 0,
    // deadband, on_target_time
    1.5, 0
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
motor Hardware::bottom_roller2(PORT1, gearSetting::ratio6_1);
motor_group Hardware::bottom_roller(bottom_roller1, bottom_roller2);
motor Hardware::top_roller(PORT15, gearSetting::ratio6_1);

// End Hardware Initialization