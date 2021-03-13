#include "hardware.h"

// Initialize Hardware below
// Form: [class] Hardware::[name](parameters);

// -- BRAIN & CONTROLLER(S) --
brain Hardware::v5_brain;
controller Hardware::master(controllerType::primary);
controller Hardware::partner(controllerType::partner);

// -- SENSORS --
inertial Hardware::inertia(PORT16);
distance Hardware::scored(PORT10);

// -- MOTORS --
motor Hardware::lf(PORT14, gearSetting::ratio6_1,false), Hardware::rf(PORT17, gearSetting::ratio6_1, false),
      Hardware::lb(PORT13, gearSetting::ratio6_1, true), Hardware::rb(PORT18, gearSetting::ratio6_1, true),
      Hardware::l(PORT2, gearSetting::ratio6_1, true), Hardware::r(PORT9, gearSetting::ratio6_1, false); //added motors

//MecanumDrive Hardware::mec_drive(lf, rf, lr, rr);

// WARNING: NOT TUNED! These are filler values
TankDrive::tankdrive_config_t tank_config = {
  (PID::pid_config_t) {
    // p, i, d, f
    10, 0, 0, 0,
    // deadband, on_target_time
    0.05, 0
  },
  (PID::pid_config_t) {
    // p, i, d, f
    0.0055, 0.01, 0, 0,
    // deadband, on_target_time
    1.5, 0
  },
  // wheel diam
  3.25,
  // wheel : motor ratio
  3.0/5.0
};
motor_group left_motors = {Hardware::lf, Hardware::lb, Hardware::l};
motor_group right_motors = {Hardware::rf, Hardware::rb, Hardware::r};
TankDrive Hardware::tank_drive(left_motors, right_motors, Hardware::inertia, tank_config);

motor Hardware::intakeLeft(PORT11);
motor Hardware::intakeRight(PORT19, true);
motor_group Hardware::intake(Hardware::intakeLeft, Hardware::intakeRight);

motor Hardware::bottom_roller(PORT12, gearSetting::ratio6_1);
motor Hardware::mid_roller(PORT1, gearSetting::ratio6_1);
motor Hardware::top_roller(PORT15, gearSetting::ratio6_1);

motor_group Hardware::bottom_rollers(Hardware::bottom_roller, Hardware::mid_roller);


// End Hardware Initialization