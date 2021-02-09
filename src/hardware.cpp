#include "hardware.h"

// Initialize Hardware below
// Form: [class] Hardware::[name](parameters);

// -- BRAIN & CONTROLLER(S) --
brain Hardware::v5_brain;
controller Hardware::master(controllerType::primary);
controller Hardware::partner(controllerType::partner);

// -- SENSORS --
inertial Hardware::inertia(PORT16);
optical Hardware::optic(PORT7);
// TODO: If anyone finds a better way to use the 3 wire ports, PLEASE fix this :)
limit Hardware::limit_switch(Hardware::v5_brain.ThreeWirePort.C);
distance Hardware::ejection(PORT11);
distance Hardware::scored(PORT1);

// -- MOTORS --
motor Hardware::lf(PORT13, gearSetting::ratio36_1, true), Hardware::rf(PORT20, gearSetting::ratio36_1),
      Hardware::lr(PORT12, gearSetting::ratio36_1, true), Hardware::rr(PORT19, gearSetting::ratio36_1);

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
    0.0055, 0, 0, 0.05,
    // deadband, on_target_time
    0.255, 0
  },
  4.0
};
motor_group left_motors = {Hardware::lf, Hardware::lr};
motor_group right_motors = {Hardware::rf, Hardware::rr};
TankDrive Hardware::tank_drive(left_motors, right_motors, Hardware::inertia, tank_config);

motor Hardware::intakeLeft(PORT15);
motor Hardware::intakeRight(PORT9, true);
motor_group Hardware::intake(Hardware::intakeLeft, Hardware::intakeRight);

motor Hardware::front_rollers(PORT10, gearSetting::ratio6_1);

motor Hardware::flywheel_right(PORT8, gearSetting::ratio6_1, true);
motor Hardware::flywheel_left(PORT6, gearSetting::ratio6_1);
motor_group Hardware::flywheel(flywheel_right, flywheel_left);

motor Hardware::indexer(PORT14);

// End Hardware Initialization