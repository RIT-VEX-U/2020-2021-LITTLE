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
rotation Hardware::enc_wheel(PORT11);
// TODO: Find an actual port for this
distance Hardware::ejection(PORT1);

// -- MOTORS --
motor Hardware::lf1(PORT13, gearSetting::ratio36_1, true), Hardware::rf1(PORT20, gearSetting::ratio36_1),
      //Hardware::lf2(PORT5, gearSetting::ratio36_1), Hardware::rf2(PORT6, gearSetting::ratio36_1, true),
      Hardware::lr1(PORT12, gearSetting::ratio36_1, true), Hardware::rr1(PORT19, gearSetting::ratio36_1);
      //Hardware::lr2(PORT3, gearSetting::ratio36_1), Hardware::rr2(PORT4, gearSetting::ratio36_1, true);
// motor_group left_front = motor_group(Hardware::lf1, Hardware::lf2);
// motor_group right_front = motor_group(Hardware::rf1, Hardware::rf2);
// motor_group left_back = motor_group(Hardware::lr1, Hardware::lr2);
// motor_group right_back = motor_group(Hardware::rr1, Hardware::rr2);

// WARNING: NOT TUNED! These are filler values
TankDrive::tankdrive_config_t tank_config = {
  (PID::pid_config_t) {
    // p, i, d, f
    0.5, 0, 0, 0,
    // deadband, on_target_time
    0.5, 100
  },
  (PID::pid_config_t) {
    // p, i, d, f
    0.5, 0, 0, 0,
    // deadband, on_target_time
    0.5, 100
  },
  4.0
};
motor_group left_motors = {Hardware::lf, Hardware::lr};
motor_group right_motors = {Hardware::rf, Hardware::rr};
TankDrive Hardware::tank_drive(left_motors, right_motors, Hardware::inertia, tank_config);

MecanumDrive Hardware::mec_drive(lf1, rf1, lr1, rr1);

motor Hardware::intakeLeft(PORT15);
motor Hardware::intakeRight(PORT9, true);
motor_group Hardware::intake(Hardware::intakeLeft, Hardware::intakeRight);

motor Hardware::front_rollers(PORT10, gearSetting::ratio6_1);
motor Hardware::bottom_roller(PORT18, gearSetting::ratio6_1);
motor Hardware::top_roller(PORT17, gearSetting::ratio6_1);

motor Hardware::flywheel_right(PORT8, gearSetting::ratio6_1, true);
motor Hardware::flywheel_left(PORT6, gearSetting::ratio6_1);
motor_group Hardware::flywheel(flywheel_right, flywheel_left);

motor Hardware::indexer(PORT14);

// -- SENSORS --
inertial Hardware::inertia(PORT16);
optical Hardware::optic(PORT7);
// TODO: If anyone finds a better way to use the 3 wire ports, PLEASE fix this :)
limit Hardware::limit_switch(Hardware::v5_brain.ThreeWirePort.C);
rotation Hardware::enc_wheel(PORT11);
// TODO: Find an actual port for this
distance Hardware::ejection(PORT11);
distance Hardware::scored(PORT1);

// End Hardware Initialization