#include "hardware.h"

// Initialize Hardware below
// Form: [class] Hardware::[name](parameters);

// -- BRAIN & CONTROLLER(S) --
brain Hardware::v5_brain;
controller Hardware::master(controllerType::primary);
controller Hardware::partner(controllerType::partner);

// -- MOTORS --
motor Hardware::lf1(PORT13, gearSetting::ratio36_1, true), Hardware::rf1(PORT20, gearSetting::ratio36_1),
      //Hardware::lf2(PORT5, gearSetting::ratio36_1), Hardware::rf2(PORT6, gearSetting::ratio36_1, true),
      Hardware::lr1(PORT12, gearSetting::ratio36_1, true), Hardware::rr1(PORT19, gearSetting::ratio36_1);
      //Hardware::lr2(PORT3, gearSetting::ratio36_1), Hardware::rr2(PORT4, gearSetting::ratio36_1, true);
// motor_group left_front = motor_group(Hardware::lf1, Hardware::lf2);
// motor_group right_front = motor_group(Hardware::rf1, Hardware::rf2);
// motor_group left_back = motor_group(Hardware::lr1, Hardware::lr2);
// motor_group right_back = motor_group(Hardware::rr1, Hardware::rr2);

MecanumDrive Hardware::mec_drive(lf1, rf1, lr1, rr1);

motor Hardware::intakeLeft(PORT15);
motor Hardware::intakeRight(PORT9, true);
motor_group Hardware::intake(Hardware::intakeLeft, Hardware::intakeRight);

motor Hardware::front_rollers(PORT10, gearSetting::ratio6_1);

motor Hardware::flywheel_right(PORT8, gearSetting::ratio6_1, true);
motor Hardware::flywheel_left(PORT6, gearSetting::ratio6_1);
motor_group Hardware::flywheel(flywheel_right, flywheel_left);


// -- SENSORS --
inertial Hardware::inertia(PORT16);
rotation Hardware::enc_wheel(PORT11);
// TODO: Find an actual port for this
distance Hardware::scored(PORT1);

// End Hardware Initialization