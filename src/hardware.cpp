#include "hardware.h"

// Initialize Hardware below
// Form: [class] Hardware::[name](parameters);

// -- BRAIN & CONTROLLER(S) --
brain Hardware::v5_brain;
controller Hardware::master(controllerType::primary);
controller Hardware::partner(controllerType::partner);

// -- MOTORS --
motor Hardware::lf(PORT13, gearSetting::ratio36_1, true), Hardware::rf(PORT20, gearSetting::ratio36_1),
      Hardware::lr(PORT12, gearSetting::ratio36_1, true), Hardware::rr(PORT19, gearSetting::ratio36_1);
MecanumDrive Hardware::mec_drive(lf, rf, lr, rr);

motor Hardware::intakeLeft(PORT15);
motor Hardware::intakeRight(PORT9, true);
motor_group Hardware::intake(Hardware::intakeLeft, Hardware::intakeRight);

motor Hardware::front_rollers(PORT10, gearSetting::ratio6_1);
motor Hardware::bottom_roller(PORT18, gearSetting::ratio6_1);
motor Hardware::top_roller(PORT17, gearSetting::ratio6_1);

motor Hardware::flywheel(PORT8, gearSetting::ratio6_1, true);

// -- SENSORS --
inertial Hardware::inertia(PORT16);
optical Hardware::optic(PORT7);
// TODO: If anyone finds a better way to use the 3 wire ports, PLEASE fix this :)
limit Hardware::limit_switch(Hardware::v5_brain.ThreeWirePort.C);
rotation Hardware::enc_wheel(PORT11);
// TODO: Find an actual port for this
distance Hardware::ejection(PORT1);

// End Hardware Initialization