#include "hardware.h"

// Initialize Hardware below
// Form: [class] Hardware::[name](parameters);

brain Hardware::v5_brain;

controller Hardware::master(controllerType::primary);
controller Hardware::partner(controllerType::partner);

motor Hardware::lf(PORT20), Hardware::rf(PORT11, true), Hardware::lr(PORT19), Hardware::rr(PORT12, true);

MecanumDrive Hardware::mec_drive(lf, rf, lr, rr);

motor Hardware::intakeLeft(PORT2);
motor Hardware::intakeRight(PORT1, true);

motor_group Hardware::intake(Hardware::intakeLeft, Hardware::intakeRight);

motor Hardware::front_rollers(PORT5, true);
motor Hardware::bottom_roller(PORT6, gearSetting::ratio6_1);
motor Hardware::top_roller(PORT7, gearSetting::ratio6_1);

optical Hardware::optic(PORT13);

// TODO: If anyone finds a better way to use the 3 wire ports, PLEASE fix this :)
limit Hardware::limit_switch(Hardware::v5_brain.ThreeWirePort.C);

// End Hardware Initialization