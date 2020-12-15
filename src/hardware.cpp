#include "hardware.h"

// Initialize Hardware below
// Form: [class] Hardware::[name](parameters);

// -- BRAIN & CONTROLLER(S) --
brain Hardware::v5_brain;

// -- MOTORS --

// First create the motors normally
motor Hardware::left_front(PORT1);
motor Hardware::left_rear(PORT2);
motor Hardware::right_front(PORT3);
motor Hardware::right_rear(PORT4);

// Then create groups from those motors
motor_group Hardware::left_group(Hardware::left_front, Hardware::left_rear);
motor_group Hardware::right_group(Hardware::right_front, Hardware::right_rear);

// Then create a "TankDrive" object using the motor groups, inertial object, and a config struct.
TankDrive Hardware::drive_system(Hardware::left_group, Hardware::right_group, Hardware::inertia, Config::drive_config);


// -- SENSORS --
inertial Hardware::inertia(PORT16);

// End Hardware Initialization