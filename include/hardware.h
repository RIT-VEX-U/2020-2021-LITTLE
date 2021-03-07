#ifndef _HARDWARE_
#define _HARDWARE_

#include "vex.h"
#include "core.h"

using namespace vex;

/**
 * hardware.h
 * created 4/3/2020
 * 
 * Contains all the "hardware" objects (motors, sensors, controllers, etc)
 * to be initialized in hardware.cpp
 * 
 */
namespace Hardware
{
// Hardware Declared Below
// Form: extern [class_name] [name];

// -- BRAIN & CONTROLLER(S) --
extern brain v5_brain;
extern controller master;
extern controller partner;

// -- MOTORS --
extern motor lf, rf, lr, rr;
extern TankDrive tank_drive;

extern motor intakeLeft;
extern motor intakeRight;
extern motor_group intake;

extern motor top_roller;
extern motor bottom_roller;

// -- SENSORS --
extern inertial inertia;
// maxed out ~100 mm
extern distance scored;

//End Hardware Declarations
} // namespace Hardware

#endif