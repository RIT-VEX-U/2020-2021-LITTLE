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
extern motor lf1, rf1, lf2, rf2, lr1, rr1, lr2, rr2;
extern MecanumDrive mec_drive;

extern motor intakeLeft;
extern motor intakeRight;
extern motor_group intake;

extern motor front_rollers;

extern motor flywheel_right;
extern motor flywheel_left;
extern motor_group flywheel;

// -- SENSORS --
extern inertial inertia;
extern rotation enc_wheel;
extern distance scored;

//End Hardware Declarations
} // namespace Hardware

#endif