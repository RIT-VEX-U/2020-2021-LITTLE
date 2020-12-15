#ifndef _HARDWARE_
#define _HARDWARE_

#include "vex.h"
#include "core.h"
#include "config.h"

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

// -- MOTORS --

extern motor left_front, left_rear, right_front, right_rear;

extern motor_group left_group, right_group;

extern TankDrive drive_system;

// -- SENSORS --
extern inertial inertia;

//End Hardware Declarations
} // namespace Hardware

#endif