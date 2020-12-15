/**
 * config.h
 * Created 4/3/2020
 * 
 * A location where all configuration variables are set for the robot, usually in the form
 * of creating Structs, and initializing their values at runtime in config.cpp
 * 
 */
#include "core.h"

namespace Config
{


// Declare configuration structs below
// Form: extern [structName] [name];

extern TankDrive::tankdrive_config_t drive_config;

// End Config Declarations

void initConfig();
} // namespace Config