#include "../core/include/subsystems/tank_drive.h"
#include "hardware.h"
#include <iostream>

inertial &TankDrive::gyro_sensor = Hardware::inertia;
double TankDrive::curr_rotation = 0;

TankDrive::TankDrive(motor_group &left_motors, motor_group &right_motors, inertial &gyro_sensor, TankDrive::tankdrive_config_t &config)
    : config(config), left_motors(left_motors), right_motors(right_motors), drive_pid(config.drive_pid), turn_pid(config.turn_pid)//, gyro_sensor(gyro_sensor)
{
  //TankDrive::gyro_sensor = gyro_sensor;
}

int TankDrive::gyroSample() {
  while(true) {
    curr_rotation = gyro_sensor.rotation();
    //std::cout<< "rotation: " << curr_rotation << "\n";
    wait(10, timeUnits::msec);
  }
  return 1;
}

/**
 * Stops rotation of all the motors using their "brake mode"
 */
void TankDrive::stop()
{
  left_motors.stop(brakeType::brake);
  right_motors.stop(brakeType::brake);
}

/**
 * Drive the robot using differential style controls. left_motors controls the left motors,
 * right_motors controls the right motors.
 * 
 * left_motors and right_motors are in "percent": -1.0 -> 1.0
 */
void TankDrive::drive_tank(double left, double right)
{
  left_motors.spin(directionType::fwd, left * 100, velocityUnits::pct);
  right_motors.spin(directionType::fwd, right * 100, velocityUnits::pct);
}

/**
 * Drive the robot using arcade style controls. forward_back controls the linear motion,
 * left_right controls the turning.
 * 
 * left_motors and right_motors are in "percent": -1.0 -> 1.0
 */
void TankDrive::drive_arcade(double forward_back, double left_right)
{
  double left = forward_back + left_right;
  double right = forward_back - left_right;

  left_motors.spin(directionType::fwd, left * 100, velocityUnits::pct);
  right_motors.spin(directionType::fwd, right * 100, velocityUnits::pct);
}

/**
 * Autonomously drive the robot X inches forward (Negative for backwards), with a maximum speed
 * of percent_speed (-1.0 -> 1.0).
 * 
 * Uses a PID loop for it's control.
 */
bool TankDrive::drive_forward(double inches, double percent_speed)
{
  // On the first run of the funciton, reset the motor position and PID
  if (initialize_func)
  {
    left_motors.resetPosition();
    drive_pid.reset();

    drive_pid.set_limits(-fabs(percent_speed), fabs(percent_speed));
    // setting target to # revolutions the motor has to do
    drive_pid.set_target(inches / (PI * config.wheel_diam * config.wheel_motor_ratio));

    initialize_func = false;
  }

  // Update PID loop and drive the robot based on it's output
  drive_pid.update(left_motors.position(rotationUnits::rev));

  double pid_out = drive_pid.get();
  drive_tank(pid_out, pid_out);

  // If the robot is at it's target, return true
  if (drive_pid.is_on_target())
  {
    drive_tank(0, 0);
    initialize_func = true;
    return true;
  }

  return false;
}

/**
 * Autonomously turn the robot X degrees to the right (negative for left), with a maximum motor speed
 * of percent_speed (-1.0 -> 1.0)
 * 
 * Uses a PID loop for it's control.
 */
bool TankDrive::turn_degrees(double degrees, double percent_speed)
{
  // On the first run of the funciton, reset the gyro position and PID
  if (initialize_func)
  {
    gyro_sensor.resetRotation();
    turn_pid.reset();

    turn_pid.set_limits(-fabs(percent_speed), fabs(percent_speed));
    turn_pid.set_target(degrees);

    task gyro_sample = task(&TankDrive::gyroSample);

    initialize_func = false;
  }

  // Update PID loop and drive the robot based on it's output
  //double curr_rotation = gyro_sensor.rotation(rotationUnits::deg);
  turn_pid.update(curr_rotation);
  double pid_out = turn_pid.get();
  drive_tank(pid_out, -1 * pid_out);

  // If the robot is at it's target, return true
  if (turn_pid.is_on_target())
  {
    drive_tank(0, 0);
    gyro_sample.stop();
    initialize_func = true;
    return true;
  }

  return false;
}