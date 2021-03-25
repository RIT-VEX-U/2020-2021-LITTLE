#include "../core/include/subsystems/tank_drive.h"
#include "hardware.h"
#include <iostream>

inertial &TankDrive::gyro_sensor = Hardware::inertia;
double TankDrive::curr_rotation = 0;

using namespace Hardware;

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

void TankDrive::drive_volt(double left, double right)
{
  left_motors.spin(directionType::fwd, left, volt);
  right_motors.spin(directionType::fwd, right, volt);
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
float prevAngle; //allows robot to follow the line defined by the angle defined from the previous turn or starting angle

bool TankDrive::drive_forward(double inches, double percent_speed)
{
  // On the first run of the funciton, reset the motor position and PID
  if (initialize_func)
  {
    left_motors.resetPosition();
    right_motors.resetPosition();
    drive_pid.reset();

    drive_pid.set_limits(-fabs(percent_speed), fabs(percent_speed));

    // setting target to # revolutions the motor has to do
    drive_pid.set_target(inches*2.78 / (PI * config.wheel_diam * config.wheel_motor_ratio));

    initialize_func = false;
  }

  // Update PID loop and drive the robot based on it's output
  drive_pid.update((left_motors.position(rotationUnits::rev) + right_motors.position(rev))/2);
  //std::cout << "avg [rev]: " << (left_motors.position(rev) + right_motors.position(rev))/2<< std::endl;
  //std::cout << "error: " << drive_pid.get_error() << std::endl;

  double pid_out = drive_pid.get();
  //std::cout << "output: " << pid_out << "\n" << std::flush;

  double turnPower;
  if(fabs(inertia.rotation() - prevAngle) > .5 && fabs(drive_pid.get_error()) > 1)
   turnPower = (inertia.rotation() - prevAngle) * .01;
  else
    turnPower = 0;

    std::cout << "tp: " << turnPower << "\n" << std::flush;


  drive_tank(pid_out - turnPower, pid_out + turnPower);

  // If the robot is at it's target, return true
  if (drive_pid.is_on_target())
  {
    drive_tank(0, 0);
    initialize_func = true;
    return true;
  }

  std::cout << "\n";

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
    prevAngle = degrees; //update global variable between drive function and turn function
    turn_pid.reset();

    turn_pid.set_limits(-fabs(percent_speed), fabs(percent_speed));
    turn_pid.set_target(degrees);

    task gyro_sample = task(&TankDrive::gyroSample);

    initialize_func = false;
  }

  // Update PID loop and drive the robot based on it's output
  turn_pid.update(gyro_sensor.rotation(deg));
  double pid_out = turn_pid.get();
  //std::cout << "pid out: " << pid_out << "\n" << std::flush;
  drive_tank(pid_out, -pid_out);

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