#ifndef _TANKDRIVE_
#define _TANKDRIVE_

#define PI 3.141592654

#include "vex.h"
#include "../core/include/utils/pid.h"

using namespace vex;

class TankDrive
{
public:

  struct tankdrive_config_t
  {
    PID::pid_config_t drive_pid;
    PID::pid_config_t turn_pid;

    double wheel_diam;
    double wheel_motor_ratio;
  };

  /**
   * Create the TankDrive object 
   */
  TankDrive(motor_group &left_motors, motor_group &right_motors, inertial &gyro_sensor, tankdrive_config_t &config);

  static int gyroSample();

  /**
   * Stops rotation of all the motors using their "brake mode"
   */
  void stop();

  /**
   * Drive the robot using differential style controls. left_motors controls the left motors,
   * right_motors controls the right motors.
   * 
   * left_motors and right_motors are in "percent": -1.0 -> 1.0
   */
  void drive_tank(double left, double right);
  void drive_volt(double left, double right);

  /**
   * Drive the robot using arcade style controls. forward_back controls the linear motion,
   * left_right controls the turning.
   * 
   * left_motors and right_motors are in "percent": -1.0 -> 1.0
   */
  void drive_arcade(double forward_back, double left_right);

  /**
   * Autonomously drive the robot X inches forward (Negative for backwards), with a maximum speed
   * of percent_speed (-1.0 -> 1.0).
   * 
   * Uses a PID loop for it's control.
   */
  bool drive_forward(double inches, double volt);

  /**
   * Autonomously turn the robot X degrees to the right (negative for left), with a maximum motor speed
   * of percent_speed (-1.0 -> 1.0)
   * 
   * Uses a PID loop for it's control.
   */
  bool turn_degrees(double degrees, double volt);

private:
  tankdrive_config_t &config;

  motor_group &left_motors;
  motor_group &right_motors;

  PID drive_pid;
  PID turn_pid;

  // it should be fine that these are static
  // (shared amongst all instances of the class)
  // bc there should only ever be one tank drive instance
  static inertial &gyro_sensor;
  static double curr_rotation;
  task gyro_sample;

  bool initialize_func = true;
};

extern float prevAngle;

#endif