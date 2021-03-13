#include "../core/include/subsystems/tank_drive.h"
#include "hardware.h"
#include <iostream>

using namespace Hardware;

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
 * rpm and dps are acceptable units
 */
void TankDrive::drive_tank(double left, double right, velocityUnits r)
{
  left_motors.spin(directionType::fwd, left, r);
  right_motors.spin(directionType::fwd, right, r);
}

/**
*
* Overloaded function that accepts direct voltage control
* 
*/
void TankDrive::drive_tank(double left, double right, voltageUnits v)
{
  left_motors.spin(directionType::fwd, left, v);
  right_motors.spin(directionType::fwd, right, v);
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
 * Autonomously drive the robot X inches forward (Negative for backwards), with a maximum voltage
 * of (-13 -> 13).
 * 
 * Uses a PID loop for it's control.
 */
float vMax, accel, vCap, prevAngle ; //slew
int sign;
  

bool TankDrive::drive_forward(double inches, double maxVoltage)
{

  // On the first run of the funciton, reset the motor position and PID
  if (initialize_func)
  {
    vMax = maxVoltage, accel = .5, vCap = 0; //slew
    prevAngle = gyro_sensor.heading(deg);
    left_motors.resetPosition();
    right_motors.resetPosition();
    drive_pid.reset();

    drive_pid.set_limits(-maxVoltage, maxVoltage);
    // setting target to # revolutions the motor has to do
    drive_pid.set_target((inches*180*3)/((PI*config.wheel_diam*5)));

    initialize_func = false;
  }

  // Update PID loop and drive the robot based on it's output
  drive_pid.update(lf.position(deg)); //get average position
  double pid_out = drive_pid.get(); //get output
  sign  = (pid_out > 0) ? 1 : -1;;

    //slew rate
      if(fabs(pid_out) > vMax)
        pid_out = vMax*sign; //is the output greater than the absolute speed

      vCap += accel*sign; //increase the temporary max velocity cap

      if(fabs(vCap) > fabs(vMax)){
        vCap = vMax*sign; //is the temporary max voltage greater than the absolute max?
      }

      if(fabs(pid_out) > fabs(vCap))
        pid_out = vCap; //constrain to temporary max speed

  //debug
  std::cout << "p: "<< pid_out - (gyro_sensor.heading(deg)-prevAngle) <<std::endl; //power output
  std::cout << "e: "<< drive_pid.get_error() <<std::endl; //error
  //std::cout << "a: "<< <<std::endl; //angle
  std::cout << "" <<std::endl; 


  drive_tank(pid_out , pid_out, volt); //output PID with straight line

  // If the robot is at it's target, return true
  if (drive_pid.is_on_target())
  {
    drive_tank(0, 0, volt);
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
  drive_tank(pid_out, -1 * pid_out, velocityUnits::pct);

  // If the robot is at it's target, return true
  if (turn_pid.is_on_target())
  {
    drive_tank(0, 0, volt);
    gyro_sample.stop();
    initialize_func = true;
    return true;
  }

  return false;
}