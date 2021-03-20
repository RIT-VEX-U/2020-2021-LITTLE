#include "hardware.h"

// Initialize Hardware below
// Form: [class] Hardware::[name](parameters);

// -- BRAIN & CONTROLLER(S) --
brain Hardware::v5_brain;
controller Hardware::master(controllerType::primary);
controller Hardware::partner(controllerType::partner);

// -- SENSORS --
inertial Hardware::inertia(PORT16);
distance Hardware::indexer(PORT6);
limit Hardware::lowerIndexer(Hardware::v5_brain.ThreeWirePort.B);
line Hardware::intakeIndexer(Hardware::v5_brain.ThreeWirePort.A);
distance Hardware::goalSensor(PORT10);

// -- MOTORS --
motor Hardware::lf(PORT14, gearSetting::ratio18_1), Hardware::rf(PORT18, gearSetting::ratio18_1, true),
      Hardware::lr(PORT13, gearSetting::ratio18_1, true), Hardware::lr2(PORT2, gearSetting::ratio18_1, true),
      Hardware::rr(PORT17, gearSetting::ratio18_1), Hardware::rr2(PORT9, gearSetting::ratio18_1);

//MecanumDrive Hardware::mec_drive(lf, rf, lr, rr);

// WARNING: NOT TUNED! These are filler values
TankDrive::tankdrive_config_t tank_config = {
  (PID::pid_config_t) {
    // p, i, d, f
    //1.0, 0, 0, 0,
    .95, 0, 0, 0,
    // deadband, on_target_time
    0.05, 0
  },
  (PID::pid_config_t) {
    // p, i, d, f
    0.005, 0, 0, 0,
    // deadband, on_target_time
    1.5, 0
  
  },
  // wheel diam
  3.25,
  // wheel : motor ratio
  1.6667
};
motor_group left_motors = {Hardware::lf, Hardware::lr, Hardware::lr2};
motor_group right_motors = {Hardware::rf, Hardware::rr, Hardware::rr2};
TankDrive Hardware::tank_drive(left_motors, right_motors, Hardware::inertia, tank_config);

motor Hardware::intakeLeft(PORT11);
motor Hardware::intakeRight(PORT19, true);
motor_group Hardware::intake(Hardware::intakeLeft, Hardware::intakeRight);

motor Hardware::bottom_roller1(PORT12, gearSetting::ratio6_1);
motor Hardware::bottom_roller2(PORT7, gearSetting::ratio6_1, true);
motor_group Hardware::bottom_roller(bottom_roller1, bottom_roller2);
motor Hardware::top_roller(PORT15, gearSetting::ratio6_1);

// End Hardware Initialization

//HARDWARE CONTROL
using namespace Hardware;
/**
* Directly set the voltage of each motor on the uptake
*/
void uptake(int top, int middle, int bottom){
  bottom_roller1.spin(fwd, bottom, volt);
  bottom_roller2.spin(fwd, bottom, volt);
  top_roller.spin(fwd, top, volt);
}
/**
* Same as above but control how many revolutions 
*/
void uptakeRevolution(int top, int bottom, int speed){
  bottom_roller.rotateFor(bottom, rev, speed, velocityUnits::pct);
  top_roller.rotateFor(top, rev, speed, velocityUnits::pct, true); 
}
/**
* Stop all motors on the uptake system
*/
void stopIntaking(){
  intake.stop();
  bottom_roller.stop();
  top_roller.stop();
}

/**
* This sub function carries out the deployment and won't allow the run to continue
* unless the robot has deployed
*/
void deploy(){
  bottom_roller.spin(reverse, 13, volt);
    wait(400, msec);
  bottom_roller.spin(fwd, 13, volt);
    wait(250, msec);
  top_roller.spin(reverse, 13, volt);
  bottom_roller.spin(reverse, 13, volt);
    wait(200, msec);
  top_roller.stop();
  bottom_roller.stop();
}

timer shootPause = timer();

void shootIndexer(){
  int counter = 0;
  float prevTime = shootPause.time(msec);
  intake.spin(fwd, 13, volt);
while(1){
    if(counter == 0){ //only executes upon entering the loop
      top_roller.spin(fwd, 13, volt); //get the top roller up to speed
      counter++;
      wait(100, msec);
    }

    bottom_roller2.spin(fwd, 13, volt); //shoot top ball
    bottom_roller1.spin(reverse, 10, volt); //hold back botttom ball

    if(indexer.objectDistance(mm) <= 20 || shootPause.time() - prevTime > 2000){ //if the top ball reaches the top sensor or timer passes
      wait(250, msec); //wait until the ball fully exits
      top_roller.spin(reverse, 13, volt); //bring bottom ball back down to shooting position
      bottom_roller1.spin(fwd, 13, volt);
      
      while(indexer.objectDistance(mm) > 200 || shootPause.time() - prevTime > 4000) //wait untill ball shoots out or after 4 secs
        wait(10,msec); //spin the uptake until the 2nd ball reaches index 
        break;
    }
    
    wait(20,msec);
  }
  bottom_roller.rotateFor(reverse, .1, rev, 300, velocityUnits::rpm, false);
  top_roller.rotateFor(reverse, 1, rev, 600, velocityUnits::rpm, false);
}

void indexing(){
  int counter = 0;
  float prevTime = shootPause.time(msec);
  intake.spin(fwd, 13, volt);
 while(master.ButtonL1.pressing()){
    if(counter == 0){ //only executes upon entering the loop
      top_roller.spin(fwd, 13, volt); //get the top roller up to speed
      counter++;
      wait(100, msec);
    }

    bottom_roller2.spin(fwd, 13, volt); //shoot top ball
    bottom_roller1.spin(reverse, 10, volt); //hold back botttom ball

    if(indexer.objectDistance(mm) <= 20 || shootPause.time() - prevTime > 2000){ //if the top ball reaches the top sensor or timer passes
      wait(250, msec); //wait until the ball fully exits
      bottom_roller2.spin(fwd, 13, volt); //shoot top ball
      bottom_roller1.spin(fwd, 13, volt); //hold back botttom ball
      break;
    }
    
    wait(20,msec);
  }
  top_roller.stop();
  bottom_roller.stop();
  bottom_roller.rotateFor(reverse, .1, rev, 300, velocityUnits::rpm, false);
  top_roller.rotateFor(reverse, 1, rev, 600, velocityUnits::rpm, false);
}




