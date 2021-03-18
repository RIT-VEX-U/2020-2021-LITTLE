#include "competition/opcontrol.h"
#include <iostream>
using namespace Hardware;

//INITIALIZE GLOBAL VARIABLES

// flag to be used when bottom roller is running in other functions
bool bottom_running = false;

// -- TIME OUT --

// Using a timer from the vex api, a task will determine whether or not a
// specified amount of milliseconds has passed since a function began.
// To use the time out, begin the task at the start of a function with:
//    task time_out_task = task(&timeOut).
// The global variable:
//    time_out 
// can be used to tell whether or not the function has used up its time.

timer t = timer();

// Represents whether or not a function has used up its time
bool time_out = false;

/*
 * Should be used as a task at the start of a function or loop
 * @return task functions need to return an int for some
 *  reason, so just ignore the return statement
 */
int timeOut() {
  // reset
  time_out = false;
  t.clear();

  // Note: may change to a thread so I can send this an arg for how much time
  // the function / loop can run. For now, the default is 500 ms
  while(t.time(timeUnits::msec) < 500) {}

  time_out = true;
  return 1;
}

// -- INDEXING --

// Check if an object has reached a certain point in the robot's intake
// system through looping in a task until the distance threshhold has
// been met.
// To use indexing, begin the task by calling:
//    startIndexerChecking()
// If the threshhold has not been met and you wish to break out of the loop, call:
//    endIndexerChecking()

// global ref to a task so it can be stopped anywhere in the file
task check_indexer_task;

//global ref to number of balls currently stored in the robot 
int ballCount = 0;

/*
 * Run as a task
 * Continuously checks if an object is within X [mm] of the top distance sensor
 * If there is an object there, stop the bottom rollers
 */
int checkIndexer() {
  bottom_running = true;

  while(indexer.objectDistance(distanceUnits::mm) > 80) {}
  bottom_roller.stop(brakeType::brake);

  bottom_running = false;
  return 0;
}

void startIndexerChecking() {
  check_indexer_task = task(&checkIndexer);
  // give bottom_running flag time to be set
  wait(10, timeUnits::msec);
}


void endIndexerChecking() {
  check_indexer_task.stop();
  bottom_running = false;
  // give bottom_running flag time to be set
  wait(10, timeUnits::msec);
}


// -- SCORING --

/*
 * Initially used for color sorting, will probably be removed soon
 */
void score() {
  wait(100, timeUnits::msec);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);

  task time_out_task = task(&timeOut);
  wait(10, timeUnits::msec);  // wait for time_out to be set to false again

  bottom_roller.stop();
  top_roller.stop();
}

void user_deploy(){
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

/*
 * Function called when scoring in user control
 * (also initially created for color sorting but this seems more useful)
 */
void userScore() {
  bottom_running = true;
  intake.spin(fwd, 13, volt);
  bottom_roller.spin(directionType::fwd, 13, voltageUnits::volt);
  top_roller.spin(directionType::fwd, 13, voltageUnits::volt);
  
  while(master.ButtonL1.pressing()) {} //this is where indexing code goes

  bottom_roller.stop();
  top_roller.stop();
  bottom_running = false;
}
///////////////////////////////////////////
// -- chassis control -- MV
// -- tasks --
vex::thread dTask, intakeTask, shootTask;
vex::thread sensorStateTask, debugTask;

mutex mtx; //protect tasks

int ballCounter = 0;
bool bottomSensor = false, midSensor = false, topSensor = false;

/**
* Task for chassis control 
*/
int driveTask(){
  while(1){
   tank_drive.drive_tank(master.Axis3.position() / 100.0, master.Axis2.position() / 100.0); //control chassis

   this_thread::sleep_for(20);
  }
  return(0);
}

/**
* Continually get state of the balls in the robot
*/
int sensorState(){
  while(1){
    mtx.lock(); //protect multi thread calls

    if(indexer.objectDistance(mm) < 120){ //is there a ball at the top?
      topSensor = true;
    }else
      topSensor = false;

    if(lowerIndexer.objectDistance(mm) < 120) //is there a ball in the middle?
      midSensor = true;
    else
      midSensor = false;

    if(intakeIndexer.objectDistance(mm) < 120) //is there a ball in the intakes?
      bottomSensor = true;
    else
      midSensor = false;
  
      this_thread::sleep_for(20);

      mtx.unlock(); //ditto
  }
  return(0);
}

/**
* Determine how many balls are in the robot
*/
int getBallCount(){
  while(1){
    if(topSensor == true && midSensor == true && bottomSensor == true)
      ballCounter = 3;
    else if((topSensor == true && midSensor == true && bottomSensor == false) || (topSensor == false && midSensor == true && bottomSensor == true))
      ballCounter = 2;
    else if((topSensor == true && midSensor == false && bottomSensor == false) || (topSensor == false && midSensor == true && bottomSensor == false))
      ballCounter = 1;
    else if((topSensor == false && midSensor == false && bottomSensor == true))
      ballCounter = 1; //couldn't fit condition on previous line
    else
      ballCounter = 0;

    this_thread::sleep_for(20);
  }
  return 0;
}

/**
* Intake function, automatically indexes balls based on states of sensor
*/
int runIntake(){
  while(1){
  mtx.lock();

  if(master.ButtonR1.pressing()){ //is the button being pressed?
    if(topSensor == false && bottomSensor == false){ //if there are no balls in the robot
      intake.spin(fwd, 13, volt);
      bottom_roller.spin(fwd, 13, volt); //run uptake until ball reaches the top roller
    }else if (topSensor == true && midSensor == false) { // if there is a ball at the top
      intake.spin(fwd, 13, volt);
      bottom_roller1.spin(fwd, 13, volt); //run bottom roller until ball reaches desired state
    }else
      intake.spin(fwd, 13, volt); //otherwise, just run intakes
   }

   this_thread::sleep_for(20);
   mtx.unlock();
  }
  return(0);
}

/**
* Shoot balls based on states of sensor
*/
int shootBalls(){
  while(1){
    if(master.ButtonL1.pressing()){ //only run if button is being pressed
       intake.spin(fwd, 13, volt); //spin the intakes and top roller in any case
       top_roller.spin(fwd, 13, volt); 

       if(topSensor == true){
        bottom_roller2.spin(fwd, 13, volt); //if there's a ball in the top roller, only run those the top roller
        bottom_roller1.stop();
       }else
        bottom_roller.spin(fwd, 13, volt); //run both rollers otherwise
      /*
       if(topSensor == true && midSensor == true){ //if there are two balls in the robot
        //shoot only the top ball in the robot by spinning mid roller only
        bottom_roller2.spin(fwd, 13, volt); //this case will switch after the first ball exits, automatically

       }else if(topSensor == false && midSensor == true){ //after the first ball gets shot out
        bottom_roller.spin(fwd, 13, volt); //spin both rollers
       }else 
        bottom_roller.spin(fwd, 13, volt);
        */

     }else{
        bottom_roller.stop();
        top_roller.stop();
        intake.stop();
     }
   this_thread::sleep_for(20);
  }
  return 0;
}

int debug(){
  while(1){
    std::cout << "Top State: " << topSensor << std::endl;
    std::cout << "Mid State: " << midSensor << std::endl;
    std::cout << "Bottom State: " << bottomSensor << std::endl;

    this_thread::sleep_for(20);
  }
  return 0;
}

/**
 * Code for the Driver Control period is executed below.
 */
 /*
 void OpControl::opcontrol(){
  
  //allow tasks to run in background
   dTask = thread(driveTask);
   sensorStateTask = thread(sensorState);
   intakeTask = thread(runIntake);
   shootTask = thread(shootBalls);
   debugTask = thread(debug);
  
     while(1){
     //allow other functions to run
     vexDelay(10);
   }
 }
*/
///////////////////////////////////////////


void OpControl::opcontrol()
{
  //user_deploy();
  master.ButtonL1.pressed(&userScore); //why is this outside the main loop?
  // OpControl Init
  
  // OpControl Loop
  while (true)
  {
    tank_drive.drive_tank(master.Axis3.position() / 100.0, master.Axis2.position() / 100.0);
    // intake
    if(master.ButtonR1.pressing()) {
      // intakes are unaffected by indexing –– always spinning
      intake.spin(directionType::fwd, 13, voltageUnits::volt);

      // indexing
      startIndexerChecking(); //is there a ball at the top sensor?
      while(bottom_running && master.ButtonR1.pressing()) {
        // this is gross but I couldn't quickly think of a better way to keep driving while indexing :(
        tank_drive.drive_tank(master.Axis3.position() / 100.0, master.Axis2.position() / 100.0);
        bottom_roller.spin(directionType::fwd, 13, voltageUnits::volt); //run the bottom rollers until the ball reaches the top sensor
      }
      bottom_roller.stop();
      endIndexerChecking();
    }
    // de-intake
    else if(master.ButtonR2.pressing()) {
      intake.spin(directionType::rev, 13, voltageUnits::volt);
      bottom_roller.spin(directionType::rev, 10, voltageUnits::volt);
      top_roller.spin(reverse, 13, volt);
    }
    else {
      intake.stop();
      if(!bottom_running){
        bottom_roller.stop();
        top_roller.stop();
      }
    }

    vexDelay(10); // Small delay to allow time-sensitive functions to work properly.
  }
}
