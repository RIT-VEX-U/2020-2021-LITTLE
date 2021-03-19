#include "competition/opcontrol.h"
#include <iostream>
using namespace Hardware;

//INITIALIZE GLOBAL VARIABLES
// -- tasks --
vex::thread intakeTask, shootTask;
vex::thread sensorStateTask, goalTask;

mutex mtx; //protect tasks

int ballCounter = 0, goalLevel = 1, prevBallCount = 0;
int* currentState = 0;
bool bottomSensor = false, midSensor = false, topSensor = false;


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

/**
* Continually get state of the balls in the robot
*/
int sensorState(){
  while(1){
    mtx.unlock(); //protect multi thread calls

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

    std::cout << "Top State: " << topSensor << std::endl;
    std::cout << "Mid State: " << midSensor << std::endl;
    std::cout << "Bottom State: " << bottomSensor << std::endl;

      this_thread::sleep_for(20);

      mtx.lock(); //ditto
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
* Determine how many balls in the goal –– defaults to 1
*/
int updateGoalLevel(){
  while(1){
    if(goalSensor.value() <= 100) //if there's 2 blue balls in the goal?
      goalLevel = 2;
    else 
      goalLevel = 1;
  }
  return(0);
}

/**
* Get ball states in the goal and robot -- for shooting function
*/
int getCurrentState(){
  while(1){
  if(goalLevel == 2 && getBallCount() == 1) //while there's two balls in the goal and one in the robot
    *currentState = 1;
  
  if(goalLevel == 2 && getBallCount() == 2) //while there's two balls in the goal and two balls in the robot
    *currentState = 2;
  
  if(goalLevel == 1  && getBallCount() == 1) //while there's one ball in the goal and one in the robot
    *currentState = 3;
  
  if(goalLevel == 1  && getBallCount() == 2) //while there's one ball in the goal and two balls in the robot
    *currentState = 4;
    
    this_thread::sleep_for(20);
  }

  return(0);
}

// -- INTAKING --
/**
* Intake function, automatically indexes balls based on states of sensor
*/
int runIntake(){
  while(1){

  if(master.ButtonR1.pressing()){ //is the button being pressed?
    if(topSensor == false && bottomSensor == false){ //if there are no balls in the robot
      intake.spin(fwd, 13, volt);
      bottom_roller.spin(fwd, 13, volt); //run uptake until ball reaches the top roller
    }else if (topSensor == true && midSensor == false) { // if there is a ball at the top
      intake.spin(fwd, 13, volt);
      bottom_roller1.spin(fwd, 13, volt); //run bottom roller until ball reaches desired state –– when both are true
    }else
      intake.spin(fwd, 13, volt); //otherwise, just run intakes
   }

   this_thread::sleep_for(20);
  }
  return(0);
}

// -- SCORING --
int shootBalls(int* currentState){
  while(1){
   if(master.ButtonL1.pressing()){ //only run if button is being pressed

    prevBallCount = getBallCount(); //check to see if a ball was intaked/shot

      switch(*currentState){
       case 1: //descore two balls and shoot one 
         intake.rotateFor(2, rev, 100,velocityUnits::pct);
         bottom_roller2.rotateFor(2, rev, 100, velocityUnits::pct); //only spin mid roller and top
         top_roller.rotateFor(2, rev, 100, velocityUnits::pct);
         bottom_roller1.stop(brake); //don't bring up second ball until first ball gets shot –– threshold determines when ball gets shot

         break;

       case 2: //descore two balls, shoot two
         bottom_roller1.stop(brake); //don't bring up second ball until first ball gets shot –– threshold determines when ball gets shot
         intake.rotateFor(2, rev, 100,velocityUnits::pct); //descore 1
         bottom_roller2.rotateFor(2, rev, 100, velocityUnits::pct); //only spin mid roller and top
         top_roller.rotateFor(2, rev, 100, velocityUnits::pct, false); //wait until the ball gets shot out
         
         intake.rotateFor(2, rev, 100,velocityUnits::pct); //descore 1
         bottom_roller1.rotateFor(1, rev, 100, velocityUnits::pct); //bring bottom ball up only
         bottom_roller2.rotateFor(2, rev, 100, velocityUnits::pct); //shoot bottom ball only
         top_roller.rotateFor(2, rev, 100, velocityUnits::pct, false); //wait until the ball gets shot out

         break;

        case 3: //descore one, shoot one
         bottom_roller1.stop(brake); //don't bring up any other balls
         intake.rotateFor(2, rev, 100,velocityUnits::pct); //descore 1
         bottom_roller2.rotateFor(2, rev, 100, velocityUnits::pct); //only spin mid roller and top
         top_roller.rotateFor(2, rev, 100, velocityUnits::pct, false); //wait until the ball gets shot out

          break;

        case 4: //descore one, shoot two
         bottom_roller1.stop(brake); //don't bring up any other balls
         intake.rotateFor(2, rev, 100,velocityUnits::pct); //descore 1
         bottom_roller2.rotateFor(2, rev, 100, velocityUnits::pct); //only spin mid roller and top
         top_roller.rotateFor(2, rev, 100, velocityUnits::pct, false); //wait until the ball gets shot out

      }

   this_thread::sleep_for(20);
   }
 }
  return 0;
}


//OLD CODE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * Initially used for color sorting, will probably be removed soon -- OLD
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


/*
 * Function called when scoring in user control
 * (also initially created for color sorting but this seems more useful) -- OLD
 */
void userScore() {
  bottom_running = true;
  intake.spin(fwd, 13, volt);
  bottom_roller.spin(directionType::fwd, 13, voltageUnits::volt);
  top_roller.spin(directionType::fwd, 13, voltageUnits::volt);
  
  while(master.ButtonL1.pressing()) {}

  bottom_roller.stop();
  top_roller.stop();
  bottom_running = false;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Code for the Driver Control period is executed below.
 */
 
 void OpControl::opcontrol(){
  
  //allow tasks to run in background
   sensorStateTask = thread(sensorState);
   intakeTask = thread(runIntake);
   goalTask = thread(getCurrentState); //continually update the state of the goals and robot
   shootTask = thread(shootBalls(currentState)); //pass in a variable 
  
     while(1){
     //allow other functions to run
     tank_drive.drive_tank(master.Axis3.position() / 100.0, master.Axis2.position() / 100.0); //control chassis 

     vexDelay(10);
   }
 }