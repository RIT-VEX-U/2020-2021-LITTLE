#include "competition/opcontrol.h"
#include <iostream>
using namespace Hardware;

//INITIALIZE GLOBAL VARIABLES
// -- tasks --
vex::thread stateTask, driveTask;

int ballCounter = 0, goalLevel = 1, prevBallCount = 0, currentState = 0; //track sensor states within file
bool bottomSensor = false, midSensor = false, topSensor = false, bottom_running;

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

// -- CHASSIS --
/**
* Control chassis
*/
int move(){
  while(1){
   tank_drive.drive_tank(master.Axis3.position() / 100.0, master.Axis2.position() / 100.0); //control chassis
   this_thread::sleep_for(20);
  }
  return 0;
}


// -- INDEXING --

/**
* Continually get state of the balls in the robot
*/
void updateSensorState(){
    if(indexer.objectDistance(mm) < 5) //is there a ball at the top?
      topSensor = true;
    else
      topSensor = false;

    //if(lowerIndexer.objectDistance(mm) < 50) //is there a ball in the middle? -- distance sensor broke RIP
     if(lowerIndexer.pressing())
      midSensor = true;
    else
      midSensor = false;

    if(intakeIndexer.value(mV) < 2500) //is there a ball in the intakes?
      bottomSensor = true;
    else
      bottomSensor = false;

    //std::cout << "Top State: " << topSensor << std::endl;
    //std::cout << "Mid State: " << midSensor << std::endl;
    //std::cout << "Bottom State: " << bottomSensor << std::endl;
    //std::cout << "" << std::endl;
}

/**
* Determine how many balls are in the robot
*/
void updateBallCount(){
    if(topSensor && midSensor && bottomSensor)
      ballCounter = 3;
    else if((topSensor && midSensor && !bottomSensor) || (!topSensor && midSensor && bottomSensor))
      ballCounter = 2;
    else if((topSensor && !midSensor && !bottomSensor) || (!topSensor && midSensor && !bottomSensor)|| (!topSensor && !midSensor && bottomSensor))
      ballCounter = 1;
    else
      ballCounter = 0;
}

/**
* Determine how many balls in the goal –– defaults to 1
*/
void updateGoalLevel(){
    if(goalSensor.objectDistance(mm) <= 110) //if there's 2 blue balls in the goal?
      goalLevel = 2;
    else 
      goalLevel = 1;
}

/**
* Get ball states in the goal and robot -- for shooting function
*/
int getCurrentState(){
  while(1){
    updateSensorState(); //ditto
    updateBallCount();
    updateGoalLevel();

  if(goalLevel == 2 && ballCounter == 1) //while there's two balls in the goal and one in the robot
    currentState = 1;
  
  if(goalLevel == 2 && ballCounter == 2) //while there's two balls in the goal and two balls in the robot
    currentState = 2;
  
  if(goalLevel == 1  && ballCounter == 1) //while there's one ball in the goal and one in the robot
    currentState = 3;
  
  if(goalLevel == 1  && ballCounter == 2) //while there's one ball in the goal and two balls in the robot
    currentState = 4;
    
  std::cout << "current state: " << currentState <<std::endl;

    this_thread::sleep_for(20);
  }

  return(0);
}

// -- INTAKING --
/**
* Intake function, automatically indexes balls based on states of sensor
*/
void runIntake(){
   if(master.ButtonR1.pressing()){ //is the button being pressed?

    if(topSensor == false && bottomSensor == false){ //if there are no balls in the robot
       intake.spin(fwd, 13, volt);
       bottom_roller1.spin(fwd, 8, volt); //run uptake until ball reaches the top roller
       bottom_roller2.spin(fwd, 2, volt);
       top_roller.spin(reverse, 13, volt);
     }else if (topSensor == true && midSensor == false) { // if there is a ball at the top
       intake.spin(fwd, 13, volt);
       bottom_roller1.spin(fwd, .5, volt); //run bottom roller until ball reaches desired state 
       top_roller.spin(reverse, 13, volt);
     }else{
       intake.spin(fwd, 13, volt); //otherwise, just run intakes
       bottom_roller.stop(brake);
     }
   }else if(master.ButtonL2.pressing()){ //regular outtake
       intake.spin(reverse, 13, volt);
       bottom_roller.spin(reverse, 13, volt);
       top_roller.spin(reverse, 13, volt);
      if(master.ButtonR2.pressing()){//if shift key is being pressed with the descore button–– center goal descore
         bottom_roller.stop(brake);
         top_roller.stop(brake);
         intake.spin(reverse, 13, volt); //only outtake intakes
       } 

   }else{
      bottom_roller.stop(brake);
      top_roller.spin(reverse, 5, volt);
      intake.stop();
   }
}

// -- SCORING --
/*
 * shoots one ball at a time 
 */
 void shoot(int balls){ //how many balls will get shot
   bool prevVal = topSensor, ballShot, ballExitFully;
   int entryTime = t.time(msec);

  for(int i = 0; i <= balls; i++){
    ballShot = false, ballExitFully = false;
    while(!(ballShot && ballExitFully)){ //run the uptake until a ball gets shot
      if(t.time(msec) - entryTime > 1500) //if you get stuck shooting
        break;

       top_roller.spin(fwd, 13, volt);
       bottom_roller2.spin(fwd, 13, volt);

       if(topSensor != prevVal){
          ballShot = true; //allow top ball to exit fully because sensor is placed lower than top roller
          wait(500, msec);
         ballExitFully = true;
       }

      wait(20, msec);
    }
    if( t.time(msec) - entryTime > 3000) //if you get stuck shooting
        break;
    //after ball gets shot
    top_roller.spin(reverse, 8, volt);
    bottom_roller2.stop();

    if(i == 1){ //if there is a ball to be indexed 
      while(!topSensor){
        bottom_roller1.spin(fwd, 8, volt);
        bottom_roller2.spin(fwd, 2, volt);
        wait(20,msec);
      }
    }
  }
}

void shootBalls(){
   if(master.ButtonL1.pressing()){ //only run if button is being pressed 

      switch(currentState){ //arg is global variable updated in seperate task

        case 1: //descore two balls and shoot one 
         intake.rotateFor(2, rev, 100,velocityUnits::pct, false); //descore 2
         shoot(1);
         break;

        case 2: //descore two balls, shoot two
         intake.rotateFor(2, rev, 100,velocityUnits::pct, false); //descore 2
         shoot(2);
         break;

        case 3: //descore one, shoot one
         intake.rotateFor(1, rev, 100,velocityUnits::pct, false); //descore 1
         shoot(1);
         break;

        case 4: //descore one, shoot two
         intake.rotateFor(1, rev, 100,velocityUnits::pct, false); //descore 1
         shoot(2);
         break;
      }
   }
}

/*
void shootBalls(){
   if(master.ButtonL1.pressing()){ //only run if button is being pressed 

      switch(currentState){ //arg is global variable updated in seperate task

        case 1: //descore two balls and shoot one 
         intake.spin(fwd,13,volt);
         bottom_roller2.spin(fwd, 13, volt); //only spin mid roller and top
         top_roller.spin(fwd, 13, volt);
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
         break;
      }
   }
}
*/

//OLD CODE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
   stateTask = thread(getCurrentState); //update sensors and state functions  
   driveTask = thread(move); //allow chassis to move independent of everything

   while(1){
     //allow other functions to run

    //run intaking and shooting on the same thread –– one function automatically takes priority
     runIntake(); //determines when to intake/outtake
     shootBalls(); //determines how much to rotate rollers

     vexDelay(10);
   }
 }