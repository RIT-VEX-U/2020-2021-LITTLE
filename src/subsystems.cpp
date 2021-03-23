#include "competition/opcontrol.h"
#include <iostream>

//HARDWARE CONTROL
using namespace Hardware;

//global variables within file
int ballCounter = 0, goalLevel = 1, prevBallCount = 0, currentState = 0; //track robot states 
bool bottomSensor = false, midSensor = false, topSensor = false;

//TIMERS
timer shootDelay = timer(); //timer tracking the timing of a shot
timer t = timer(); //general timer


/**
* Directly set the voltage of each motor on the uptake
*/
void uptake(int top, int middle, int bottom){
  bottom_roller1.spin(fwd, bottom, volt);
  bottom_roller2.spin(fwd, middle, volt);
  top_roller.spin(fwd, top, volt);
}

/**
* Directly control how many revolutions each motor makes
*/
void uptakeRevolution(int top, int bottom, int speed){
  bottom_roller.rotateFor(bottom, rev, speed, velocityUnits::pct);
  top_roller.rotateFor(top, rev, speed, velocityUnits::pct, true); 
}

/**
* Stop all motors on the uptake and intake system
*/
void stopAll(){
  intake.stop();
  bottom_roller.stop();
  top_roller.stop();
}

/**
* at the start of opcontrol/autonomous deploy hood and intakes
*/
void deploy(){
  bottom_roller2.spin(fwd, 13, volt); //release hood
  bottom_roller1.spin(reverse, 13, volt); //release intakes
  wait(500,msec); //allow time for all parts to release
  bottom_roller.stop(); 
}

// -- INDEXING -- 

/**
* Continually get state of the balls in the robot
*/
void updateSensorState(){
    if(indexer.value() < 10) //is there a ball at the top?
      topSensor = true;
    else
      topSensor = false;

    if(lowerIndexer.value() < 30) //is there a ball in the middle? -- distance sensor broke RIP
      midSensor = true;
    else
      midSensor = false;

    if(intakeIndexer.value(mV) < 2500) //is there a ball in the intakes?
      bottomSensor = true;
    else
      bottomSensor = false;

    //debug
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
* Get ball states in the goal and robot -- for shooting function
*/
int getCurrentState(){
  while(1){
    updateSensorState(); //ditto
    updateBallCount();
    
    this_thread::sleep_for(20);
  }

  return(0);
}


/**
* Index one ball currently at the bottom roller by moving it to the
* middle roller but right under the top roller
*/

void index(){ //for after shooting
    uptake(-10,2,6); //move ball up to second roller

    while(!topSensor){wait(10,msec);} //while the ball is not indexed 

    uptake(-5, 5, -3); //backout any balls that over indexed
    wait(200,msec);

    uptake(0,0,0); //stop
}


// -- INTAKING --
/**
* Intake function, automatically indexes balls based on states of sensor
*/
void runIntake(){
   if(master.ButtonR1.pressing()){ //is the button being pressed?
       if(!topSensor){ //if there is no ball at the top of the robot
        intake.spin(fwd, 13, volt);
        uptake(13,2,8); //run uptake until ball reaches the top roller
      }else if (topSensor) { // if there is a ball at the top
         intake.spin(fwd, 13, volt);
         uptake(-5, 2, 1);
      }else{
         intake.spin(fwd, 13, volt); //otherwise, just run intakes
         bottom_roller.stop(brake);
      }
   }else if(master.ButtonL2.pressing()){ //regular outtake
        intake.spin(reverse, 13, volt);
       uptake(-13, -13, -13);
       if(master.ButtonR2.pressing()){ //center goal descore
         bottom_roller.stop(brake);
         top_roller.stop(brake);
         intake.spin(reverse, 13, volt); //only outtake intakes
       }
   }else{
      bottom_roller.stop(brake);
      //top_roller.spin(reverse, 4, volt); //always run backwards unless shooting
      top_roller.stop();
      intake.stop();
   }
}

// -- shooting --

/**
* Spins the uptake until a ball is detected to be shot
* Proceeds to index the next ball in the robot 
*
* @param number of balls to be shot 
*/

void shoot(int balls){ 
  bool prevVal = topSensor, ballShot;
  int entryTime, shootTime = 1000, indexTime = 500 ,exitTime; //shoot time determines how long before loop cuts out to prevent stuck loops

  for(int i = 0; i < balls; i++){ //cycle through a shoot cycle depending on how many balls need to be shot
    ballShot = false; //reset exit conditions
    entryTime = t.time(msec);
    while(!ballShot){ //run the uptake until a ball gets shot
      if(t.time(msec) - entryTime > shootTime) //if you get stuck shooting
        break;

       top_roller.spin(fwd, 13, volt);
       wait(200, msec); //get up to speed –– first ball should always be well under the top roller
       bottom_roller2.spin(fwd, 13, volt); //shoot top indexed ball only

       if(topSensor != prevVal){ //if the ball leaves the top sensor -- presumably shot by the top roller
          ballShot = true; //allow top ball to exit fully because sensor is placed lower than top roller
          wait(200, msec);
       }

      wait(20, msec);
    }
    exitTime = t.time();
    //after ball gets shot
    top_roller.spin(reverse, 8, volt); //prevent other balls from exiting

     //if there is a ball to be indexed -- skips after first iteration
      while(!topSensor){
        if(t.time(msec) - exitTime > indexTime)
          break;
        uptake(0,2,8);
        wait(20,msec);
    }
  }
  uptake(0,0,0); //stop all after exiting everything
}

