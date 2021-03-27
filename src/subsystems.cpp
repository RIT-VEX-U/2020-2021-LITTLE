#include "competition/opcontrol.h"
#include <iostream>

//HARDWARE CONTROL
using namespace Hardware;

//global variables within file
int ballCounter = 0; //track robot states 
bool bottomSensor = false, topSensor = false;

//TIMERS
timer shootDelay = timer(); //timer tracking the timing of a shot
timer t = timer(); //general timer


/**
* Directly set the voltage of each motor on the uptake
*/
void uptake(int top, int middle, int bottom){
  bottom_roller1.spin(fwd, bottom, volt);
  bottom_roller2.spin(fwd, bottom, volt);
  mid_roller.spin(fwd, middle, volt);
  top_roller.spin(fwd, top, volt);
}

/**
* Directly control how many revolutions each motor makes
*/
void uptakeRevolution(int top, int bottom, int speed){
  bottom_rollers.rotateFor(bottom, rev, speed, velocityUnits::pct);
  top_roller.rotateFor(top, rev, speed, velocityUnits::pct, true); 
}

/**
* Stop all motors on the uptake and intake system
*/
void stopAll(){
  intake.stop();
  bottom_rollers.stop();
  top_roller.stop();
}

/**
* at the start of opcontrol/autonomous deploy hood and intakes
*/
void deploy(){
      bottom_roller2.spin(reverse, 13, volt); //release hood
      bottom_roller1.spin(reverse, 13, volt); //release intakes
      mid_roller.spin(fwd, 13, volt);
      wait(100,msec);
  while(bottom_roller1.torque(Nm) > .25 || mid_roller.torque(Nm) > .25){ //wait until the load is released from the motor
   wait(20,msec);
  }
  top_roller.spin(fwd, 13, volt); //prevent hood from catching
  bottom_rollers.stop();
  wait(500, msec); //allow ball to shoot back
  top_roller.stop();
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
    if(topSensor && bottomSensor)
      ballCounter = 2;
    else if(topSensor && bottomSensor)
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
    double timeEntered = t.time(msec);

    while(!topSensor){
      if(t.time() - timeEntered > 800)
        break;
        
      uptake(-10,2,8); //move ball up to second roller
      
      wait(10,msec);
    } //while the ball is not indexed 

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
         uptake(-5, 2, 0);
      }
   }else if(master.ButtonR2.pressing()){ //regular outtake
        intake.spin(reverse, 13, volt);
        uptake(-13, -13, -13);
       if(master.ButtonL2.pressing()){ //center goal descore
         bottom_rollers.stop(brake);
         top_roller.stop(brake);
         intake.spin(reverse, 13, volt); //only outtake intakes
       }
   }else{
      //bottom_rollers.stop(brake);
      mid_roller.spin(fwd, 2, volt);
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

void shoot(int balls, bool indexBall = true){ 
  bool prevVal = topSensor, ballShot;
  int entryTime, shootTime = 1000, indexTime = 800, exitTime; //shoot time determines how long before loop cuts out to prevent stuck loops

  for(int i = 0; i < balls; i++){ //cycle through a shoot cycle depending on how many balls need to be shot
    ballShot = false; //reset exit conditions
    entryTime = t.time(msec);
    while(!ballShot){ //run the uptake until a ball gets shot
      if(t.time(msec) - entryTime > shootTime) //if you get stuck shooting
        break;

       top_roller.spin(fwd, 13, volt);
       wait(200, msec); //get up to speed –– first ball should always be well under the top roller
       mid_roller.spin(fwd, 13, volt); //shoot top indexed ball only

       if(topSensor != prevVal){ //if the ball leaves the top sensor -- presumably shot by the top roller
          ballShot = true; //allow top ball to exit fully because sensor is placed lower than top roller
          wait(250, msec);
       }

      wait(20, msec);
    }
    exitTime = t.time(msec);
    //after ball gets shot
    top_roller.spin(reverse, 8, volt); //prevent other balls from exiting

     //if there is a ball to be indexed -- skips after first iteration
      while(!topSensor && indexBall){
        if(t.time(msec) - exitTime > indexTime)
          break;
        uptake(0,2,8);
        wait(20,msec);
    }
  }
  uptake(0,0,0); //stop all after exiting everything
}

