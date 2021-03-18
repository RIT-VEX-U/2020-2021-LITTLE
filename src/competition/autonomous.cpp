#include "competition/autonomous.h"
#include <iostream>

using namespace Hardware;

/**
 * DRIVING TEST AUTO:
 */
void driveTest() {

  while(!tank_drive.drive_forward(30, 13)) {vexDelay(20);} //test forward movement
  wait(20000, msec);
  while(!tank_drive.turn_degrees(90, 1)) {vexDelay(20);}  //test turning 
}


/**
 * skills sub rountings
 */

///////////////////////////////////////////////////////////////////////
/**                                                                 //
* These sub function condenses the amount of code needed to type    //
*/                                                                  //
//////////////////////////////////////////////////////////////////////

void move(float target, float speed, float waitTime){ //"speed" is in volts
  while(!tank_drive.drive_forward(target, speed)){vexDelay(20);} //move towards goal 
  tank_drive.stop();
  wait(waitTime, timeUnits::msec);
}

void turnTo(float target, float percent, float waitTime){
  while(!tank_drive.turn_degrees(target, 100)){vexDelay(20);} //move towards goal 
  tank_drive.stop();
  wait(waitTime, timeUnits::msec);
}


//////////////////////////////////////////////////////////////////////

/**
* Assumes the robot is on diagonal ready to score and turns onto 
* the horizontal after
*/

float offset = 6; //distance from the center of the robot to the center of the intakes

void scoreCornerTower(){ //and turn onto horizontal
  //move towards goal 
  move(32, 13, 100);

  //Score ball –– don't use intakes
  uptake(3,3, 100);  //make 4 revolutions at max speed
  
  //move back from goal 
  move(-21.48 + offset, 13, 100); 

  //turn to face horizontal line
  turnTo(121.2, 1.0, 100);
}

/**
*  Assumes the robot is on the horizontal line right after a corner goal
*/

void scoreSideTowerHorizontal() {
  //start intaking
  intake.spin(fwd, 13, volt);

  //move to ball and intake
  //move(55.256, 13, 100);
  move(50, 13, 100);
  intake.spin(fwd, 0, volt);

  //turn to the goal
  turnTo(-90, 1.0, 100);
  
  //score ball into goal
  move(18.355, 13, 100);
  uptake(2, 2, 600);

  //back away from goal 
  move(-18.355, 13, 100);

  //turn to diagonal ball
  turnTo(104, 1.0, 100);
}

/**
* Scores in corner goal directly across the corner goals the go onto horizontal. Robot starts 
* on the start of diagonal
*/ 
void scoreOppositeCorner(){
  //move along diagonal 
  intake.spin(fwd, 13, volt);
  move(49.477, 13, 100);
  intake.spin(fwd,0,volt);

  //turn to goal
  turnTo(107.1, 1.0, 100);

  //move to goal
  move(35.473, 13, 100);

  //shoot ball into goal
  uptake(2,2,600); 

  //backout
  move(-35.47, 13, 100);

  //turn to ball on diagonal
  turnTo(177.5, 1.0, 100);
}

/**
* Scores in side goal directly in between the corner and opposite corners
*/ 
void scoreOppositeSideGoal(){ //starts at the start of the diagonal to ball
  //move to intake ball
  intake.spin(fwd, 13, volt);
  move(43.267, 13, 100);
  intake.spin(fwd, 0, volt);

  //turn to goal
  turnTo(56.3, 1.0, 100);

  //move to goal and score
  move(42.355, 13, 100);
  uptake(2,2, 600);

  //back out goal and turn to horizontal to connect to corner goals
  move(-18.355, 13, 100);
  turnTo(90, 1.0, 100);

}


timer shootDelay = timer();

void shootIndex(){
  //int ballCount = 1;
  int counter = 0;
  //while(ballCount > 1){
  float prevTime = shootDelay.time(msec);

while(1){
    if(counter == 0){
      top_roller.spin(fwd, 13, volt); //get the top roller up to speed
      counter++;
      wait(100, msec);
    }
    bottom_roller.spin(fwd, 13, volt); //shoot ball

    if(indexer.objectDistance(mm) <= 20 || shootDelay.time() - prevTime > 2000){ //if the ball exits or timer passes
      wait(250, msec); //wait until the ball fully exits
      top_roller.spin(reverse, 13, volt);

      intake.rotateFor(fwd, .4, rev, 100, velocityUnits::pct); //bring in 2nd ball
      while(indexer.objectDistance(mm) > 200 || shootDelay.time() - prevTime > 4000) //wait till ball shoots out or after 4 secs
        wait(10,msec); //spin the uptake until the 2nd ball reaches index 
        break;
    }
    
    wait(20,msec);
  }
  bottom_roller.rotateFor(reverse, .1, rev, 300, velocityUnits::rpm, false);
  top_roller.rotateFor(reverse, 1, rev, 600, velocityUnits::rpm, false);
}

void index(){
  int ballCount = 0;
  while(ballCount < 1){
    bottom_roller.spin(fwd, 8, volt);
    top_roller.spin(fwd,-13,volt);
    if(indexer.objectDistance(mm) <= 100){
      ballCount++;
      bottom_roller.spin(fwd, -10, volt);
      wait(90, msec);
      break;
    }
    wait(20, msec);
  }
  bottom_roller.spin(fwd, 0, volt);
  top_roller.spin(fwd,0,volt);
}

void indexS(){
   top_roller.spin(reverse, 10, volt);
   //bool indexed = false;
   bottom_roller1.setBrake(brake);
   bottom_roller2.setBrake(brake);
   top_roller.setBrake(brake);
   //while(!indexed){
   
    //if there's a ball floating in the middle and in the intake
    if(indexer.objectDistance(mm) > 140 && lowerIndexer.objectDistance(mm) < 50 && intakeIndexer.objectDistance(mm) < 80){ 
      intake.rotateFor(reverse, .1, rev, 100,velocityUnits::pct, false); //get ball in away from intake
      //while(lowerIndexer.objectDistance(mm) > 80)
        //bottom_roller.spin(reverse, 13, volt); //drop ball down
        
      while(indexer.objectDistance(mm) > 150)
        bottom_roller.spin(fwd, 12, volt); //bring ball back up

       intake.rotateFor(fwd, .1, rev, 100,velocityUnits::pct, false); //get ball in away from intake
        //indexed = true;
       // break;
    }

    /*
    //if there's only one ball at the top
    if(indexer.objectDistance(mm) <= 65 && lowerIndexer.objectDistance(mm) > 20 && intakeIndexer.objectDistance(mm) > 80){ 
      while(lowerIndexer.objectDistance(mm) > 20)
        bottom_roller.spin(reverse, 10, volt); //drop ball down
      while(indexer.objectDistance(mm) > 130)
        bottom_roller.spin(fwd, 8, volt); //bring ball back up

        indexed = true;
        break;
    }
    
    //if there's one ball floating in the middle
    if(lowerIndexer.objectDistance(mm) < 20 && indexer.objectDistance(mm) > 100 && lowerIndexer.objectDistance(mm) > 90){
      while(indexer.objectDistance(mm) > 130)
        bottom_roller.spin(fwd, 8, volt); //bring ball back up

        indexed = true;
    }
    */

    //wait(20, msec);
   //}
  bottom_roller.spin(fwd, 0, volt);
  top_roller.spin(fwd,0,volt);
}
/**
 * Code for the autonomous period is executed below.
 */
void Auto::autonomous()
{
  intakeLeft.setBrake(brake);
  intakeRight.setBrake(brake);
  lf.setBrake(brake);
  lr.setBrake(brake);
  lr2.setBrake(brake);
  rf.setBrake(brake);
  rr.setBrake(brake);
  rr2.setBrake(brake);
  
  inertia.calibrate();
  while(inertia.isCalibrating()){}

    deploy();
  wait(1000, msec);

  //FLIPOUT
//score corner tower
  //intake first ball
  intake.rotateFor(directionType::fwd, .6, rotationUnits::rev, false);
  move(15, 13, 100);
  
  //turn to goal
  turnTo(-118, 1.0, 100);
  //move towards goal 
  move(20, 13, 0);
  move(4,7, 100);

  //Score ball –– don't use intakes
  shootIndex();

  //move back from goal 
  intake.spin(reverse, 13, volt);
  move(-13, 13, 100); 
  //turn to face horizontal line
  intake.stop();
  turnTo(121.5, .8, 100); 
  

//score side tower
  //move to ball and intake
  move(37, 13, 0);
  intake.rotateFor(directionType::fwd, .6, rotationUnits::rev, false);
  move(15, 6, 100);

  //turn to the goal
  turnTo(-90, 1.0, 100);
  
  //score ball into goal
  move(5, 13,0);
  move(3, 5, 100);
  shootIndex();
  wait(150, msec);
  //shoot();

  //back away from goal 
  intake.spin(reverse, 13, volt);
  move(-19.35 + offset, 13, 100);
  intake.stop();

  //turn to diagonal ball and intake
  turnTo(105, 1.0, 100);
  move(30, 13, 0);
  intake.rotateFor(directionType::fwd, 1, rotationUnits::rev, false); //intake ball
  move(19, 6, 100);
  move(-10, 13, 0); //back out to face goal


//score corner goal
  turnTo(-58, 1.0, 100);
  //move to goal
  move(21, 13, 0);
  move(6, 5, 100);
  shootIndex();

  //backout
  move(-34.47, 13, 100);
  

  //turn to ball on diagonal and intake
  turnTo(155, 1.0, 100);
  move(14, 13, 0);
  intake.rotateFor(directionType::fwd, 1.4, rotationUnits::rev, false);
  move(18, 7, 100);
  

//score side goal
  move(-10, 13, 0); //backout
  turnTo(-90, 1, 100); //face goal

  //score ball
  move(30, 13, 0);
  move(5, 5, 100);
  shootIndex();



  /*
  scoreCornerTower();
  scoreSideTowerHorizontal();
  scoreOppositeCorner();
  scoreOppositeSideGoal();


  //connect to mirrored part of route
  move(36, 13, 100);
  turnTo(-31.2, 1.0, 100);

  scoreCornerTower();
  scoreSideTowerHorizontal();
  scoreOppositeCorner();
  scoreOppositeSideGoal();
  */
  //descore center goal and shoot ball

  
  //Autonomous Loop
  // while (true)
  // {
  //   vexDelay(20); // Small delay to allow time-sensitive functions to work properly.
  // }
}