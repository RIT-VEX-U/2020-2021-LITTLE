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
  while(!tank_drive.turn_degrees(target, percent)){vexDelay(20);} //move towards goal 
  tank_drive.stop();
  wait(waitTime, timeUnits::msec);
}

//controls how many revolutions each roller makes and at what speed [-600,600] rpm
//BLOCKING Code
void uptake(int top, int bottom, int speed){
  bottom_roller.rotateFor(bottom, rev, speed, velocityUnits::rpm); //make 2 revolutions at max speed
  top_roller.rotateFor(top, rev, speed, velocityUnits::rpm, true); 
}

void stopIntaking(){
  intake.stop();
  bottom_roller.stop();
  top_roller.stop();
}

void intake_ball(float percent){
  intake.spin(directionType::fwd, percent, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, percent, velocityUnits::pct);
  top_roller.spin(directionType::rev, percent, velocityUnits::pct);
}


//////////////////////////////////////////////////////////////////////

/**
* This sub function carries out the deployment and won't allow the run to continue
* unless the robot has deployed
*/

void deploy(){

}

/**
* Assumes the robot is on diagonal ready to score and turns onto 
* the horizontal after
*/

void scoreCornerTower(){ //and turn onto horizontal
  //move towards goal 
  move(35.47, 13, 100);

  //Score ball –– don't use intakes
  uptake(2,2, 600);  //make 2 revolutions at max speed
  
  //move back from goal 
  move(-21.48, 13, 100); 

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
  move(55.256, 13, 100);
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


/**
 * Code for the autonomous period is executed below.
 */
void Auto::autonomous()
{
  inertia.calibrate();
  while(inertia.isCalibrating()) {
  }
  //FLIPOUT

  //intake first ball
  intake.spin(directionType::fwd, 13, volt);
  move(9, 13, 100);
  intake.stop();
  
  //turn to goal
  turnTo(-121.2,1.0, 100);

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

  //descore center goal and shoot ball

  
  //Autonomous Loop
  // while (true)
  // {
  //   vexDelay(20); // Small delay to allow time-sensitive functions to work properly.
  // }
}