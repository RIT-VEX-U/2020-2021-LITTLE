#include "competition/autonomous.h"
#include "subsystems.h"
#include <iostream>

using namespace Hardware;

thread autonSensors;

/**
 * DRIVING TEST AUTO:
 */
void driveTest() {

  while(!tank_drive.drive_forward(30, 13)) {vexDelay(20);} //test forward movement
  wait(20000, msec);
  while(!tank_drive.turn_degrees(90, 1)) {vexDelay(20);}  //test turning 
}

///////////////////////////////////////////////////////////////////////
///                                                                  //
// These sub function condenses the amount of code needed to type    //
//                                                                   //
///////////////////////////////////////////////////////////////////////

void move(float target, float speed, float waitTime){ //pct for speed
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

//SKILLS

/**
 * Code for the autonomous period is executed below.
 * Everything is in inches and degrees
 */
float descoringRotation = 1.5, descoringOffset = 2; //descoring offset is the distance from the intaking center to the center of a goal at the descoring position
float offset = 9.75; //distance of center of rotation to intaking center

void Auto::autonomous(){
  autonSensors = thread(getCurrentState);

  prevAngle = -55; 
  inertia.setRotation(-55, deg);//set starting position of robot

  //had to hard code lol
  intakeLeft.setBrake(brake);
  intakeRight.setBrake(brake);
  lf.setBrake(brake); 
  lr.setBrake(brake);
  lr2.setBrake(brake);
  rf.setBrake(brake);
  rr.setBrake(brake);
  rr2.setBrake(brake);

  deploy(); //score into side goal

//score corner tower//////////////////////////////////////////////////////////////
  //intake first balls in line
  intake.spin(fwd, 13, volt);
  uptake(0,0,2);
  move(25, 1, 0);
  index();
  
  turnTo(-61, 1, 0); //intake second ball slowly
  move(28.5, 1, 100); //subtract some distance to prevent robot from bottoming out on wall
  intake.stop();

  //back up to goal
  move(-31 + offset + 2.5, 1, 0); //add to offset to counter previous offset

  //turn to goal
  turnTo(-118.5, 1.0, 100);

  //move towards goal 
  move(35.8 - offset - descoringOffset, 1, 0); //subtract offset twice

  //Score balls and descore two from corner
  intake.rotateFor(fwd, 1, rev, 100, velocityUnits::pct, false); //descore while shooting
  shoot(1);
  intake.rotateFor(fwd, 1, rev, 100, velocityUnits::pct, false); //descore while shooting
  shoot(1);
 
  //backout and spit out blue balls slowly
  uptake(-13, -13, -7);
  intake.spin(reverse, 10, volt);
  move(-17 + descoringOffset, 13, 0);
  intake.stop();

  //turn to ball
  turnTo(3.5, 1, 0); 
  
//score side tower/////////////////////////////////////////////////////////////
  //move to ball and intake
  intake.spin(fwd, 13, volt);
  move(44 + offset, 1, 0);
  intake.stop();
  index();
 
  //turn to the goal and score
  turnTo(-90, 1.0, 100);
  move(8, 1, 0);
  intake.rotateFor(fwd, 1, rev, 100, velocityUnits::pct, false); //descore while shooting
  shoot(1);

  //back away from goal and spit out blue ball
  intake.spin(reverse, 8, volt);
  uptake(0,0,-13);
  move(-19.35, 13, 0);
  uptake(0,0,0);
  intake.stop();

  //turn to diagonal ball and intake
  turnTo(8, 1.0, 100);
  intake.spin(fwd, 13, volt); //intake ball
  move(45, 1, 0);
  index();
  intake.stop();
  move(-8, 13, 0); //back out to face goal


//score 2nd corner goal/////////////////////////////////////////////////////////////
  turnTo(-45, 1.0, 100);
  move(32, 1, 0); //move to goal
  //descore before shooting
  intake.rotateFor(fwd, 3.5, rev, 100, velocityUnits::pct); 
  shoot(1); //score

  //backout and spit out balls
  uptake(-13, -13, -7);
  intake.spin(reverse, 8, volt);
  move(-34.47, 13, 0);
  intake.stop();

  //turn to ball on diagonal and intake
  turnTo(154, .8, 100); //slow speed for long movements after turns
  move(14, 13, 0);
  intake.spin(fwd, 13, volt);
  move(18, 7, 200);
  intake.stop();
  

//score 2nd side goal/////////////////////////////////////////////////////////////
  move(-10, 13, 0); //backup to goal
  turnTo(-90, 1, 100); //face goal head on

  //score ball and descore tower
  move(30, 13, 0);
  move(5, 5, 100);
  intake.rotateFor(fwd, descoringRotation, rev); //blocking code
  shoot(1);

  //back out and spit ball out
  intake.spin(reverse, 7, volt);
  move(-15, 13, 0);
  intake.stop();

//score 3rd corner goal/////////////////////////////////////////////////////////////
  //turn to ball and intake
  turnTo(90, 1, 100);
  move(20, 13, 0);
  intake.spin(fwd, 13, volt);
  move(5, 13, 200);
  intake.stop();

  //turn to goal and descore 2 then score 1
  turnTo(-35, 1, 0);
  move(30, 13, 0);
  move(5, 8, 100);
  intake.rotateFor(fwd, descoringRotation*2, rev); //blocking
  shoot(1);

  //backout and spit out balls
  intake.spin(reverse, 8, volt);
  move(-20, 13, 0);

//score 3rd side goal/////////////////////////////////////////////////////////////
  //turn to second back ball and intake
  turnTo(135, 13, 150);
  move(25, 13, 0);
  intake.spin(fwd, 13, volt);
  move(5, 7, 200);
  intake.stop();
  index();

  //turn to goal and intake 2nd ball
  turnTo(-135, 1, 100);
  move(10, 13, 0);
  intake.spin(fwd, 13, volt);
  move(5, 7, 200);
  intake.stop();

  //descore one, shoot two 
  intake.rotateFor(fwd, descoringRotation, rev);
  shoot(2);

  //back out and spit out blue ball
  intake.spin(reverse, 7, volt);
  move(-15, 13, 0);
  intake.stop();


//score last corner goal/////////////////////////////////////////////////////////////
  //turn to ball and intake
  turnTo(110, 1, 100);
  move(30, 13, 0);
  intake.spin(fwd, 13, volt);
  move(5, 7, 100);
  intake.stop();
  index();

  //back up to goal and descore 2, shoot 1
  move(-10, 13, 0);
  turnTo(-100, 1, 100);
  move(15, 13, 0);
  move(5, 7, 100);
  intake.rotateFor(fwd, descoringRotation, rev);
  shoot(1);
  intake.rotateFor(fwd, descoringRotation, rev);

//center goal/////////////////////////////////////////////////////////////

  //spit out last balls and intake final ball
  uptake(-13, -8, -5);
  intake.spin(reverse, 5, volt);
  move(-25,13, 0);
  uptake(0,0,0);
  
  //turn to final ball and intake
  turnTo(-170, 1, 100);
  move(10, 13, 0);
  intake.spin(fwd, 13, volt);
  move(5, 7, 200);
  intake.stop();
  index();

  //turn to goal and to descore
  turnTo(-100,1, 100);
  move(10, 13, 0);
  //descore center
  intake.spin(reverse, 13, volt);
  move(5, 10, 0);
  move(-5, 10, 0);

  move(5, 10, 0);
  move(-5, 10, 0);

  move(5, 10, 0);
  move(-5, 10, 0);

  //shoot last ball
  move(5, 5, 100);
  shoot(1);

  
  //backout to clear goal
  move(-10, 13, 0);


  //Autonomous Loop
  // while (true)
  // {
  //   vexDelay(20); // Small delay to allow time-sensitive functions to work properly.
  // }
}