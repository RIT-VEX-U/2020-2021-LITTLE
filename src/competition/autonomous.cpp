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

void move(float target, float volt, float waitTime){ //pct for speed
  while(!tank_drive.drive_forward(target, volt)){vexDelay(20);} //move towards goal 
  tank_drive.stop();
  wait(waitTime, timeUnits::msec);
}

void turnTo(float target, float volt, float waitTime){
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
  move(25, 13, 0);
  index();
  
  turnTo(-61, 13, 0); //intake second ball slowly
  move(26.2, 13, 100); //subtract some distance to prevent robot from bottoming out on wall
  

  //back up to goal
  move(-32.1 + offset + 2.5, 13, 0); //add to offset to counter previous offset
  intake.stop();

  //turn to goal
  turnTo(-120.9, 11, 100);

  //move towards goal 
  move(36.5 - offset - descoringOffset, 10, 0); //subtract offset twice

  //Score balls and descore two from corner
  intake.rotateFor(fwd, 1.2, rev, 100, velocityUnits::pct, false); //descore while shooting
  shoot(1);
  intake.rotateFor(fwd, 1.5, rev, 100, velocityUnits::pct, false); //descore while shooting
  shoot(1);
 
  //backout and spit out blue balls slowly
  uptake(-13, -13, -7);
  intake.spin(reverse, 10, volt);
  wait(300, msec);
  move(-24 + descoringOffset, 13, 0);
  intake.stop();

  //turn to ball
  turnTo(2.3, 13, 0); 
  
//score side tower/////////////////////////////////////////////////////////////
  //move to ball and intake
  intake.spin(fwd, 13, volt);
  move(43 + offset, 13, 0);
  intake.stop();
  index();
  move(-7.5, 13, 0);
 
  //turn to the goal and score
  turnTo(-83, 13, 0);
  move(8.6, 13, 0);
  intake.rotateFor(fwd, 1.5, rev, 100, velocityUnits::pct, false); //descore while shooting
  shoot(1);

  //back away from goal and spit out blue ball
  intake.spin(reverse, 8, volt);
  uptake(0,0,-13);
  move(-20.5, 13, 0);
  uptake(0,0,0);
  intake.stop();

  //turn to diagonal ball and intake
  turnTo(12, 13, 0);
  intake.spin(fwd, 13, volt); //intake ball
  move(45, 13, 0);
  index();
  intake.stop();

  move(-4, 13, 0); //back out to face goal


//score 2nd corner goal/////////////////////////////////////////////////////////////
  turnTo(-43, 13, 0);
  move(29.2, 11, 0); //move to goal
  //descore before shooting
  intake.rotateFor(fwd, 3.2, rev, 100, velocityUnits::pct, false); 
  shoot(1); //score

  //backout and spit out balls
  uptake(-13, -13, -7);
  intake.spin(reverse, 8, volt);
  move(-34, 13, 0);
  intake.stop();

  //turn to ball on diagonal and intake
  turnTo(119, 13, 0); //slow speed for long movements after turns
  intake.spin(fwd, 13, volt);
  move(45, 10, 0);
  intake.stop();
  index();
  move(-5, 13, 0); //back up to goal
  

//score 2nd side goal/////////////////////////////////////////////////////////////
  turnTo(-3, 13, 0); //face goal head on

  //score ball and descore tower
  move(37.5, 13, 0);
  intake.rotateFor(fwd, 1.2, rev, 100, velocityUnits::pct); //blocking code
  shoot(1);

  //back out and spit ball out
  intake.spin(reverse, 7, volt);
  uptake(-13, -13, -8);
  move(-15, 13, 0);
  intake.stop();

//score 3rd corner goal/////////////////////////////////////////////////////////////
  //turn to ball and intake
  turnTo(93.5, 13, 0);
  intake.spin(fwd, 13, volt);
  move(48, 13, 0);
  index();
  intake.stop();

  //turn to goal and descore 2 then score 1
  turnTo(43, 13, 0);
  move(12.5, 11, 0);
  intake.rotateFor(fwd, 3.2, rev, false); //blocking
  shoot(1);

  //backout and spit out balls
  uptake(-13, -13, -7);
  intake.spin(reverse, 8, volt);
  move(-28, 13, 0);

//score 3rd side goal/////////////////////////////////////////////////////////////
  //turn to ball and intake
  turnTo(180, 13, 0);
  intake.spin(fwd, 13, volt);
  move(42.5, 13, 0);
  intake.stop();
  index();
  move(-3.5, 13, 0);

  //turn to goal score
  turnTo(88.5, 13, 0);
  move(8, 13, 0);
  intake.rotateFor(fwd, 1.5, rev, 100, velocityUnits::pct, false); 
  shoot(1);

  //back out and spit out blue ball
  uptake(-13, -8, -8);
  intake.spin(reverse, 8, volt);
  move(-15, 13, 0);
  intake.stop();
  uptake(0,0,0);


//score last corner goal/////////////////////////////////////////////////////////////
  //turn to ball and intake
  turnTo(198, 13, 0);
  intake.spin(fwd, 13, volt);
  move(46, 13, 0);
  intake.stop();
  index();
  move(-5, 13, 0);

  //turn to goal and score
  turnTo(135, 13, 0);
  move(13, 11, 0);
  intake.rotateFor(fwd, 3.2, rev, 100, velocityUnits::pct, false);
  shoot(1);

//center goal/////////////////////////////////////////////////////////////

  //spit out last balls and intake final ball
  uptake(-13, -8, -5);
  intake.spin(reverse, 5, volt);
  move(-25,13, 0);
  uptake(0,0,0);
  
  //turn to final ball and intake
  turnTo(305, 1, 100);
  intake.spin(fwd, 13, volt);
  move(34, 13, 0);
  intake.stop();
  index();

  //turn to goal and to descore
  turnTo(0, 13, 0);
  move(22, 8, 0);

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