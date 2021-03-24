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

//SKILLS

/**
 * Code for the autonomous period is executed below.
 */
float descoringRotation = 2;

void Auto::autonomous(){
  autonSensors = thread(getCurrentState);

  turnTo(90, 0.5, 20);
  wait(500000, msec);

  //had to hard code lol
  intakeLeft.setBrake(brake);
  intakeRight.setBrake(brake);
  lf.setBrake(brake); 
  lr.setBrake(brake);
  lr2.setBrake(brake);
  rf.setBrake(brake);
  rr.setBrake(brake);
  rr2.setBrake(brake);
  
  //inertia.calibrate();
  //while(inertia.isCalibrating()){wait(5,msec);}

  deploy(); //score into side goal

//score corner tower//////////////////////////////////////////////////////////////
  //intake two balls in line
  move(25, 10, 0); //go slow in on continuous motion until second ball
  intake.spin(fwd, 13, volt);
  index();
  move(5, 7, 100);
  intake.stop();

  //back up to goal
  move(-10, 13, 0);

  //turn to goal
  turnTo(-20, 1.0, 100);

  //move towards goal 
  move(20, 13, 0);
  move(4,7, 100);

  //Score balls and descore two from corner
  intake.rotateFor(fwd, descoringRotation*2, rev, false); //descore while shooting
  shoot(2);

  //backout and spit out blue balls slowly
  move(-13, 13, 0);
  uptake(-13, -7, -5);
  intake.spin(reverse, 6, volt);
  index();
  intake.stop();

  //turn to face back ball
  turnTo(135, .8, 100); 
  

//score side tower/////////////////////////////////////////////////////////////
  //move to ball and intake
  move(42, 13, 0);
  //intake.rotateFor(fwd, .6, rev, false);
  intake.spin(fwd, 13,volt);
  move(15, 6, 100);
  intake.stop();
  index();

  //turn to the goal and intake 2nd ball
  turnTo(-90, 1.0, 100);
  move(5, 13, 0);
  intake.spin(fwd, 13, volt);
  move(10, 13, 0);
  intake.stop();
  
  //score ball into goal and descore bottom ball
  move(5, 13,0);
  move(3, 5, 100);
  shoot(1);
  intake.rotateFor(fwd, descoringRotation,rev);
  shoot(1);

  //back away from goal and spit out ball
  intake.spin(reverse, 7, volt);
  uptake(0,0,-5);
  move(-19.35, 13, 0);
  intake.stop();

  //turn to diagonal ball and intake
  turnTo(105, 1.0, 100);
  move(30, 13, 0);
  intake.spin(fwd, 13, volt); //intake ball
  move(19, 6, 200); //allow some time for ball to get intaked fully –– last param
  index();
  intake.stop();
  move(-10, 13, 0); //back out to face goal


//score 2nd corner goal/////////////////////////////////////////////////////////////
  turnTo(-58, 1.0, 100);
  move(21, 13, 0); //move to goal
  move(6, 5, 100);
  //descore before shooting
  intake.rotateFor(fwd, descoringRotation*2, rev); 
  shoot(1); //score

  //backout and spit out balls
  intake.spin(reverse, 13, volt);
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