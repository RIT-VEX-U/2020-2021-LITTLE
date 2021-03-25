#include "competition/opcontrol.h"
#include "subsystems.h"
#include <iostream>
using namespace Hardware;

thread driveThread;
thread sensorThread;

// -- CHASSIS --
/**
* Control chassis in seperate thread
*/
int move(){
  while(1){
    double outputL = pow(master.Axis3.position(),3)/pow(127,2);
    double outputR = pow(master.Axis2.position(),3)/pow(127,2);

    int lSign = outputL/fabs(outputL);
    int rSign = outputR/fabs(outputR);

    if(lSign != rSign){ //if turning
      if(fabs(outputL) > 8)
        outputL = 8*lSign;
      if(fabs(outputR) > 8)
        outputR = 8*rSign;
    }
  
   tank_drive.drive_volt(outputL, outputR); //control chassis
   this_thread::sleep_for(20);
  }
  return 0;
}

/**
 * Code for the Driver Control period is executed below.
 */
 
 void OpControl::opcontrol(){
   deploy();

   //allow tasks to run in background
   driveThread = thread(move); //allow chassis to move independent of everything
   sensorThread = thread(getCurrentState); //update sensors and state functions 

   while(1){
    //run intaking and shooting on the same thread –– one function automatically takes priority
     runIntake(); //determines when to intake/outtake

     if(master.ButtonL1.pressing()){
       shoot(1); //always shoot only one
     }

     vexDelay(10);
   }
 }