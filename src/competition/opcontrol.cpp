#include "competition/opcontrol.h"
#include "subsystems.h"
#include <iostream>
using namespace Hardware;

thread driveThread;

// -- CHASSIS --
/**
* Control chassis in seperate thread
*/
int move(){
  while(1){
   tank_drive.drive_tank(master.Axis3.position() / 100.0, master.Axis2.position() / 100.0); //control chassis
   this_thread::sleep_for(20);
  }
  return 0;
}

/**
 * Code for the Driver Control period is executed below.
 */
 
 void OpControl::opcontrol(){
   //allow tasks to run in background
   driveThread = thread(move); //allow chassis to move independent of everything
   stateThread = thread(getCurrentState); //update sensors and state functions  
 
   while(1){
     //allow other functions to run

    //run intaking and shooting on the same thread –– one function automatically takes priority
     runIntake(); //determines when to intake/outtake

     if(master.ButtonL1.pressing()){
       shoot(1); //always shoot only one
     }

     vexDelay(10);
   }
 }