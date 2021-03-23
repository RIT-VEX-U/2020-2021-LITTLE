#include "hardware.h"
#include "competition/opcontrol.h"


extern thread stateThread; //allow global access to sensor states

//Function List:

//General Access / Auton
void index();
void deploy();
void shoot(int balls);
void stopAll();
void uptakeRevolution(int top, int bottom, int speed);
void uptake(int top, int middle, int bottom);

//Sensors
int getCurrentState();

//OP Control
void runIntake();





