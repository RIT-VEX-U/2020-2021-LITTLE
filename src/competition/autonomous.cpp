#include "competition/autonomous.h"

using namespace Hardware;

// -- HELPER FUNCTIONS --

void intake_ball() {
  intake.spin(directionType::fwd, 100, velocityUnits::pct);
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  front_rollers.spin(directionType::fwd, 100, velocityUnits::pct);
  bottom_roller.spin(directionType::fwd, 100, velocityUnits::pct);
  top_roller.spin(directionType::fwd, 100, velocityUnits::pct);

  while(!limit_switch.pressing()) {
    lf.spin(directionType::fwd, 50, velocityUnits::pct);
    rf.spin(directionType::fwd, 50, velocityUnits::pct);
    lr.spin(directionType::fwd, 50, velocityUnits::pct);
    rr.spin(directionType::fwd, 50, velocityUnits::pct);
  }

  lf.stop();
  rf.stop();
  lr.stop();
  rr.stop();

  intake.stop();
  front_rollers.stop();
  front_rollers.stop();
  bottom_roller.stop();
  top_roller.stop();
}

/**
 * Code for the autonomous period is executed below.
 */
void Auto::autonomous()
{
  //Autonomous Init

  //Autonomous Loop
  while (true)
  {

    vexDelay(20); // Small delay to allow time-sensitive functions to work properly.
  }
}