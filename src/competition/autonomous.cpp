#include "competition/autonomous.h"

/**
 * Code for the autonomous period is executed below.
 */
void Auto::autonomous()
{
  //Autonomous Init

  // Defines how the robot will move. Default values are a good starting point.
  SplinePath::motion_profile_t profile;
  
  SplinePath path(Hardware::drive_system, Hardware::inertia, Hardware::left_front, Hardware::right_front, profile);

  // Create a list of waypoints:
  // {x, y, rotation}
  // {x (forward is positive), y (right is positive), rotation (clockwise is postive)}
  // Units are {inches, inches, radians}
  Waypoint point_list[3] = {
    {0, 0, d2r(0)},
    {12, 12, d2r(90)},
    {24, 24, d2r(0)}
  };

  //Autonomous Loop
  while (true)
  {
    // "run_path" is non-blocking: other code can be run while it is moving.
    // It returns true when the path is complete.
    if(path.run_path(point_list, 3))
      return;

    vexDelay(20); // Small delay to allow time-sensitive functions to work properly.
  }
}