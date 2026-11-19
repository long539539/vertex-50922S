#include "vex.h"
#include "rcl-config.h"

// RCL Tracking setup. See include/RclTracking.h for what each part does.
// All positions are real field coordinates in inches: field center (0, 0), walls at +-70.25.

// Distance sensors. Put in the real ports. PORT0 = not plugged in, RCL skips it.
vex::distance rear_dist(PORT0);
vex::distance left_dist(PORT0);
vex::distance right_dist(PORT0);

// RclSensor(&sensor, x offset, y offset, facing, angle tolerance)
//   x offset:  inches right of the tracking center (left is negative)
//   y offset:  inches in front of the tracking center (behind is negative)
//   facing:    0 = front, 90 = right, 180 = back, 270 = left
//   angle tolerance: only use the sensor while its beam is within this many degrees of square to a wall
// These copy the distance sensors in robot-config.cpp. Check the rear one: its y offset is +6, which means
// 6 in IN FRONT of center. A sensor on the back is usually behind center (negative).
RclSensor rear_rcl(&rear_dist, 4.0, 6.0, 180.0, 15.0);
RclSensor left_rcl(&left_dist, -6.0, 4.0, 270.0, 15.0);
RclSensor right_rcl(&right_dist, 6.0, 0.0, 90.0, 15.0);

// RclTracking(&chassis, updates per second, auto sync, min delta, max delta, max delta from odom,
//             max sync speed in/s, min pause ms, max range in, smoothing samples)
// The last two aren't in the original RCL. With only a rear and a left sensor, each axis is seen by one sensor
// at a time, so 50 in max range + a median of 5 keeps long, noisy readings from jerking odometry around.
// Once you have sensors on opposite sides (front + rear, left + right), set both to 0: that was most accurate in testing.
RclTracking RclMain(&chassis, 25, true, 0.5, 4.0, 10.0, 3.0, 20, 50.0, 5);

// Obstacles: sensor beams that pass through these are ignored. Add your game's field elements that sit
// between the robot and a wall, in field coordinates.
//   Circle_Obstacle goal(x, y, radius);
//   Line_Obstacle structure(x1, y1, x2, y2);
//   Line_Obstacle::addPolygonObstacle({{x1, y1}, {x2, y2}, {x3, y3}});
//
// Optional: ignore everything across the field's center line, like the original RCL example does in autonomous.
// Line_Obstacle disableLine(0, FIELD_NEG_HALF_LENGTH, 0, FIELD_HALF_LENGTH);
