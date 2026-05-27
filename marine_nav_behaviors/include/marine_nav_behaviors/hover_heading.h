#ifndef MARINE_NAV_BEHAVIORS_HOVER_HEADING_H
#define MARINE_NAV_BEHAVIORS_HOVER_HEADING_H

#include <cmath>

namespace marine_nav_behaviors
{

// Outcome of the hover approach-heading choice: the heading error to steer
// toward, and the sign to apply to the forward speed (+1 forward, -1 reverse).
struct ApproachHeading
{
  double steering_angle;
  double drive_sign;
};

// Decide how to approach the hold point given the forward heading error to it
// (radians, normalized to [-pi, pi]) and the heading mode.
//
// point_at_target == true (the virtual-anchor default): always face the target,
// i.e. steer to null the forward error and drive forward (drive_sign = +1).
//
// point_at_target == false (ArduPilot LOIT_TYPE=0): approach forward or in
// reverse, whichever needs less rotation. If pointing 180 deg away is the
// smaller turn, steer to that reversed heading and drive in reverse
// (drive_sign = -1) — spending less yaw thrust to hold station. Ties favor
// forward. Kept free-standing and dependency-free so it is unit-testable
// without a behavior/ROS fixture.
inline ApproachHeading chooseApproachHeading(double forward_error, bool point_at_target)
{
  if (point_at_target) {
    return {forward_error, 1.0};
  }

  double reverse_error = forward_error + M_PI;
  if (reverse_error > M_PI) {
    reverse_error -= 2.0 * M_PI;
  }

  if (std::abs(reverse_error) < std::abs(forward_error)) {
    return {reverse_error, -1.0};
  }
  return {forward_error, 1.0};
}

}  // namespace marine_nav_behaviors

#endif  // MARINE_NAV_BEHAVIORS_HOVER_HEADING_H
