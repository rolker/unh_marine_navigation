#ifndef MARINE_NAV_BEHAVIORS_HOVER_SPEED_H
#define MARINE_NAV_BEHAVIORS_HOVER_SPEED_H

#include <algorithm>

namespace marine_nav_behaviors
{

// Forward-speed *magnitude* command for the hover controller, as a function of
// range to the hold point and heading alignment. Mirrors the field-tuned "v4"
// logic in Hover::onCycleUpdate. The caller multiplies the result by the
// approach drive sign (see chooseApproachHeading): a reverse approach negates
// the whole command — including the small inner-deadband "back off" term, which
// must keep pushing the boat *away* from the point regardless of whether it is
// approaching bow- or stern-first. Kept free-standing and dependency-free so it
// is unit-testable without a behavior/ROS fixture.
//
// Return value (before drive_sign), by range band:
//   range >= maximum_radius          -> +maximum_speed
//   minimum_radius < range < max     -> linear ramp toward +maximum_speed
//   range < minimum_radius/2          -> small *negative* speed (back off, <=10%)
//   minimum_radius/2 <= range <= min -> 0
// then: a heading-error taper (zero past ~45 deg error), a range-aware turn
// floor (only at/outside minimum_radius), and a minimum_speed floor (only
// outside minimum_radius).
inline double computeHoverSpeed(
  double current_range,
  double steering_proportion,
  double minimum_radius,
  double maximum_radius,
  double minimum_speed,
  double maximum_speed)
{
  double current_target_speed = 0.0;

  if (current_range >= maximum_radius) {
    current_target_speed = maximum_speed;
  } else if (current_range > minimum_radius) {
    float p = (current_range - minimum_radius) / (maximum_radius - minimum_radius);
    current_target_speed = p * maximum_speed;
  } else {
    if (current_range < minimum_radius / 2.0) {
      float p = 0.1 * (1.0 - (current_range / minimum_radius));
      current_target_speed = -p * maximum_speed;  // apply some reverse, up to 10%
    } else {
      current_target_speed = 0.0;
    }
  }

  // Smooth taper: 1.0 at aligned, 0.0 at >= 45 deg heading error.
  current_target_speed *= std::max(0.0, 1.0 - steering_proportion * 4.0);

  // Range-aware floor for vectored-thrust authority during heading correction:
  // tapers from 0.5 * maximum_speed at maximum_radius down to 0.2 * at
  // minimum_radius, then drops to zero inside minimum_radius so the boat can
  // settle instead of orbiting at the kinematic radius v_min/yaw_rate (v4 of
  // the field patch; v3's flat 0.2 floor produced a ~3 m stable orbital loop).
  if (steering_proportion > 0.1 && current_range >= minimum_radius) {
    double range_factor = std::clamp(
      (current_range - minimum_radius) / (maximum_radius - minimum_radius), 0.0, 1.0);
    double turn_min_speed = (0.2 + 0.3 * range_factor) * maximum_speed;
    current_target_speed = std::max(current_target_speed, turn_min_speed);
  }

  if (current_range > minimum_radius) {
    current_target_speed = std::max(current_target_speed, minimum_speed);
  }

  return current_target_speed;
}

}  // namespace marine_nav_behaviors

#endif  // MARINE_NAV_BEHAVIORS_HOVER_SPEED_H
