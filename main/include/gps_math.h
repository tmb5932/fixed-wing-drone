#ifndef GPS_MATH_H
#define GPS_MATH_H

// Pure lat/lon and bearing math, split out of gps.c so it has zero
// FreeRTOS/ESP-IDF dependencies and can be linked directly into the host-side
// SITL (sitl/) the same way pid.c is -- the exact code that flies, not a
// reimplementation.

double degrees_to_rads(double degrees);
double rads_to_degrees(double rads);

/**
 * Calculates heading from current location to target location in degrees.
 * math gotten from here: https://www.movable-type.co.uk/scripts/latlong.html
 *
 * Returns heading in degrees from 0 to 360, where 0 is north, 90 is east, etc.
*/
float heading_to_target(double cur_lat, double cur_long, double goal_lat, double goal_long);

/**
 * Calculates distance to target in meters.
 * Using Equirectangular approximation, as the distances for this use case will be in the hundreds of meters, not miles.
 */
double distance_to_target(double cur_lat, double cur_long, double goal_lat, double goal_long);

/**
 * Shortest-turn signed error from current to desired heading, in [-180, 180].
 * Positive means "turn right (clockwise)" to reach desired.
 */
float heading_error_deg(float current_deg, float desired_deg);

#endif // GPS_MATH_H
