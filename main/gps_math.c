#include <math.h>
#include "gps_math.h"

#define EARTH_RADIUS (6371000.0) // in meters

double degrees_to_rads(double degrees) {
    return degrees * (M_PI / 180.0);
}

double rads_to_degrees(double rads) {
    return rads * 180.0 / M_PI;
}

float heading_to_target(double cur_lat, double cur_long, double goal_lat, double goal_long)
{
    double lat1 = degrees_to_rads(cur_lat);
    double lat2 = degrees_to_rads(goal_lat);
    double d_long = degrees_to_rads((goal_long - cur_long));

    double y = sin(d_long) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(d_long);
    double heading_rad = atan2(y, x);
    float heading_deg = fmod((rads_to_degrees(heading_rad) + 360.0), 360.0);

    return heading_deg;
}

double distance_to_target(double cur_lat, double cur_long, double goal_lat, double goal_long)
{
    double lat1 = degrees_to_rads(cur_lat);
    double lat2 = degrees_to_rads(goal_lat);
    double d_lat = lat2 - lat1;
    double d_long = degrees_to_rads((goal_long - cur_long));

    double x = d_long * cos(((lat1 + lat2) / 2));
    double y = d_lat;
    double dist = sqrt((x*x) + (y*y)) * EARTH_RADIUS;

    return dist;
}

float heading_error_deg(float current_deg, float desired_deg)
{
    float err = fmodf(desired_deg - current_deg + 540.0f, 360.0f) - 180.0f;
    return err;
}
