#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H

// Independent per-sensor toggles between simulated (fake) data and real
// hardware I/O -- lets the full system (nav waypoint/heading logic, the
// roll/pitch/heading/airspeed PID loops, etc.) be exercised end-to-end
// without every physical sensor wired up and calibrated.
//
// Comment one out to switch that sensor back to its real driver. Nothing
// outside imu.c/gps.c/airspeed.c needs to change either way: the fake data
// is written into the exact same shared structs (imu_data_t, gps_data_t,
// airspeed_g) via the same mutex + volatile-bool-ready-flag pattern the
// real drivers already use, so every downstream consumer (nav.c, main.c's
// control_task) is none the wiser.
// #define IMU_SIMULATED
// #define GPS_SIMULATED
// #define AIRSPEED_SIMULATED

// --- Fake IMU (imu.c) ---
// Roll/pitch held level; yaw sweeps continuously so nav's heading-error/PID
// logic has something real to react to during a simulated bench run.
#define IMU_SIM_YAW_RATE_DPS (3.0f)

// --- Fake GPS (gps.c) ---
// A simple circular "orbit" around a fixed home point, alternating with a
// stationary "parked" phase (zero groundspeed) -- the orbit phase exercises
// nav's waypoint-acceptance/heading-to-target math.
// Set GPS_SIM_HOME_LAT_DEG/_LON_DEG to somewhere near your actual mission
// waypoints if you want the orbit to plausibly pass near/through them.
#define GPS_SIM_HOME_LAT_DEG       (47.6062)
#define GPS_SIM_HOME_LON_DEG       (-122.3321)
#define GPS_SIM_ORBIT_RADIUS_M     (75.0)
#define GPS_SIM_ORBIT_PERIOD_S     (60.0)
#define GPS_SIM_MOVING_DURATION_S  (45)
#define GPS_SIM_PARKED_DURATION_S  (45)

// --- Fake airspeed (airspeed.c) ---
// Fixed below AIRSPEED_TARGET_CMS (main.c) so the airspeed-hold PID has a
// real, visible error to correct during a simulated run.
#define AIRSPEED_SIM_CMS (800)

#endif // SIM_CONFIG_H
