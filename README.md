# Autonomous RC Plane

A custom flight controller for a fixed-wing RC plane, built from scratch, both the PCB and the firmware. It's built around an ESP32-S3 and runs in-between a standard RC receiver and the servos/esc, giving the plane an onboard brain capable of taking over from the pilot and flying autonomously.

All software is written in ESP-IDF (specifically v5.5). It's a personal challenge to write as much of it as possible without plug-and-play libraries (ESP-IDF's own UART/I2C drivers are fair game), mainly the device drivers and the flight control logic.

## The Goal

The flight control system is a cascade of PID loops: GPS waypoint/heading navigation on the outside, commanding roll/pitch attitude loops on the inside, plus an airspeed loop on the throttle. Target milestones, roughly in order:

- [x] RC pass-through with working sensor drivers (IMU, GPS) and RC I/O
- [x] First flight (manual pass-through, October 3, 2026)
- [ ] Stable autonomous level flight
- [ ] Altitude hold
- [ ] GPS-guided heading / waypoint following (implemented and SITL-tested, not flown yet)
- [ ] Autonomous takeoff and landing

Longer term, the hopes are giving it mission-oriented capabilities like payload release or coordinated flight with a second drone.

## Current Status

The plane had its first flight on October 3, 2026, in manual pass-through mode only. The RC link had signal dropouts during that flight, so that's being debugged before anything autonomous is tried in the air. Since then the firmware has gained an on-board flight event log, a longer signal-loss window, and safer behaviour when the radio drops (see Safety below).

Autonomous mode (attitude stabilization, waypoint following, return-to-home) has only been tested on the bench and in the SITL simulator. The roll/pitch/heading gains come from SITL sweeps and still need real flight tuning.

## Hardware

- ESP32-S3
- ICM-20948 IMU (I2C), roll/pitch/yaw via a Madgwick filter, with its AK09916 magnetometer used for compass heading
- GPS module (UART, NMEA)
- MS4525DO differential-pressure airspeed sensor (I2C)
- Standard PWM RC receiver and servos/ESC, running alongside the ESP32 instead of being replaced by it

Future hardware will hopefully include lidar and a barometer.

### PCB

The firmware currently runs on the **v2.1** board: a custom carrier PCB that an ESP32-S3-DevKitC plugs into. Everything plugs into labeled connectors on the board rather than bare GPIO pins:

- 6 RC receiver inputs (PWM), 6 labeled servo outputs, and primary/secondary ESC signal outputs
- Separate UBEC power inputs for the flight controller and the ESC/servo rail, each with P-FET reverse-polarity protection
- JST-SH (STEMMA QT) connectors for the IMU, the barometer, and a spare QT port for future I2C sensors
- A JST-GH connector for the airspeed sensor, a header for the GPS UART, and connectors for a LIDAR
- An accessory I/O header for things like LEDs, a payload-release servo/relay, etc.
    -   (obviously will require alterations to code, but software is easier to change than hardware is...)

The GPIO-to-connector mapping the firmware uses (CH1_OUT_GPIO etc. in main.c) is defined by the board's pinout, not repeated here.

The KiCad sources in [`kicad/`](kicad) are the **next** revisions, which the firmware hasn't been ported to yet:

- [`kicad/flight-controller/`](kicad/flight-controller): flight controller v3.1, a fully integrated 4-layer board (ESP32-S3 module, ICM-42688-P + BMI088 IMUs, DPS368 barometer, microSD, SBUS/CRSF receiver input, USB-C). Pinout and power details are in [`docs/buses.md`](docs/buses.md), and its design review is in [`kicad/flight-controller/design-review-fixed-wing-drone.md`](kicad/flight-controller/design-review-fixed-wing-drone.md).
- [`kicad/mission-board/`](kicad/mission-board): a second ESP32-S3 payload board (camera, motor drivers, switched outputs, servo driver) that talks to the flight controller over UART.
- [`kicad/uwb-radio-kicad/`](kicad/uwb-radio-kicad): a DWM3000 ultra-wideband radio module for ranging between drones.

## Software Architecture

- **RC input:** 6 channels read via the ESP32-S3's MCPWM capture timers. Interrupt-driven edge timestamping converts rising/falling edges into pulse widths directly, no external library.
- **RC/servo output:** 6 servo channels plus 2 dedicated ESC channels, driven via MCPWM comparators on a 20ms timebase. Input/output channel mapping, per-channel PWM range and reversal are all configurable at runtime.
- **IMU:** ICM-20948 and its AK09916 magnetometer driven with hand-written I2C register reads/writes, fused through a ported Madgwick AHRS filter for roll/pitch/yaw, with a stillness-gated gyro bias calibration at boot.
- **GPS:** UART driver with a hand-written NMEA RMC sentence parser, plus u-blox config commands to quiet down unused sentence types and set a 5Hz fix rate.
- **Airspeed:** MS4525DO driver with a boot-time zero-offset calibration and a low-pass filter on the pressure reading.
- **PID:** a small PID module with integral anti-windup, used for roll, pitch, heading and airspeed.
- **Navigation:** a 10Hz task that steers toward GPS waypoints (or holds the heading the plane had when autonomous engaged) by commanding a bank angle, using the compass heading or, when moving, GPS course over ground.
- **Control loop:** a 100Hz task that reads all RC channels every cycle and switches between manual pass-through and autonomous output based on the mode switch (below 1500us = autonomous), RC signal health and sensor availability.

### Safety

- Autonomous mode is only armed after the mode switch has been seen in manual at least once since boot, and it needs a fresh GPS fix to engage.
- If the RC signal is lost for 1 second, the plane returns to its home point (averaged from GPS at power-on), then spirals down with the motor off. With no GPS fix or no home, it spirals down where it is. If autonomous isn't available at all, it holds the surfaces at trim with the motor off instead of replaying the last stick input.
- After a reboot in the air, manual pass-through comes back first, before anything slow in boot.
- A small flight event log in RTC memory records RC dropouts, signal losses, failsafe transitions and the reset reason of every boot. It survives a reset (not a power cycle) and can be read from setup mode after landing.

### SITL

[`sitl/`](sitl) holds host-side simulators that compile the firmware's own `pid.c` and `gps_math.c` against a simple airframe model, used to tune the attitude and heading gains, with an 80-scenario regression suite. See [`sitl/README.md`](sitl/README.md).

## Build / Flash

Set up ESP-IDF v5.5, then from the project root:

```sh
idf.py build
idf.py flash monitor
```

### Setup mode

Holding the board's BOOT button within 5 seconds of power-on boots into a
setup mode instead of normal flight init: it hosts its own WiFi access point
(`plane-fc-setup` by default) with a local web page for calibrating servo/ESC
PWM ranges and direction, remapping RC/servo channels, capturing trim,
planning the GPS mission, tuning PID gains, setting the airframe mixing mode
and airspeed-hold target, running bench control tests, and reading the flight
event log. Connect to the AP
and browse to `http://192.168.4.1/`. Everything is saved to NVS immediately;
a normal reset (BOOT not held) returns to flight mode with those settings
applied.

## Author

Travis Brown