# Autonomous RC Plane

A custom flight controller for a fixed-wing RC plane, built from scratch, both the PCB and the firmware. It's built around an ESP32-S3 and runs alongside a standard RC receiver and servos, giving the plane an onboard brain capable of taking over from the pilot and flying autonomously.

All software is written in ESP-IDF (targeting v5.5). It's a personal challenge to write as much of it as possible without plug-and-play libraries, mainly the device drivers and the flight control logic.

## The Goal

The flight control system is built around PID loops for roll and pitch, with GPS-guided heading planned next. Target milestones, roughly in order:

- [x] RC pass-through with working sensor drivers (IMU, GPS) and RC I/O
- [ ] Stable autonomous level flight
- [ ] Altitude hold
- [ ] GPS-guided heading / waypoint following
- [ ] Autonomous takeoff and landing

Longer term, the hopes are giving it mission-oriented capabilities like payload release or coordinated flight with a second drone.

## Current Status

Bench/ground testing only. The RC input/output path, IMU, and GPS drivers all work, and the PID loops run against live IMU data toward a level-flight setpoint, but it hasn't flown yet, as the actual plane is still being built. The RC controller passthrough and autonomous takeover has been tested and is working. Gains are placeholders, still being checked on the bench before anything gets tested in flight.

## Hardware

- ESP32-S3
- ICM-20948 IMU (over I2C), roll/pitch/yaw via a Madgwick filter. Magnetometer is wired up but not used for yaw yet.
- GPS module (UART, NMEA/UBX)
- Standard RC receiver and servos/ESC, running alongside the ESP32 instead of being replaced by it

### GPIO

**RC input (capture)**

| Channel | Function | GPIO |
|---|---|---|
| CH1 | Aileron | 21 |
| CH2 | Elevator | 47 |
| CH3 | Throttle | 48 |
| CH4 | Dial/Aux | 35 |
| CH5 | Rudder | 36 |
| CH6 | Mode switch | 37 |

**Servo/ESC output**

| Channel | GPIO |
|---|---|
| CH1 | 9 |
| CH2 | 10 |
| CH3 | 11 |
| CH4 | 12 |
| CH5 | 13 |
| CH6 | 14 |

**IMU (I2C):** SDA 2, SCL 1

**GPS (UART):** TX 6, RX 5

The mode switch (CH6) below 1500us hands control to the autonomous loop. If the receiver signal goes stale, no update for 200ms, the plane also falls back to autonomous instead of holding the last stick input. A lost RC link is worse than handing control to the flight controller.

### PCB

The flight controller sits on a custom carrier PCB, designed in KiCad, included in [`kicad/`](kicad). It breaks out an ESP32-S3-DevKitC into:

- 6 servo/ESC outputs and an RC signal input
- Separate UBEC power inputs for the flight controller and the ESC, each with reverse-polarity protection
- JST-SH connectors for the IMU, GPS UART, and a STEMMA QT port for future I2C sensors
- JST-SH/UART connectors for a barometer, airspeed sensor, and LIDAR. These are wired in on the board but not hooked up in software yet.

## Software Architecture

- **RC input:** 6 channels read via the ESP32-S3's MCPWM capture timers. Interrupt-driven edge timestamping converts rising/falling edges into pulse widths directly, no external library.
- **RC/servo output:** 6 channels driven via MCPWM comparators on a shared 20ms timebase.
- **IMU:** ICM-20948 driven with hand-written I2C register reads/writes, fused through a ported Madgwick AHRS filter for roll/pitch/yaw.
- **GPS:** UART driver with a hand-written NMEA RMC sentence parser, plus u-blox config commands to quiet down unused sentence types and set the fix rate.
- **PID:** a small, loop-rate-agnostic PID module (dt is passed in explicitly instead of assumed) with integral anti-windup, currently driving roll/pitch toward a level-flight setpoint.
- **Control loop:** a single 100Hz task that reads all RC channels every cycle and switches between manual pass-through and autonomous PID output based on the mode switch and RC signal health.

## Build / Flash

Set up ESP-IDF v5.5, then from the project root:

```sh
idf.py build
idf.py flash monitor
```
