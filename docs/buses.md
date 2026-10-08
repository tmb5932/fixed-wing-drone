# Hardware Reference

Everything a firmware developer needs to know about this board's (v3.1) hardware —
pinout, protocols, onboard parts, external connectors, power architecture —
without opening KiCad. Pulled directly from the schematic netlist
(`kicad-cli sch export netlist`, re-pulled 2026-10-08) and cross-checked
against manufacturer datasheets. If GPIO assignments ever change in a future
hardware revision, this file needs a re-pull; it will drift otherwise.

**Note:** this describes the v3.1 board in `kicad/flight-controller/`. The
firmware in `main/` currently targets the built v2.1 board (ESP32-S3-DevKitC
carrier, PWM receiver, ICM-20948 over I2C), so none of the GPIO assignments
below are in the code yet. See the "Firmware to-do for bring-up" section of
`kicad/flight-controller/design-review-fixed-wing-drone.md`.

## Board overview

Custom flight controller for a fixed-wing RC plane, built around an
ESP32-S3-WROOM-1-N16R2 (16MB flash, 2MB quad PSRAM). Onboard: primary IMU,
backup IMU, barometer, microSD logging. Potential (supported but may not always
be connected in a specific drone) external (cabled, not populated on
this PCB): GPS+compass, airspeed sensor, LIDAR, UWB radio link, a second ESP
board over UART, an IR beacon LED. The airspeed and gps+compass will always be
connected; the rest are considered optional upgrades for a drone. Drives up to
6 servos and 2 ESCs, and takes a standard RC receiver over SBUS (RX-only) or a
bidirectional link such as TBS Crossfire/ExpressLRS (CRSF, RX + telemetry TX).
Has a USB-C port for debug/programming.

## MCU: ESP32-S3-WROOM-1-N16R2

LCSC `C2913205`, Espressif, 16MB flash + 2MB **quad** PSRAM (both integrated
in the module package).

- **Do not substitute an R8/R16V (octal PSRAM) module.** On those variants
  GPIO35/36/37 are wired internally to the octal PSRAM. This board uses those
  three pins for the microSD bus, which only works with quad PSRAM. The
  N16R2 is also rated −40–85 °C, versus −40–65 °C for the N16R8. In
  `sdkconfig`, PSRAM must be set to 2MB quad, not octal.
- **GPIO19 / GPIO20 are the native USB D− / D+ pins.** They're fixed by
  silicon, not routed through the GPIO matrix, and can't be reassigned to
  another peripheral.
- **Strapping pins (GPIO0, GPIO3, GPIO45, GPIO46)** — all four are in use on
  this board:
  - `GPIO0` = `BOOT` button. This is the pin's intended purpose: it and
    GPIO46 together select boot mode at reset (GPIO0=1 → SPI Boot from
    flash, the default; GPIO0=0 with GPIO46=0 → Joint Download Boot, i.e.
    UART/USB flashing mode). GPIO0 defaults to an internal weak pull-up
    (reads 1) when nothing external drives it.
  - `GPIO46` = `STATUS_LED`, driven through a 220Ω resistor (`R5`) and LED
    (`D1`) to ground. GPIO46 defaults to an internal weak pull-down (reads 0)
    at reset. The LED string can't source current into the pin, so the strap
    still reads its correct default level during boot-mode sampling.
  - `GPIO45` = `LED_TOGGLE`, the IR beacon MOSFET gate (see "Other onboard
    parts"). GPIO45 is the VDD_SPI voltage strap when eFuse
    `EFUSE_VDD_SPI_FORCE` is not set: 0 → 3.3V, 1 → 1.8V. `R17` (100kΩ)
    pulls the gate, and therefore GPIO45, to ground, so it reads 0 at reset
    and selects 3.3V, which is the correct VDD_SPI for this module either way.
  - `GPIO3` selects JTAG signal source at boot (only if the
    `STRAP_JTAG_SEL` eFuse is set). It has **no internal pull resistor** and
    must not be left floating. It's wired here to `GPS_RX`, driven by the
    external GPS module's UART TX output (idle high), which satisfies that
    requirement.
- **The RC link is on UART0** (`RC_SBUS` on GPIO44/`RXD0`, `RC_TX` on
  GPIO43/`TXD0`). The ROM bootloader prints its boot log to UART0 by default,
  so on every boot that log is transmitted out of `RC_TX` toward the
  receiver. Suppress ROM UART0 printing (eFuse, or GPIO46's log-control
  strap), or move the console to USB-Serial-JTAG.

## Power architecture

```
Battery/UBEC ──> J20 "FC UBEC Input" (pin 2) ──> D5 (B5819W Schottky) ──┐
                                                                         ├──> FC_5V
USB-C VBUS (J13) ──> D4 (ESD) + C17 ─────────> D3 (B5819W, one-way) ────┘       │
                                                              ┌──────────────────┴─────────────────┐
                                                              ▼                                    ▼
                                                      U4 (XC6220B331PR-G)                 U6 (AP2112K-3.3)
                                                      5V → ESP_3V3                        5V → SENSOR_3V3
                                                      powers U1, microSD,                 powers both IMUs,
                                                      LEDs, IR beacon feed                barometer, I2C pull-ups

FC_5V ──> ferrites L1/L2/L3/L4 ──> external connectors J14 (GPS), J8 (LIDAR), J10 (UWB), J15 (airspeed)

ESC1 connector (J22) BEC, pin 2 ─────────> SERVO_5V ──> servo connectors (J1-J6), RC receiver (J7), C20 (22µF)
```

VBUS and the battery both feed `FC_5V` in parallel, each through its own
one-way Schottky. Either can power the flight-computer rail on its own (e.g.
bench-testing over USB with no battery connected), and neither can push
current back out through the other's path. `FC_5V` sits at roughly
4.5–4.7V after the diode drop.

- **Two independent 5V domains, deliberately isolated.** `FC_5V`
  (flight-computer power) and `SERVO_5V` (actuator power) are separate nets
  with their own copper pours, enforced by a custom DRC rule
  (`fixed-wing-drone.kicad_dru`, `FC_SERVO_5V_isolation`) requiring ≥2mm
  clearance between them, so servo/ESC current transients or brownouts
  can't couple into the flight computer's own supply. Don't assume these are
  the same rail.
- **Reverse polarity: protected by connector pinout and diodes, not FETs.**
  `J20` is GND / V+ / GND (pins 1 / 2 / 3), so a 3-pin plug works either way
  round, and D5 blocks reverse current. The SERVO_5V input (`J22` pin 2)
  has no reverse-polarity element. The earlier AO3407A P-FET stages (Q1/Q2)
  were removed in v3.1: Q1 was redundant with D5, and Q2's thermal margin
  was inadequate for six servos.
- **U4 (Torex XC6220B331PR-G, SOT-89-5)** regulates `FC_5V` → `ESP_3V3`.
  Input cap C18 (22µF/25V 0805), output caps C10 (22µF) + C19 (4.7µF) + C11
  (100nF). Its thermal tab is tied to GND and stitched to the ground plane.
- **U6 (Diodes Inc AP2112K-3.3, SOT-23-5)** regulates `FC_5V` →
  `SENSOR_3V3` for both IMUs, the barometer and the I2C pull-ups. It's a
  separate LDO from the MCU's supply, so a noisy or loaded sensor rail can't
  sag the processor's power.
- **The UWB header (`J10`) gets 5V, not 3.3V** (FC_5V through ferrite L3).
  The UWB board must carry its own 3.3V LDO for the DWM3000.
- **Only one ESC connector's BEC is actually used for power.** `J22`
  ("ESC1")'s pin 2 feeds `SERVO_5V`. `J19` ("ESC2") pin 2 is intentionally
  left unconnected: if you plug an ESC with its own BEC into J19, that BEC's
  output goes nowhere. Don't feed BEC power into both ESC connectors
  expecting redundancy.

## Protected external interfaces

All ESD protection is on the USB port and the RC receiver connector. Nothing
else has any:

- **USB-C VBUS** (`J13`): `D4` (ESD5Z5.0T1G) plus a 10µF cap (`C17`),
  ahead of the D3 blocking diode.
- **USB-C D+/D−**: `U5` (USBLC6-2SC6) ESD array.
- **RC receiver SBUS line** (`J7` pin 1, `RC_SBUS`): `D6` (ESD5Z5.0T1G)
  at the connector, then a 330Ω series resistor (`R27`) to the MCU.
- **RC receiver telemetry TX line** (`J7` pin 4, `RC_TX`): `D7`
  (ESD5Z5.0T1G) at the connector, then a 330Ω series resistor (`R26`).

Every other external connector (servo outputs, ESC signal lines, the I2C
sensor headers, GPS/compass, inter-ESP UART, UWB SPI header, IR beacon) has
no ESD protection beyond the MCU's own I/O pins.

## Sensor buses

**SPI — one shared sensor bus, five chip-selects.** `SCK` (GPIO14) /
`MOSI` (GPIO47) / `MISO` (GPIO21) are common to all of these:

| Device | CS net (GPIO) | Notes |
|---|---|---|
| Primary IMU (`U2`, ICM-42688-P) | `IMU1_CS` (GPIO18) | `INT1` wired to `IMU1_INT` (GPIO12). `INT2/FSYNC/CLKIN` (pin 9) and the reserved pins are unconnected. |
| Backup IMU (`U3`, BMI088) | `IMU2_ACCEL_CS` (GPIO15) + `IMU2_GYRO_CS` (GPIO7) | Two CS lines: the accelerometer and gyroscope are separate dies in one package. **All four interrupt pins (INT1–INT4) are unconnected.** Poll it. |
| Barometer (`U7`, DPS368) | `BAROMETER_CS` (GPIO17) | SPI mode 3 only. |
| UWB radio (external, via `J10`) | `UWB_CS` (GPIO16) | Interrupt on `UWB_INT` (GPIO13). Shares the bus over an off-board cable with no series resistors, so a harness fault on MISO takes out both IMUs and the barometer too. |

None of the five CS lines has an external pull-up. Drive them all high (with
internal pull-ups enabled) at the very start of `app_main`, before any SPI
init.

**SPI — dedicated microSD bus** on its own SPI host:

| Signal | GPIO |
|---|---|
| `SD_SCK` | 35 |
| `SD_MOSI` | 36 |
| `SD_CS` | 37 |
| `SD_MISO` | 8 |
| `SD_CARD_DETECT` | 4 (10kΩ pull-up `R11`) |

`SD_MISO`/`SD_CS`/`SD_MOSI` have 10kΩ pull-ups (`R28`/`R29`/`R30`) to
`ESP_3V3`. The unused DAT1/DAT2 card pins are pulled up via `R2`/`R1`.

**I2C — one shared external bus.** `SDA` = GPIO10, `SCL` = GPIO11, 4.7kΩ
pull-ups (`R20`/`R21`) to `SENSOR_3V3`. Only external, cabled sensors are on
this bus; no onboard sensor uses I2C:

| Connector | Purpose | Pinout |
|---|---|---|
| `J8` | LIDAR | 1 5V / 2 SCL / 3 SDA / 4 GND |
| `J14` | GPS/Compass (also carries GPS UART, see below) | 1 SDA / 2 GND / 3 GPS_RX / 4 GPS_TX / 5 5V / 6 SCL |
| `J15` | Airspeed sensor | 1 5V / 2 SCL / 3 SDA / 4 GND |

All three connectors' 5V pins are filtered through a ferrite bead (L2, L1,
L4 respectively) in series from `FC_5V`. This shows up as a separate
auto-named net on the connector side of each bead in netlist output, which is
expected and doesn't mean anything is disconnected.

**UART:**

| Net | GPIO | Purpose |
|---|---|---|
| `GPS_RX` / `GPS_TX` | 3 / 9 | GPS module UART, via `J14`. Pinout matches the BN-880 with TX/RX already crossed, so a straight-through cable works. BN-880 default is 38400 baud. |
| `INTER_ESP_RX` / `INTER_ESP_TX` | 5 / 6 | Link to the mission board, via `J11` (1 GND / 2 TX / 3 RX) |
| `RC_SBUS` | 44 (silicon `RXD0`) | RC receiver input (SBUS, or the RX half of a CRSF/ExpressLRS link), via `J7`. SBUS needs UART RX inversion enabled. |
| `RC_TX` | 43 (silicon `TXD0`) | RC receiver telemetry output (unused for plain SBUS; the TX half of a CRSF/ExpressLRS link), via `J7`. See the ROM boot-log note above. |

`J7` is a 4-pin connector: pin 1 `RC_SBUS` (signal in), pin 2 `SERVO_5V`,
pin 3 `GND`, pin 4 `RC_TX` (signal out).

## PWM outputs

6 servo channels + 2 ESC channels, each through its own 330Ω series resistor
before reaching its connector:

| Signal | GPIO | Connector |
|---|---|---|
| `SERVO_OUT_1` | 42 | `J1` |
| `SERVO_OUT_2` | 41 | `J2` |
| `SERVO_OUT_3` | 39 | `J3` |
| `SERVO_OUT_4` | 48 | `J4` |
| `SERVO_OUT_5` | 40 | `J5` |
| `SERVO_OUT_6` | 38 | `J6` |
| `ESC_OUT1` | 2 | `J22` ("ESC1", also the SERVO_5V power source, see above) |
| `ESC_OUT2` | 1 | `J19` ("ESC2", pin 2 NC) |

Servo connectors are signal / SERVO_5V / GND (pins 1 / 2 / 3). ESC_OUT1/2
have no pull-downs, so they float during boot until firmware drives them.
ESCs need a valid pulse train to arm, so this is low risk.

## USB-C debug port (`J13`)

16-pin GCT USB4105-GF-A receptacle, USB 2.0 only (no SuperSpeed pins).
`CC1`/`CC2` each have a 5.1kΩ pull-down (`R3`/`R4`) to GND: correct UFP
(sink) termination, so a compliant USB-C source will present VBUS. Native
USB D+/D− go to the MCU's GPIO20/GPIO19 through the `U5` USBLC6-2SC6 ESD
array, with no series resistors.

## Other onboard parts

- **`D1`** — status LED on GPIO46 via `R5` (220Ω). **`D2`** — power LED
  on `ESP_3V3` via `R6` (220Ω).
- **IR beacon output (`J9`)** — a 3-pin connector for an external IR LED.
  Pins 1 and 3 are fed from `ESP_3V3` through 33Ω current-limit resistors
  (`R24`/`R25`, 1206, 0.75W). Pin 2 is the low-side switch: `Q3` (AO3400A
  N-MOSFET), gate driven from GPIO45 (`LED_TOGGLE`) through `R16` (220Ω),
  with `R17` (100kΩ) pulling it off at boot.
- **`SW1`** — reset button. Pulls `EN` (chip enable/reset) to GND when
  pressed. `EN` has a 10kΩ pull-up to `ESP_3V3` (`R7`) and an RC filter
  (`C12` 1µF + `C14` 100nF) placed within a few mm of the module pin.
- **`SW2`** — BOOT button. Pulls GPIO0 to GND through a 100Ω series
  resistor (`R9`) when pressed. Hold it while tapping reset to enter
  UART/USB download mode (see the GPIO0/GPIO46 boot-mode note above).
- **Test points** — `TP1`–`TP3` (MOSI/MISO/SCK), `TP5`/`TP6` (SDA/SCL),
  `TP7` (ESP_3V3), `TP8` (SENSOR_3V3), `TP9` (FC_5V), `TP4`/`TP10` (GND).
