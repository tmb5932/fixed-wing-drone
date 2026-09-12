# Hardware Reference

Everything a firmware developer needs to know about this board's hardware —
pinout, protocols, onboard parts, external connectors, power architecture —
without opening KiCad. Pulled directly from the schematic/netlist as of the
final pre-fabrication design; if GPIO assignments ever change in a future
hardware revision, this file needs a re-pull (`kicad-cli sch export netlist`),
it will drift otherwise.

> ⚠️ **Known unresolved hardware bug:** `J14` (GPS/Compass connector) pin 5,
> which should carry VCC power to the module, is not connected to anything —
> it's a dangling wire that was drawn toward the `FC_5V` rail but never
> reached it. Every other pin on that connector is correctly wired. Until
> this is fixed in the schematic, **the GPS/compass module will not power
> on** even though its data lines (SDA/SCL/UART) are correctly connected.
> Don't spend time debugging a "dead" GPS in firmware before checking this.

## Board overview

Custom flight controller for a fixed-wing RC plane, built around an
ESP32-S3-WROOM-1-N16R8 (16MB flash, 8MB octal PSRAM). Onboard: dual IMU
(primary + backup), barometer, microSD logging. External (cabled, not
populated on this PCB): GPS+compass, airspeed sensor, lidar, UWB radio.
Drives up to 8 servos and 2 ESCs, and passes through a standard RC receiver
(SBUS).

## MCU: ESP32-S3-WROOM-1-N16R8

- LCSC `C2913202`, Espressif, 16MB flash + 8MB octal PSRAM (both integrated
  in the module package).
- **GPIO33–37 are permanently reserved** for the octal PSRAM interface —
  hard silicon fact for this exact module, not a schematic choice. Not
  wired to anything on this board; never configure them as GPIO in firmware.
- **GPIO19/GPIO20** are the native USB D-/D+ pins — fixed by silicon, not
  routed through the GPIO matrix, not reassignable.
- **GPIO43/GPIO44** are the chip's default UART0 TX/RX pins (only used by
  the ROM bootloader for boot-time messages before firmware starts) —
  repurposed on this board for `SERVO_OUT1`/`ESC_OUT2`. This is safe; see
  the PWM section below for the one thing to know about it.
- **Strapping pins (GPIO0, GPIO3, GPIO45, GPIO46)** — all four are in use on
  this board, all confirmed safe against the real ESP32-S3-WROOM-1-N16R8
  datasheet:
  - `GPIO0` = `BOOT` button — this is the pin's intended purpose (LOW =
    download/flash mode, HIGH = normal boot).
  - `GPIO45` = `SERVO_OUT8` — normally this pin selects flash voltage, but
    this specific module has integrated PSRAM, which means Espressif burns
    the `VDD_SPI_FORCE` eFuse at the factory, permanently disabling that
    strapping function. Confirmed via the module's own reference schematic.
    Free to use as a normal GPIO.
  - `GPIO46` = `STATUS_LED` — this pin affects boot-mode entry (must be
    LOW/floating to enter UART download mode), but a passive LED+resistor
    load doesn't actively drive a level onto it before firmware runs, and
    the pin's own internal weak pull-down (its documented default state)
    already reads LOW, which is exactly what's needed. Safe.
  - `GPIO3` = `GPS_RX` — this pin selects JTAG signal source only (not boot
    success). It has no internal pull resistor and per the datasheet
    *needs* an active external driver to avoid an undefined strap read —
    the GPS module's UART TX output satisfies that requirement, and the
    worst case if it reads "wrong" is JTAG selecting an unexpected source,
    not a boot or flashing failure.

## Complete GPIO pinout

All 41 module pins. Fixed/silicon-level pins are marked; everything else is
routed through the GPIO matrix and was a schematic-time choice (still
correct as of this doc, but re-verify against the schematic if it's been a
while since this was written).

| Pin | GPIO | Net | Notes |
|---|---|---|---|
| 1 | — | GND | |
| 2 | — | `ESP_3V3` | power |
| 3 | — | `EN` | reset, RC delay circuit |
| 4 | GPIO4 | `SD_CARD_DETECT` | microSD socket mechanical switch |
| 5 | GPIO5 | `INTER_ESP_RX` | UART |
| 6 | GPIO6 | `INTER_ESP_TX` | UART |
| 7 | GPIO7 | `IMU2_GYRO_CS` | SPI |
| 8 | GPIO15 | `IMU2_ACCEL_CS` | SPI |
| 9 | GPIO16 | `UWB_CS` | SPI (external module, not populated) |
| 10 | GPIO17 | `BAROMETER_CS` | SPI |
| 11 | GPIO18 | `IMU1_CS` | SPI |
| 12 | GPIO8 | `SD_CS` | SPI |
| 13 | GPIO19 | USB D- | **fixed, native USB** |
| 14 | GPIO20 | USB D+ | **fixed, native USB** |
| 15 | GPIO3 | `GPS_RX` | UART, **strapping pin** (see above, safe) |
| 16 | GPIO46 | `STATUS_LED` | **strapping pin** (see above, safe) |
| 17 | GPIO9 | `GPS_TX` | UART |
| 18 | GPIO10 | `SDA` | I2C |
| 19 | GPIO11 | `SCL` | I2C |
| 20 | GPIO12 | `IMU1_INT` | interrupt input |
| 21 | GPIO13 | `UWB_INT` | interrupt input (external module) |
| 22 | GPIO14 | `SCK` | SPI clock |
| 23 | GPIO21 | `MISO` | SPI |
| 24 | GPIO47 | `MOSI` | SPI |
| 25 | GPIO48 | `SERVO_OUT5` | PWM |
| 26 | GPIO45 | `SERVO_OUT8` | PWM, **strapping pin** (see above, safe) |
| 27 | GPIO0 | `BOOT` | **strapping pin**, intended use |
| 28 | GPIO35 | unconnected | **reserved, octal PSRAM — never use** |
| 29 | GPIO36 | unconnected | **reserved, octal PSRAM — never use** |
| 30 | GPIO37 | unconnected | **reserved, octal PSRAM — never use** |
| 31 | GPIO38 | `SERVO_OUT4` | PWM |
| 32 | GPIO39 | `SERVO_OUT7` | PWM |
| 33 | GPIO40 | `SERVO_OUT3` | PWM |
| 34 | GPIO41 | `SERVO_OUT6` | PWM |
| 35 | GPIO42 | `SERVO_OUT2` | PWM |
| 36 | GPIO44 | `SERVO_OUT1` | PWM — chip's default UART0 RX pin, repurposed |
| 37 | GPIO43 | `RC_SBUS` | UART/serial input — chip's default UART0 TX pin, repurposed |
| 38 | GPIO2 | `ESC_OUT1` | PWM |
| 39 | GPIO1 | `ESC_OUT2` | PWM |
| 40 | — | GND | |
| 41 | — | GND | |

## SPI bus (shared, one peripheral, independent chip-selects)

`SCK`=GPIO14, `MOSI`=GPIO47, `MISO`=GPIO21.

| Device | Part | CS net / GPIO | Notes |
|---|---|---|---|
| IMU1 (primary) | ICM-42688-P, LCSC `C1850418` | `IMU1_CS` / GPIO18 | interrupt on `IMU1_INT` / GPIO12 |
| IMU2 (backup) | BMI088, LCSC `C194919` | `IMU2_ACCEL_CS` / GPIO15, `IMU2_GYRO_CS` / GPIO7 | one package, two independent sub-cores (accel + gyro), each with its own CS; both share one `MISO` line safely since only one CS is ever active |
| Barometer | Infineon DPS368XTSA1, LCSC `C3232508` | `BAROMETER_CS` / GPIO17 | drop-in successor to DPS310 (discontinued) — same register map per Infineon, so existing DPS310 driver code should need little to no change |
| microSD (SPI mode) | — | `SD_CS` / GPIO8 | see microSD section below |
| UWB radio | — | `UWB_CS` / GPIO16 | external module via J10, not populated on this PCB; interrupt on `UWB_INT` / GPIO13 |

### IMU2 (BMI088) — populated, but write the firmware to treat it as optional anyway

IMU2 is populated on this board run (it was briefly marked DNP to save
cost, then reinstated once the actual per-unit cost turned out lower than
first estimated — worth knowing in case an older note or memory of this
project says otherwise). Even so, it's worth writing the driver/fusion code
as if it *might* be absent, rather than hardcoding an assumption it's
always there — cheap insurance against a future board revision dropping it
again, and against a single populated unit having a dead/DOA IMU2 at
assembly. Two ways to structure this, and dynamic detection is the better
one:

- **Compile-time (`#ifdef`)**: simplest, but means a separate firmware build
  per board variant — easy to accidentally flash the wrong build to the
  wrong board.
- **Runtime detection (preferred)**: at boot, attempt a SPI transaction with
  `IMU2_ACCEL_CS`/`IMU2_GYRO_CS` and read BMI088's chip-ID register (`0x00`
  on both the accel and gyro sub-core, expected values `0x1E`/`0x0F`
  respectively per the BMI088 datasheet). If the read times out or returns
  an unexpected value, treat IMU2 as absent and fall back to IMU1-only
  operation — no separate firmware build needed, the same binary works on
  both populated and unpopulated boards, and it's robust to a board being
  populated later without a firmware change.

Either way, whatever attitude/sensor-fusion code consumes IMU data should
already be written against "IMU1, optionally IMU2" rather than "always
exactly one IMU" or "always exactly two" — this avoids a rewrite whenever
BMI088 does get populated (or not) on a future run.

**Extend this to runtime fault tolerance, not just boot-time detection, and
make it symmetric between the two IMUs.** Detecting absence at boot handles
"not populated" — it doesn't handle a sensor that responds fine at boot but
then stops mid-flight (bad solder joint working loose, ESD, physical shock
from a hard landing, whatever). The driver/fusion layer should treat SPI
comms failures or invalid data from *either* IMU as a live fault condition
at any point during operation, not just something checked once at startup,
and fall back to single-IMU operation using whichever one is still
responding — this should work the same way regardless of which specific
IMU (1 or 2) is the one that failed. Don't hardcode IMU1 as the assumed
"always-good" one that IMU2 falls back to; either should be able to carry
flight on its own if the other drops out.

### microSD — SPI mode is native, not a workaround

microSD cards have SPI mode built into their own controller as an official
part of the SD spec — a simplified 4-wire mode any generic SPI peripheral
can drive directly, no bridge chip involved. What turns raw SPI transfers
into "write a file" is a software filesystem driver (FatFS via ESP-IDF's
`esp_vfs_fat`) — a firmware concern, not a hardware one.

**The SD card is the bus-timing risk on this shared bus, not the IMUs.**
SD-in-SPI-mode runs comfortably at 20–25MHz, same ballpark as the IMUs —
the actual bottleneck is that the card's internal flash write takes real
time (commonly several hundred microseconds up to a few ms per 512-byte
block) regardless of bus speed, and during that time the bus is tied up.
Realistic sustained write throughput: roughly 200KB/s–800KB/s, card
dependent. The risk isn't bandwidth, it's that **a single write can block
the bus for a few ms**, which can cause missed samples if the IMU wants
data faster than that.

Firmware mitigation (scheduling, not hardware):
1. Don't log at the IMU's raw output rate — downsample to 50–100Hz.
2. Batch writes — accumulate samples in RAM (8MB PSRAM available) and flush
   a larger chunk every few hundred ms rather than one transaction per cycle.
3. Prioritize sensor reads over SD writes — sensor polling in a
   higher-priority task/ISR than the SD write task.
4. Flush right after a sensor read cycle completes, not mid-cycle.

## I2C bus (external sensors only — nothing onboard uses I2C)

`SDA`=GPIO10, `SCL`=GPIO11. Pull-ups: 6.8kΩ, to `ESP_3V3`.

| Device | Connector | Pinout |
|---|---|---|
| GPS + compass (Beitian BN-880) | J14 (6-pin JST-GH) | 1=SDA, 2=GND, 3=RX←module TX (`GPS_RX`), 4=TX→module RX (`GPS_TX`), 5=VCC **(currently unwired — see known-bug notice above)**, 6=SCL |
| Airspeed sensor | J15 (4-pin JST-GH) | 1=+5V (`FC_5V`, via ferrite bead L4), 2=SCL, 3=SDA, 4=GND |
| Lidar | J8 (4-pin Molex PicoBlade) | 1=+5V (`FC_5V`, via ferrite bead L2), 2=SCL, 3=SDA, 4=GND |

### I2C address map — verify before assuming no conflicts

| Device | 7-bit address | Notes |
|---|---|---|
| HMC5883L (compass, if this chip variant) | `0x1E` | fixed |
| QMC5883L (compass, if this chip variant) | `0x0D` (or `0x0C`) | fixed; BN-880 units carry either chip depending on manufacturing batch — check which one you actually have |
| MS4525DO (airspeed, if used) | `0x28` commonly | configurable via a separate Honeywell/TE app note if it conflicts |
| DLVR series (airspeed, if used) | order-code dependent | check the datasheet's ordering guide |
| Lidar | model-dependent | e.g. Garmin LIDAR-Lite `0x62`, Benewake TFmini/TF-Luna `0x10`, ST VL53L0X/L1X `0x29` |

None of the likely combinations collide, but the fully reliable way to know
for sure once hardware is in hand: run an I2C bus scanner (probe every
address 0x08–0x77, log ACKs) rather than trust datasheets alone.

## UART

| Signal | GPIO | Notes |
|---|---|---|
| `GPS_RX` (host receives) | GPIO3 | NMEA in from GPS module |
| `GPS_TX` (host transmits) | GPIO9 | |
| `INTER_ESP_RX` | GPIO5 | to a second ESP32 board via J11 (3-pin JST-GH) |
| `INTER_ESP_TX` | GPIO6 | |
| `RC_SBUS` | GPIO43 | from RC receiver via J7 (3-pin Molex KK-254). **SBUS is inverted UART** — configure via the ESP32 UART peripheral's RX-invert option, or verify your specific receiver's actual output polarity before assuming standard inverted SBUS |

GPIO43/44 (native UART0 default pins, used here for `RC_SBUS` and
`SERVO_OUT1`) briefly carry ROM bootloader boot-log traffic for a moment at
power-on before firmware takes over — harmless, just don't be surprised by
a flicker of activity on those lines in the first instant after reset.

## USB

Native USB (not a UART-bridge chip) on the chip's dedicated USB pins,
routed to `J13` (USB-C receptacle, GCT USB4105-xx-A). This is the primary
flashing and serial-console path for this board — GPIO43/44 (UART0) are
free for other use specifically because flashing doesn't depend on them.

## PWM outputs (8 servos + 2 ESCs)

All ten have a 330Ω series resistor between the GPIO and the connector pin
— fault-current protection in case a signal pin ever shorts to the adjacent
power or ground pin at the connector (a real, plausible failure mode with
these small connectors, not just theoretical).

| Output | GPIO | Connector |
|---|---|---|
| Servo 1 | GPIO44 | J1 |
| Servo 2 | GPIO42 | J2 |
| Servo 3 | GPIO40 | J3 |
| Servo 4 | GPIO38 | J4 |
| Servo 5 | GPIO48 | J5 |
| Servo 6 | GPIO41 | J6 |
| Servo 7 | GPIO39 | J9 |
| Servo 8 | GPIO45 | J16 |
| ESC 1 | GPIO2 | J22 |
| ESC 2 | GPIO43 | J19 |

All servo/ESC/RC connectors (J1-J7,J9,J16,J19,J20,J22) are through-hole
Molex KK-254 parts marked **DNP** (not JLCPCB-assembled) — hand-soldered.

## Power architecture

Two independent 3.3V LDOs and a split 5V power plane, both for noise
isolation between "ESP32/digital" and "sensor/analog" domains:

| Rail | Source | Feeds |
|---|---|---|
| `ESP_3V3` | U5 (AP2112K-3.3, LCSC `C51118`) | ESP32-S3 module, I2C pull-ups |
| `SENSOR_3V3` | U6 (AP2112K-3.3, LCSC `C51118`) | IMU1, IMU2, Barometer |
| `FC_5V` | USB VBUS (via D3) or ESC1 BEC (via Q1 + D5), power-OR'd | feeds both LDOs |
| `SERVO_5V` | ESC1 BEC (via Q2), separate plane from `FC_5V` on the same inner layer | all 8 servo connectors + RC receiver connector (J7) |

D3/D5 are B5819W SL Schottky diodes (LCSC `C8598`, 40V/1A). `FC_5V` and
`SERVO_5V` are deliberately isolated (2mm clearance rule) so servo/ESC
current transients don't couple into the flight-critical logic supply —
this is intentional, not something to "fix" if you ever see them as
separate rails in firmware/telemetry.

## Status/Power LEDs

| LED | GPIO / net | Part |
|---|---|---|
| D1 (Status) | GPIO46 (`STATUS_LED`) | KT-0603YG, yellow-green, LCSC `C2289` |
| D2 (Power) | passive (power-on indicator, not GPIO-driven) | — |

D1 is GPIO-driven through a 220Ω resistor (R5) — firmware controls it
directly. Expect roughly 5-6mA drive current given the LED's ~2.0-2.2V
forward voltage on the 3.3V rail.
