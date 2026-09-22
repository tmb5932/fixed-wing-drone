# Hardware Reference

Everything a firmware developer needs to know about this board's (v3.1) hardware —
pinout, protocols, onboard parts, external connectors, power architecture —
without opening KiCad. Pulled directly from the schematic netlist
(`kicad-cli sch export netlist`) and cross-checked against manufacturer
datasheets; if GPIO assignments ever change in a future hardware revision,
this file needs a re-pull, it will drift otherwise.

## Board overview

Custom flight controller for a fixed-wing RC plane, built around an
ESP32-S3-WROOM-1-N16R8 (16MB flash, 8MB octal PSRAM). Onboard: primary IMU,
backup IMU, barometer, microSD logging. Potential (supported but may not always 
be connected in a specific drone) external (cabled, not populated on
this PCB): GPS+compass, airspeed sensor, LIDAR, UWB radio link, a second ESP
board over UART. The airspeed and gps+compass will always be connected, the rest 
are considered optional upgrades for a drone. Drives up to 6 servos and 2 ESCs, 
and takes a standard RC receiver over SBUS (RX-only) or a bidirectional 
link such as TBS Crossfire/ExpressLRS (CRSF, RX + telemetry TX). Has a USB-C 
port for debug/programming.

## MCU: ESP32-S3-WROOM-1-N16R8

LCSC `C2913202`, Espressif, 16MB flash + 8MB octal PSRAM (both integrated in
the module package).

- **GPIO35, GPIO36, GPIO37 are permanently reserved** for the octal PSRAM
  interface (SPIIO6/SPIIO7/SPIDQS) — a hard silicon fact for any module with
  8MB+ PSRAM (any "R8" or higher variant), not a schematic choice. Confirmed
  unconnected on this board; never configure them as GPIO in firmware.
  GPIO33/34 (also PSRAM-related on some variants) aren't even broken out on
  this module's footprint.
- **GPIO19 / GPIO20 are the native USB D− / D+ pins** — fixed by silicon
  (`USB_D-` / `USB_D+` on the datasheet pin table), not routed through the
  GPIO matrix, not reassignable to another peripheral.
- **Strapping pins (GPIO0, GPIO3, GPIO45, GPIO46)** — all four are in use on
  this board:
  - `GPIO0` = `BOOT` button. This is the pin's intended purpose: it and
    GPIO46 together select boot mode at reset (GPIO0=1 → SPI Boot from
    flash, the default; GPIO0=0 with GPIO46=0 → Joint Download Boot, i.e.
    UART/USB flashing mode). GPIO0 defaults to an internal weak pull-up
    (reads 1) when nothing external drives it.
  - `GPIO46` = `STATUS_LED`, driven through a series resistor and LED to
    ground (GPIO → resistor → LED anode → LED cathode → GND). GPIO46
    defaults to an internal weak pull-down (reads 0) at reset. The LED
    string doesn't fight this: nothing on that path can source current
    into the pin, so the strap still reads its correct default level
    during boot-mode sampling.
  - `GPIO45` selects VDD_SPI voltage (1.8V vs 3.3V) *only on modules without
    integrated PSRAM*. Per Espressif's own module datasheet: "for modules
    with PSRAM, the VDD_SPI voltage is fixed via eFuse, so their VDD_SPI
    voltage will not be affected by the GPIO45 level" — this module has
    PSRAM, so GPIO45's strapping function is permanently disabled at the
    factory. It's used here as a plain GPIO (`RC_TX`) with no boot
    implications.
  - `GPIO3` selects JTAG signal source at boot (USB-Serial-JTAG controller
    vs. dedicated JTAG pins vs. disabled, depending on eFuse state). It has
    **no internal pull resistor** and must not be left floating — it's
    wired here to `GPS_RX`, driven by the external GPS module's UART TX
    output, which satisfies that requirement. Worst case if a boot-time
    read goes "wrong" is JTAG selecting an unexpected signal source, not a
    boot or flashing failure.
- **ROM boot messages print to UART0 by default** (both UART0 *and* the
  USB-Serial-JTAG controller, unless disabled via eFuse). UART0's default
  TX pin (silicon-labeled `TXD0`) is wired on this board to `RC_SBUS` — the
  same wire the RC receiver actively drives with SBUS frames. During the
  brief ROM-bootloader window before your firmware takes over, both the
  chip and the receiver are trying to drive that line at once. There's no
  series resistor isolating them. This doesn't damage anything (both are
  low-current CMOS push-pull drivers), but expect boot-log output and
  early SBUS frames to both be garbled during that window — don't try to
  read SBUS data before your firmware has taken ownership of the pin, and
  if clean boot logs matter, disable ROM UART0 printing via eFuse.

## Power architecture

```
Battery/BEC ──> J20 "FC UBEC Input" ──> Q1 (reverse-polarity FET) ──> D5 (Schottky) ──┐
                                                                                       ├──> FC_5V
USB-C VBUS (J13) ─────────────────────────────────> D3 (Schottky, one-way) ──────────┘       │
                                                                              ┌────────────────┴───────────────┐
                                                                              ▼                                ▼
                                                                      U4 (XC6220B331PR-G)              U6 (AP2112K-3.3)
                                                                      5V → ESP_3V3                     5V → SENSOR_3V3
                                                                      powers U1 only                   powers IMUs/baro/
                                                                                                         UWB header

ESC1 connector (J22) BEC ──> Q2 (reverse-polarity FET) ──> SERVO_5V ──> servo/ESC connectors (J1-J6,J9,J19,J22), RC receiver (J7)
```

VBUS and the battery both feed `FC_5V` in parallel, each through their own
one-way diode — either can power the board's flight-computer rail on its
own (e.g. bench-testing over USB with no battery connected), and neither
can push current back out through the other's path.

- **Two independent 5V-ish domains, deliberately isolated.** `FC_5V`
  (flight-computer power) and `SERVO_5V` (actuator power) are separate nets
  with their own copper pours, enforced by a custom DRC rule
  (`fixed-wing-drone.kicad_dru`) requiring ≥2mm clearance between them —
  so servo/ESC current transients or brownouts can't couple into the
  flight computer's own supply. Don't assume these are the same rail.
- **Reverse-polarity protection on both power inputs.** `J20` (main
  UBEC/battery input) and `J22` (ESC1's BEC output, used as the SERVO_5V
  source) each go through a P-channel MOSFET (AO3407A, Q1/Q2) with its
  gate pulled to true ground through a 100kΩ resistor and source at the
  raw input. Correct polarity turns the FET on; reversed polarity leaves
  it off, blocking current. If a connector is ever wired backwards in the
  field, nothing downstream sees power — it just won't turn on, not smoke.
- **U4 (Torex XC6220B331PR-G, SOT-89-5)** regulates `FC_5V` → `ESP_3V3` and
  powers the ESP32-S3 module exclusively. Its thermal tab is tied to GND
  (not VOUT) and stitched to the ground plane with dedicated vias.
- **U6 (Diodes Inc AP2112K-3.3, SOT-23-5)** regulates `FC_5V` → `SENSOR_3V3`
  and powers both IMUs, the barometer, and the external UWB radio header
  (`J10`). Separate LDO from the MCU's own supply, so a noisy/loaded
  sensor rail can't sag the processor's power.
- **Only one ESC connector's BEC is actually used for power.** `J22`
  ("ESC1")'s power pin feeds `SERVO_5V` through Q2. `J19` ("ESC2")'s
  equivalent pin is intentionally left unconnected — if you plug an ESC
  with its own BEC into J19, that BEC's output goes nowhere; only J22's
  BEC (or lack of one, if you're powering servos from elsewhere) matters.
  Don't feed BEC power into both ESC connectors expecting redundancy.
- **USB-C VBUS cannot back-feed the board's main power.** `D3` (Schottky,
  anode on VBUS) only conducts VBUS → FC_5V, so if the plane's battery is
  connected while a USB cable is also plugged in, the battery can't push
  current out through the USB port toward a host device.

## Protected external interfaces

Three signal paths have TVS (transient-voltage-suppression) diodes for ESD
protection; nothing else does:

- **USB-C VBUS** (`J13`) — a TVS diode plus a 10µF bulk cap, placed ahead
  of the D3 blocking diode, right at the connector.
- **RC receiver SBUS line** (`J7` pin 1, `RC_SBUS`) — D6, the same TVS
  part, since it's an externally-exposed signal input to the flight
  computer (vs. the PWM/ESC lines, which are outputs).
- **RC receiver telemetry TX line** (`J7` pin 4, `RC_TX`) — D7, matching
  TVS part. This line is an FC output, but it's exposed on the same
  external cable as `RC_SBUS`, so it gets the same protection.

Every other external connector (servo outputs, ESC signal lines, the I2C
sensor headers, GPS/compass, inter-ESP UART, UWB SPI header) has no ESD
protection beyond what's inherent to the MCU's own I/O pins.

## Sensor buses

**SPI — one shared bus, six independent chip-selects.** `SCK` / `MOSI` /
`MISO` are common to all SPI devices on the board:

| Device | CS net | Notes |
|---|---|---|
| Primary IMU (`U2`, ICM-42688-P) | `IMU1_CS` | `INT1` wired to `IMU1_INT` (GPIO12); `INT2/FSYNC/CLKIN` and both reserved pins left unconnected |
| Backup IMU (`U3`, BMI088) | `IMU2_ACCEL_CS` + `IMU2_GYRO_CS` (two separate CS lines — the accelerometer and gyroscope are independently addressable dies in one package) | **All four interrupt pins (INT1–INT4) are unconnected.** No data-ready interrupt available from this sensor — poll it. |
| Barometer (`U7`, DPS368) | `BAROMETER_CS` | |
| MicroSD (`Card1`) | `SD_CS` (GPIO8) | Card-detect switch wired to GPIO4 (`SD_CARD_DETECT`) |
| UWB radio (external, via `J10`) | `UWB_CS` | Interrupt wired to `UWB_INT` (GPIO13) |

**I2C — one shared external bus, 6.8kΩ pull-ups (`R20`/`R21`) to
`SENSOR_3V3`.** Only external, cabled sensors are on this bus — no onboard
sensor uses I2C:

| Connector | Purpose |
|---|---|
| `J8` | LIDAR |
| `J14` | GPS/Compass (also carries GPS UART, see below) |
| `J15` | Airspeed sensor |

All three connectors' VCC pins are filtered through a ferrite bead (L2, L1,
L4 respectively) in series from `FC_5V` before reaching the connector —
this shows up as a separate auto-named net on the connector side of each
bead in netlist output, which is expected and doesn't mean anything is
disconnected.

**UART:**

| Net | Pins | Purpose |
|---|---|---|
| `GPS_RX` / `GPS_TX` | GPIO3 / GPIO9 | GPS module UART, via `J14` |
| `INTER_ESP_RX` / `INTER_ESP_TX` | GPIO5 / GPIO6 | Link to a second ESP board, via `J11` |
| `RC_SBUS` | GPIO43 (silicon `TXD0`) | RC receiver input (SBUS, or the RX half of a CRSF/ExpressLRS link), via `J7` — see the strapping-pin note above about ROM boot-time contention on this pin |
| `RC_TX` | GPIO45 | RC receiver telemetry output (unused for plain SBUS; the TX half of a CRSF/ExpressLRS link), via `J7` |

`J7` is now a 4-pin connector: pin 1 `RC_SBUS` (signal in), pin 2 `SERVO_5V`,
pin 3 `GND`, pin 4 `RC_TX` (signal out). It was a 3-pin connector (no pin 4)
before this net was added — GPIO45 previously drove an 8th PWM servo channel
(`SERVO_OUT8`, via connector `J16`), which was removed to free the pin. See
the PWM outputs section below.

## PWM outputs

7 servo channels + 2 ESC channels, each through its own 330Ω series
resistor before reaching its connector:

| Signal | GPIO | Connector |
|---|---|---|
| `SERVO_OUT1` | 44 (silicon `RXD0`) | `J1` |
| `SERVO_OUT2` | 42 | `J2` |
| `SERVO_OUT3` | 40 | `J3` |
| `SERVO_OUT4` | 38 | `J4` |
| `SERVO_OUT5` | 48 | `J5` |
| `SERVO_OUT6` | 41 | `J6` |
| `SERVO_OUT7` | 39 | `J9` |
| `ESC_OUT1` | 2 | `J22` ("ESC1" — also the SERVO_5V power source, see above) |
| `ESC_OUT2` | 1 | `J19` ("ESC2") |

`J16` (formerly `SERVO_OUT8`'s connector) and its series resistor `R16` have
been removed from the schematic — GPIO45 now drives `RC_TX` instead (see
UART section above).

## USB-C debug port (`J13`)

16-pin GCT USB4105-GF-A receptacle, USB 2.0 only (no SuperSpeed pins).
`CC1`/`CC2` each have a 5.1kΩ pull-down (`R3`/`R4`) to GND — correct UFP
(sink) termination so a compliant USB-C source will present VBUS. Native
USB D+/D− go directly to the MCU's GPIO19/GPIO20 (no series resistors, no
protection beyond the VBUS TVS above). Populated for field use, not DNP.

## Other onboard parts

- **`D1`/`D2`** — status and power indicator LEDs (yellow-green and red
  respectively), each with its own current-limiting resistor.
- **`SW1`** — reset button. Pulls `EN` (chip enable/reset) to GND when
  pressed; `EN` has its own 10kΩ pull-up to `ESP_3V3` (`R7`) so it reads
  high otherwise.
- **`SW2`** — BOOT button. Pulls GPIO0 to GND through a 100Ω series
  resistor (`R9`) when pressed — hold this while tapping reset to enter
  UART/USB download mode (see the GPIO0/GPIO46 boot-mode note above).
