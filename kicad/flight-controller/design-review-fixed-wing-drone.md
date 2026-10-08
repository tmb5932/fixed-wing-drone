# Autonomous Fixed-Wing Drone Flight Controller — Design Review

**Project:** `fixed-wing-drone` v3.1 (KiCad 10.0.1, 12 hierarchical sheets, 4-layer PCB, 91.0 × 61.5 mm)
**Date:** 2026-10-03
**Baseline:** commit `ddca75b` ("progress on v1 of mission board"). "Changes since `ddca75b`" lists everything changed after that commit.
**Location:** `kicad/flight-controller/`

## Review process

1. **Four independent reviews.** Two schematic reviewers and two PCB reviewers each did a full review without seeing each other's work, and without seeing earlier review documents. Between them they raised 35 distinct issues.
2. **Independent verification.** A separate verifier checked every claim against KiCad's netlist, the PCB geometry (pcbnew Python), the manufacturer datasheets (Torex XC6220, AOS AO3407A, TDK ICM-42688-P, Bosch BMI088, Espressif ESP32-S3, ST USBLC6-2) and live JLC/LCSC catalog lookups. Results: 21 confirmed, 12 partially confirmed with corrected severity, 0 pure false positives, and two conflicts between reviewers resolved.
3. **Iterative checks after each round of fixes.** After each round I diffed the netlist and PCB footprints, ran `kicad-cli sch erc`, and ran `kicad-cli pcb drc --schematic-parity --refill-zones`. KiCad's own netlist is treated as ground truth for connectivity.

Not re-run this pass: the kicad-happy EMC, thermal and SPICE analyzers. The changes were targeted and were verified directly instead (see "Review limits").

## Verdict

**Ready for fab.** Round 2 (below) added no must-fix items on the flight controller itself. The one cross-board item, the inter-ESP connector (J11 ↔ mission-board J18), has since been fixed on the mission board.

Current state, with zones refilled:

| Check | Result |
|---|---|
| ERC errors | 0 |
| ERC warnings | 1, cosmetic: J7 library-symbol mismatch |
| DRC unconnected items | 0 |
| DRC clearance errors | 0 |
| DRC remaining errors | Only J13 pads A1/B12 "starved thermal" (accepted, see below) |
| Schematic ↔ PCB parity | In sync. Only board-only items (FID1–3 fiducials, M2 mounting holes) are flagged. |

## Changes since `ddca75b`

### Schematic and netlist

| Change | Detail |
|---|---|
| **ESP32 module variant** | U1 `ESP32-S3-WROOM-1-N16R8` (C2913202) → `-N16R2` (C2913205). The 2 MB **quad** PSRAM frees GPIO35/36/37; on R8/R16V parts these pins are wired to the octal PSRAM. The N16R2 is also rated −40–85 °C, versus −40–65 °C for the N16R8. **Do not substitute an R8 part.** |
| **Dedicated microSD SPI bus** | SD_SCK = IO35 (U1.28), SD_MOSI = IO36 (U1.29), SD_CS = IO37 (U1.30), SD_MISO = IO8 (U1.12). Card1 is no longer on the shared sensor bus. R28/R29/R30 (10k pull-ups to ESP_3V3) added on SD_MISO/SD_CS/SD_MOSI. |
| **USB data ESD** | D8/D9 (2× ESD5Z5.0T1G) replaced by U5 USBLC6-2SC6 (C7519). Pins 1/6 = D+, 3/4 = D−, 2 = GND, 5 = VBUS. The VBUS net was renamed `/esp32s3/USB_VBUS`. |
| **FC 5 V input** | Q1/R8 (P-FET reverse-polarity stage) removed. D5 (B5819W) already blocks reverse current. J20 pin 1 is now GND, so J20 = GND / V+ / GND and the plug can go in either way round. |
| **Servo rail** | Q2/R10 removed. The FET's thermal margin was inadequate for six servos, and it gave no protection against a flipped centre-V+ servo plug. J22 pin 2 now feeds SERVO_5V directly. C20 (22 µF / 25 V 0805, C45783) added on SERVO_5V near J7. |
| **U4 input cap** | C18: 10 µF / 6.3 V 0603 (C1691) → 22 µF / 25 V 0805 (C45783). The old part derated to roughly 3–4 µF at 5 V, which falls in the XC6220's "CIN 4.7 µF needs CL 47 µF" stability row. |
| **I2C pull-ups** | R20/R21: 6.8k → 4.7k (C25900), for margin with three cabled I2C devices. |
| **UWB power** | J10 pin 1: SENSOR_3V3 → FC_5V through ferrite L3 (BLM18PG121SN1D). The UWB board must carry its own 3.3 V LDO (see "Cross-board dependency"). |
| **BOM fixes** | J8 LCSC C234195 (not in JLC catalog) → C17617036. R26/R27 (330 Ω, RC_TX/RC_SBUS series resistors) given LCSC C25104. Without a number, JLC would have skipped them and left the RC lines open. R24/R25 (33 Ω 1206 IR LED feed) given C5759775 (0.75 W). ESP32 sheet title corrected to N16R2. |

### PCB

| Change | Detail |
|---|---|
| EN reset filter | C12/C14 moved from beside SW1 (~47 mm away) to 4.0/4.4 mm from U1 pin 3. R7 is 1.6 mm from the pin. |
| USB-C position | J13 moved to x = 101.875. Its "PCB edge" line is now flush with Edge.Cuts; it was previously recessed ~0.75 mm. D+/D− rerouted. |
| U5 jumper groups | `jumper_pad_groups` (1,6) and (3,4) are present. **Note:** the stock `SOT-23-6` footprint does not carry these, so updating footprints from the library strips them and brings back two false "unconnected" DRC errors. Consider a project-local copy of the footprint. |
| EPAD vias | 9 × 0.3 mm GND vias, one in each pad of U1's 3×3 EPAD (Espressif land pattern). |
| MISO | The 5 inner-layer MISO segments beside the BMI088 moved from In1 (GND) to In2 (Pwr). The In1 GND plane under U3 is unbroken. |
| SBUS/RC TVS | D6/D7 moved from ~32 mm away to 5.0/4.7 mm from J7 pins 1/4. |
| Fiducials | `REF**` ×3 renamed FID1 (142.25, 82.25), FID2 (118.25, 96.25), FID3 (161.25, 118.75). Their reference text is on F.Fab, not silkscreen. |
| Silkscreen | Board text corrected to "v3.1". |

## Review findings and disposition

Severities are the verifier's corrected ratings.

### Fixed
| Finding | Severity |
|---|---|
| SERVO_5V had no capacitance (receiver brown-out risk) | HIGH → C20 22 µF added. The user's decision was to skip bulk capacitance, because v2.0 drove six servos with no caps (see "Accepted"). |
| Q2 SOT-23 P-FET undersized for servo current | MEDIUM → removed |
| EN RC 47 mm from module | MEDIUM → moved |
| USB-C recessed 0.75 mm | MEDIUM → fixed |
| UWB powered from the IMU rail (SENSOR_3V3) | MEDIUM → moved to FC_5V via ferrite + LDO on the UWB board |
| C18 bias derating / XC6220 stability | LOW → 22 µF 25 V 0805 |
| ESP32 EPAD had no vias | LOW → 9 vias |
| MISO slot in In1 GND under BMI088 | LOW → moved to In2 |
| SBUS/RC TVS far from J7 | LOW → moved |
| I2C pull-ups weak for cabled bus | LOW → 4.7k |
| Q1 redundant with D5 | LOW → removed. J20 pinout made symmetric. |
| J8 LCSC not orderable; R24–R27 missing LCSC | LOW/HIGH-impact → fixed |
| R24/R25 1206 overloaded (0.33 W) by a shorted LED cable | LOW → 0.75 W part (C5759775) |
| Duplicate `REF**` fiducial designators | LOW → FID1–3 |
| ESP32 title said N16-R8 | INFO → fixed |

### Accepted / not changed (with rationale)
| Finding | Rationale |
|---|---|
| No bulk cap on SERVO_5V | The v2.0 board ran six servos with no caps. One 22 µF ceramic was added as a compromise. If receiver brown-outs or failsafes appear (check receiver telemetry: RX voltage, frame loss), add a 220 µF polymer (e.g. C22395163, 6.3 × 5 mm). |
| No pull-ups on the 5 sensor-bus CS lines | No board space. Handled in firmware (see "Firmware to-do"). |
| UWB shares the sensor SPI bus; no series resistors on J10 | Accepted. The ESP32-S3 has only two user SPI hosts, and the SD card uses the other one. Power is isolated (ferrite + separate LDO). |
| J14 GPS pinout isn't Pixhawk DS-009 | Not a defect. J14 (SDA, GND, RX, TX, 5V, SCL) matches the RDQ BN-880 module ("SDA, GND, TX, RX, VCC, SCL") with TX/RX already crossed, so a straight-through cable works. |
| No TVS on FC_5V | Skipped. The LDOs' absolute max is 6.0–6.5 V. Relies on a correctly set UBEC and D5. Hot-plug ringing risk is low because the servo lead stays plugged in and the BEC ramps with the ESC. |
| No 10 µF at microSD; C13 is ~7 mm from Card1 VDD | C13 cannot move closer. C10 (22 µF) is on ESP_3V3. Revisit only if SD write errors or brown-outs appear. |
| No 100 nF at U5 pin 5 | No space. C17 (10 µF) is ~5 mm away on the same net; ST lists the cap as best practice only. |
| Inner-zone clearance 0.5 mm (via anti-pads merge into slots beside U1) | Not changed. Optional improvement: 0.2–0.25 mm, then refill. |
| J13 A1/B12 starved thermal (DRC) | Accepted by the user. The pads are connected to GND, just with fewer spokes than the rule requires. |
| No battery voltage/current sense | No free ADC1 pin. Plan an I2C power monitor (e.g. INA226-based module) on the existing bus. |
| ESC_OUT1/2 no pull-downs; ICM-42688 FSYNC NC; BMI088 INT pins NC | INFO/LOW. ESCs need a valid pulse train to arm. INT2 defaults to an open-drain output. The BMI088 is polled. |
| SD pins use the GPIO matrix rather than native FSPI IO_MUX | INFO. Fine for SD at ≤20 MHz. |

## Round 2 review (2026-10-03, after all fixes)

**Method.** Four new blind reviewers (two schematic, two PCB) worked without access to round 1 or to this document. They raised 32 claims, and an independent verifier checked each one against the netlist, the PCB geometry, the datasheets and the LCSC/JLC pages. Result: no CRITICAL or HIGH issues.

**Fixed in round 2**

| Item | Fix |
|---|---|
| J9 (IR LED connector) had no LCSC number | **C161691** (BM03B-GHS-TBT), the same part as J11. It matches the footprint exactly. |
| D5 Notes field said "ESC1 UBEC diode" | Changed to "FC UBEC input diode (J20 -> FC_5V)" |
| TP1–TP10 appeared in the BOM and placement file with no part | Excluded from BOM and position files, in both schematic and PCB |
| J19 pin 2 (NC) labelled "5V" on silk | Relabelled "NC" |
| "91mm" / "61.5mm" text would print on the board | Moved from F.Silkscreen to Dwgs.User |
| 32 silk texts had 0.1 mm stroke | Raised to 0.15 mm. The silk-overlap and silk-over-copper counts did not change. |
| Mounting holes all named `M2` | Renamed H1–H3 (owner) |

**Cross-board item, fixed on the mission board.** FC J11 is 1 GND / 2 TX / 3 RX, while the mission-board J18 was 1 TX / 2 RX / 3 GND, and its description calls for a straight-through cable. With a 1:1 cable, the mission-board TX would be driven into FC GND and FC RX would be held low. The mission-board J18 is now 1 GND / 2 RX / 3 TX (confirmed in its netlist on 2026-10-08), so a 1:1 cable crosses TX and RX correctly.

**Sourcing: resolved.** The jlcsearch mirror of JLC's library proved incomplete and stale. It reported C25531 as missing, C161692 at 2 in stock and C378970 at 162. The owner checked JLC's own parts pages on 2026-10-03 and found C25531 present, **C161692 (J15) at ~14k** and **C378970 (J10) at ~23k**. Do not rely on jlcsearch for stock. DPS368 (C3232508) was reported at ~285; confirm it on JLC as well.

**Should fix (cheap; layout or placement work, left to the owner)**
- **ICM-42688 U2 pin 9 (FSYNC):** the datasheet says to connect it to GND if unused. Floating is harmless by default: at reset it is INT2, open-drain, active-low. To fix, tie pins 9, 10 and 11 to GND with an F.Cu stub. **Pad 8, next to pin 9, is SENSOR_3V3. Do not bridge it.**
- **ESC_OUT1/ESC_OUT2:** no pull-downs, so the outputs float during boot. Fix: 100k (C25741) to GND on the connector side of R22/R23.
- **C19 → C15850** (10 µF 25 V 0805). This is optional, because C10 (22 µF, 5.9 mm from U4) already gives enough output capacitance.
- **D3/D5 → DSS24 (C2923950, SOD-123FL, 2 A).** This adds margin on FC_5V; the peak load is ~0.8 A against the B5819W's 1 A rating. It is nearly a drop-in on the SOD-123 land.
- **Servo rail feed:** add 4–6 vias at J22 (there are 5 now), and order **1 oz inner copper**. JLC's default inner copper is 0.5 oz. A solid zone connection on the servo-header 5 V pins would help current capacity but makes hand-soldering harder.
- **GND island near Card1:** the F.Cu GND island around Card1 and the 330 Ω resistors (~43.5 mm²) has a single stitching via. Add 2–3 more.
- **Plane-split crossings (optional):** B.Cu servo/SBUS traces cross the Pwr.Cu FC_5V/SERVO_5V split. This is an EMI concern only. Move the crossing segments to F.Cu.
- **"Power Plane Split" silk:** the hatch line at x = 164.5 runs over the D7 pads. It is cosmetic, because the fab clips silk on pads.

**Re-confirmed as accepted.** The facts match the owner's assumptions for all of these: CS pull-ups, microSD bulk cap, FC_5V TVS, MISO on Pwr.Cu, the servo bulk cap, the I2C module pull-up meter check, and the J13 starved thermal. The starved thermal is a KiCad DRC error only and does not block fab. U4's thermal estimate is Tj ≈ 85 °C at 40 °C ambient against a 125 °C limit, which is adequate.

**Accepted failure mode to record.** The UWB module shares the IMU/barometer SPI bus through an off-board cable, with no series resistors and no ESD protection. A harness fault (MISO shorted, or a stuck UWB driving MISO) takes out **both IMUs and the barometer at once**.

**Other notes**
- **R1 back-power:** if the receiver is powered (SERVO_5V) while FC_5V is off, it back-powers the ESP32 at ~8 mA through R26/R27 and the clamp diodes. Benign.
- **R2 cables:** the LIDAR model is still unknown. Benewake parts order SDA/SCL differently from J8's Pixhawk order, so build the cable to suit. The BN-880 may ship with a 1.0 mm SH lead, which would need a GH adapter.

Minor, still open:
- TP1/TP9 "non-mirrored text" warnings are on B.Fab, which is not printed. Harmless.
- J7 ERC library-symbol mismatch. Cosmetic.

## Cross-board dependency: UWB radio

J10 pin 1 now supplies **5 V** (FC_5V via L3). The DWM3000 is a 3.3 V-only part. The UWB board (`../uwb-radio-kicad/`) **must** have its own 3.3 V LDO (e.g. AP2112K-3.3, 1 µF in/out) plus 10 µF + 100 nF at the module VDD pins. Do not connect an older UWB board without that LDO; it would destroy the DWM3000. As of 2026-10-08, the UWB schematic has its LDO drawn in: CN1 pin 1 (5V) feeds U2 (XC6220B331PR-G), whose 3.3V output supplies the DWM3000. Verify that board's decoupling before ordering both. Also consider labelling J10/CN1 pin 1 "5V" on both silkscreens.

## Firmware to-do for bring-up

- **PSRAM:** `sdkconfig` set for **2 MB quad** PSRAM (N16R2), not octal.
- **SD card:** SPI on GPIO35 (SCK), 36 (MOSI), 37 (CS), 8 (MISO), on its own SPI host.
- **Sensor-bus chip selects:** at the very start of `app_main`, before any SPI init, drive all five CS lines (IMU1_CS, IMU2_ACCEL_CS, IMU2_GYRO_CS, BAROMETER_CS, UWB_CS) as outputs high, with internal pull-ups enabled.
- **IMUs:**
  - BMI088: the accelerometer starts in I2C mode. Do a dummy read on CSB1 (e.g. ACC_CHIP_ID) to switch it to SPI.
  - ICM-42688-P: set `UI_SIFS_CFG = 11` to disable its I2C interface.
- **Barometer:** DPS368 supports SPI mode 3 only.
- **UWB:** use a slow SPI clock for DWM3000 init, before its PLL locks. The bus speed has to switch per device.
- **UART0 / RC link:** the RC link is on UART0 (GPIO43/44), so the ROM boot log is sent to the receiver. Suppress it (eFuse/GPIO46), or move the console to USB-Serial-JTAG.
- **SBUS:** enable UART RX inversion.
- **GPS (BN-880):** default 38400 baud. Detect the compass chip at bring-up (HMC5883L vs QMC5883L clone). Start I2C at 100 kHz.

## Ordering notes (JLCPCB)

- **Panel rails:** the ESP32 antenna overhangs the top edge by ~6.2 mm, by design. Request rails on the other edges only.
- **Finish:** ENIG recommended, for the 0.5 mm-pitch LGA sensors. `copper_finish` is currently unset in the project.
- **Stackup:** the file declares 0.1 mm prepreg with 35 µm inner copper. Select JLC's standard 4-layer stackup when ordering. Only the short USB pair is impedance-sensitive, and only at full speed.
- **Placement file:** check polarity and rotation in JLC's CPL preview, particularly for the polarized/oriented parts (diodes, LEDs, USBLC6, LGA sensors).
- **Hand-soldered parts:** J1–J7, J19, J20 and J22 are DNP through-hole headers. Check that standard JR/Futaba servo plugs physically seat in the KK-style footprints.

## Datasheet folder housekeeping

- `datasheets/imu_DPS368_datasheet.pdf` is actually the **BMI088** datasheet. Rename it.
- There is no local XC6220 (U4) datasheet. `C51118_…pdf` is a duplicate of the AP2112K datasheet (U6's LCSC part).

## Review limits

- The kicad-happy EMC, thermal and SPICE analyzers were not re-run on the final state. The earlier pass's results (thermal U6 Tj ≈ 41 °C; U4 manual estimate ≈ 69–80 °C worst case; SPICE subcircuits passing) predate this round. They did include Q1/Q2, which are now removed.
- No component lifecycle audit (no distributor API keys).
- No gerber or drill review. Re-export the fab package from the final board.
- External module behaviour was not verified: receiver signal voltage, BN-880 pull-up voltage (it should be 3.3 V, not 5 V; check with a meter), LIDAR minimum supply voltage (FC_5V is ~4.5–4.7 V after the Schottky).
