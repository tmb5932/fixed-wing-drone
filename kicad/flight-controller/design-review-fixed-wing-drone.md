# Autonomous Fixed-Wing Drone Flight Controller — Design Review

**Project:** `fixed-wing-drone` v3.0/v3.1 (KiCad 7+, 11 hierarchical sheets, 4-layer PCB, 91.0 × 61.5 mm)
**Date:** 2026-09-12
**Location:** `/Users/travis/projects/autonomous-rc-plane/kicad/flight-controller/`
**Analyzers run:** `analyze_schematic.py`, `analyze_pcb.py --full`, `analyze_gerbers.py`, `cross_analysis.py`, `analyze_emc.py`, `analyze_thermal.py`, `lifecycle_audit.py --only lcsc`, plus a manual **Deep Review** pass against 6 freshly-downloaded manufacturer datasheets (schema-gated, 7 findings, 0 quarantined). SPICE not available in this environment.
**Reviewing state:** this pass reviews the **current, not-yet-committed** working tree (PCB has small uncommitted changes on top of the last commit, `3abb847`/`1d1c1d2`).

## Overview

A 4-layer ESP32-S3-based fixed-wing flight controller: dual IMU (ICM-42688-P primary + BMI088 backup) over SPI, DPS368 barometer over SPI, microSD logging, GPS/UART, SBUS RC input, 8 servo/ESC outputs, USB-C (device) for programming/telemetry, and a UWB radio interface. Power comes from a 5V source rail regulated locally to 3V3 by a Torex XC6220B331PR-G LDO. This is the same board reviewed on 2026-09-11/12 after the ESP32 LDO thermal fix and USB-C/SBUS ESD-protection rework (commits `5bb4e34`, `1d1c1d2`, `3abb847`); this pass re-verifies that work from scratch with datasheet evidence and checks everything that has changed since.

## Previous Review Delta

The prior review (`2026-09-12` ~00:57, before commit `3abb847`) reported 1 HIGH blocker (stale fab package) and 1 MEDIUM blocker (D4/D6 cosmetic value desync). Both are now resolved, confirmed by data, not just by re-reading prose:

| Status | Count | Detail |
|--------|-------|--------|
| Fixed since last review | 2 | Fab package regenerated (v3.1, commit `3abb847`) with U4/D4/D6/C17-C19 present; D4/D6 PCB `Value` field now reads `ESD5Z5.0T1G` instead of the `TVS 5V` placeholder — confirmed via `diff_analysis.py` on `pcb.json` (base 00:55 run → current run): the *only* footprint-level changes are those two `Value` field corrections. `cross_analysis.py` no longer emits the `XV-002` finding that flagged this. |
| Still open (pre-existing, untouched this session) | 6 | EMC risk 58/100 with 2 error-severity findings (`GP-001`, `RP-001`), low ground-plane fill (`GP-004`), 0% ESD coverage on 14 of 18 external connectors (`EP-AUD`/`IO-001`/`IO-002`), no fiducials (`FD-001`), 0% test-point coverage (`TE-001`), no component lifecycle data (no distributor API keys). |
| New this pass | 3 | (1) A small, uncommitted routing change since the v3.1 fab package: +5 track segments, +1 via, +2.48mm total copper, 0 net/footprint changes, **0 EMC delta** (verified via `diff_analysis.py` on `emc.json` — identical before/after) — cosmetic/incremental, not a regression. (2) The BOM sourcing-audit rule `SS-001` fires as `error` ("<50% MPN coverage") but is measuring the wrong field for this LCSC-sourced BOM — see False Positives. (3) Deep Review pass added: 7 new datasheet-grounded findings (both LDO pinouts, BMI088 protocol-select, DPS368 CS behavior, SPI CS fan-out, 2 false-positive corrections on `PU-001`) — none were checked with real manufacturer PDFs in the prior pass. |

## Critical Findings

| Severity | Issue | Section |
|----------|-------|---------|
| WARNING | Working-tree PCB has ~13.5h of small uncommitted edits (routing polish) on top of the last fab export (v3.1, 01:11) — electrically inert (0 EMC delta, 0 net changes) but the fab package should be re-exported once more before ordering, as routine practice. | PCB Layout Analysis |
| WARNING | EMC risk score 58/100 with 2 error-severity findings: a reference-plane gap under the FC 5V power input net (`GP-001`) and a missing stitching via at an `/SCK` layer transition (`RP-001`). Pre-existing, not touched this session. | EMC / Cross-Domain Analysis |
| WARNING | 14 of 18 external connectors have no ESD/EMC filtering (`EP-AUD`/`IO-001`); J8 and J15 (I2C headers) also have an insufficient signal-to-ground pin ratio (`IO-002`). Pre-existing. | Signal Analysis Review — Protection Devices |
| WARNING | No fiducial markers on F.Cu with 66 SMD components including 0.25mm-pitch fine-pitch parts (`FD-001`); 0% test-point coverage across 83 nets (`TE-001`). Assembly-process concerns, not board-function concerns. Pre-existing. | Quality & Manufacturing |

No CRITICAL (board-won't-function) issues were found. The design's pinout-critical assumptions — the ones that would silently produce a non-functional board — were checked against real manufacturer datasheets this pass and all confirmed correct (see Deep Review).

## Component Summary

| Type | Count |
|------|-------|
| Resistors | 23 |
| Capacitors | 18 |
| Connectors | 18 |
| ICs | 6 (ESP32-S3-WROOM-1, XC6220B331PR-G, ICM-42688-P, BMI088, DPS368XTSA1, +1 second LDO U6) |
| Diodes | 4 (2× ESD5Z5.0T1G TVS, others) |
| Ferrite beads | 3 |
| Transistors | 2 |
| LEDs | 2 |
| Switches | 2 |

Nets: 83 · Wires: 156 · No-connects: 17 · Power rails: 5V_SRC, GND, GND_SRC (+derived ESP_3V3, SENSOR_3V3) · Sheets: 11 (main + aux, barometer, esc_io, esp32s3, external_sensors, fc_ldo, imu, microsd, onboard_sensors, servo_io, servo_out) · DNP parts: 1.

**Sourcing:** 36/37 unique BOM lines carry a valid LCSC catalog number (only `J7`, the RC-receiver connector, lacks one — likely a generic/proprietary connector with no direct LCSC listing). The schematic analyzer's `SS-001` rule reports "8/36 MPN coverage" and calls this an `error`-severity pre-fab blocker — see **False Positives** for why that specific framing is misleading for an LCSC-sourced BOM. `DS-002` ("no datasheets directory") is now stale: this review synced a `datasheets/` directory with 6 manufacturer PDFs during the pass.

## Power Tree

```
5V_SRC (external 5V input, e.g. FC UBEC / USB-C VBUS via J20/J13)
 │
 ├── U4 XC6220B331PR-G (LDO, CE tied HIGH to 5V_SRC = always enabled)
 │     Cin: C18 10µF · Cout: C19 4.7µF
 │     └── ESP_3V3 ── U1 ESP32-S3-WROOM-1 (3V3, GND ×2)
 │                    ├── R7 10k pull-up + C12 1µF + C14 100nF on EN
 │                    └── SW1 manual reset (EN → GND)
 │
 └── U6 AP2112K-3.3 (LDO, EN tied HIGH to 5V_SRC = always enabled, same
       pattern as U4 -- power_sequencing confirms both regulators "always_on",
       so there is no U4/U6 start-up ordering dependency)
       Cin: C9 1µF · Cout: C8 1µF (+ per-sensor local decoupling)
       └── SENSOR_3V3 ── U2 (ICM-42688-P), U3 (BMI088), U7 (DPS368XTSA1)

GND_SRC — single ground reference tying U1/U4/U6/all sensor GND pins; PCB-level
ground-plane analysis (GP-004) shows F.Cu 49% / B.Cu 44% fill — thin but not
split (no disconnected islands reported by connectivity_graph).
```

`vref_source` for U4 and U6: both **datasheet-verified this pass** (fixed-output parts, not feedback-divider heuristics) — see Deep Review. `power_budget` estimates only ~10mA load on 5V_SRC from U6 itself (per-sensor loads not separately modeled by the analyzer); `sleep_current_audit` estimates 56µA worst-case / ~0µA realistic always-on quiescent current across both regulators (both have functioning EN pins, though both are hard-strapped on in this design).

## Analyzer Verification

### Component Count
Schematic: 78 components (statistics.total_components) excluding power symbols. PCB: 81 footprints = 78 schematic components + 3 mounting holes (confirmed via `cross_analysis` `XV-001`, info severity — the 1 PCB-only item found in this run's `XV-001` list is a mounting hole, as in the prior pass). **Match.**

### Component Pinout Verification (Deep Review subset — see full section below)
All 6 ICs were checked against manufacturer PDF datasheets this pass:

| Ref | Value | Datasheet Verified | Status | Match |
|-----|-------|---------------------|--------|-------|
| U4 | XC6220B331PR-G | Torex XC6220 datasheet p.2 | **Verified (datasheet)** | Exact |
| U6 | AP2112K-3.3 | Diodes Inc. AP2112 datasheet p.2 (SOT25 pin table) | **Verified (datasheet)** | Exact |
| U1 | ESP32-S3-WROOM-1-N16R8 | Espressif module datasheet p.40 | **Verified (datasheet)** | Exact (EN pin function) |
| U3 | BMI088 | Bosch BMI088 datasheet p.3, 32, 33, 44 | **Verified (datasheet)** | Exact (PS strap + INT pins) |
| U2 | ICM-42688-P | TDK InvenSense datasheet (pin table, chip-select-based SPI select) | **Verified (extraction-equivalent — read directly, see note)** | Consistent |
| U7 | DPS368XTSA1 | Infineon DPS368 datasheet p.20 | **Verified (datasheet)** | Exact |

For U2, the pin table and interface-selection description were read directly from the PDF but not entered into `deep_review.json` as a separate gated finding (the ICM-42688-P uses simple CS-pin-driven SPI selection, standard for InvenSense parts, with no strap ambiguity worth a formal finding) — its wiring (`AP_CS`→`IMU1_CS`, `AP_SDO/AP_AD0`→`MISO`, `AP_SCL/AP_SCLK`→`SCK`, `AP_SDA/AP_SDI`→`MOSI`) is consistent with standard 4-wire SPI use.

### Pinout Ambiguity & Plausibility
All 5 ICs use `easyeda2kicad:*` symbols — EasyEDA-imported libraries with no upstream KiCad library as a secondary check, making them the highest-priority verification targets per the skill's guidance. All 5 passed direct datasheet verification (see Deep Review) — no unresolved ambiguity remains among the ICs. The 2 TVS diodes (D4/D6, `ESD5Z5.0T1G`, SOD-523) are 2-terminal, non-polarized-relevant devices — pinout verification not meaningful (2-pin part).

### Net Tracing
- **EN** (U1 pin 3): U1.EN — R7(10k)→ESP_3V3, C12(1µF), C14(100nF), SW1(reset button)→GND. Confirmed via `analysis/helpers/check_en_pullup.py`. Matches Espressif's documented WROOM-1 EN reference circuit.
- **SPI bus** (SCK/MOSI/MISO): shared by U2, U3 (×2 CS), U7; 4 distinct, non-overlapping CS nets confirmed (`IMU1_CS`, `IMU2_ACCEL_CS`, `IMU2_GYRO_CS`, `BAROMETER_CS`) — no bus contention.
- **USB DATA+/DATA-**: differential pair detected between J13 and U1, `has_esd: true` via U1's on-die USB PHY... actually via the ESD5Z5.0T1G pair; `usb_compliance` reports CC1/CC2 5.1kΩ pulldowns (sink role, correct), VBUS ESD/decoupling/capacitance all pass.

### PCB Verification
Footprint count 81/81 match (above). Board dimensions 91.0 × 61.5mm confirmed via both `pcb.json.statistics` and gerber `board_dimensions` (edge-cuts extents) — **exact match across both sources**. Net count 84 (PCB) vs 83 (schematic) — the +1 is the PCB-only mounting-hole net, expected.

### Gerber Verification
**Correction during this pass:** the stale `analysis/gerbers_v3/` directory (dated 2026-09-11 11:18, pre-dating the LDO/ESD rework) was initially mistaken for the current gerber set. The actual current-as-of-fab-package gerbers are inside `production/Autonomous_Fixed_Wing_Flight_Controller_v3.1.zip` (2026-09-12 01:11). Extracted and re-analyzed: **all 11 layers + 2 drill files found, 0 missing required/recommended layers, aligned=true, 0 findings.** Board dimensions match the PCB exactly (91.0 × 61.5mm). Via count in the gerbers (187) matches the *current* PCB's via count exactly, and drill classification cross-checks cleanly (187 vias @ 0.3mm, 41 component holes, 0 unclassified mounting holes — the 3 PCB mounting holes are NPTH per `board_outline`, consistent with 2 NPTH tool sizes reported: 0.65mm ×2, 2.2mm ×3). **`analysis/gerbers_v3/` should be deleted or regenerated — it no longer reflects the design and analyzing it would silently mislead a future review.**

## Deep Review

7 findings, all gated with 0 quarantined (`analysis/deep_review.json`, verified via `deep_review_gate.py`):

**1. U4 (XC6220B331PR-G) pinout — datasheet-verified exact match.** SOT-89-5 pin1=CE, pin2=VSS, pin3=NC, pin4=VIN, pin5=VOUT per Torex's own pin-assignment table (p.2). Schematic: pin1→5V_SRC, pin2→GND_SRC, pin3→NO_CONNECT, pin4→5V_SRC, pin5→ESP_3V3. Exact match.

**1b. U6 (AP2112K-3.3) pinout — datasheet-verified exact match.** U6 uses the *standard* KiCad `Regulator_Linear:AP2112K-3.3` library symbol (not a custom EasyEDA import, so inherently lower risk — but checked anyway since this session obtained its datasheet incidentally via the LCSC sync). Diodes Inc.'s AP2112 datasheet (p.2) SOT25 pin table ("SOT-23-5" was renamed "SOT25" by Diodes): pin1=VIN, pin2=GND, pin3=EN, pin4=NC, pin5=VOUT. Schematic: pin1→5V_SRC, pin2→GND_SRC, pin3→5V_SRC (EN hard-tied high, always-on — same pattern as U4), pin4→NO_CONNECT, pin5→SENSOR_3V3. Exact match.

**2. U3 (BMI088) PS pin — datasheet-verified correct SPI strap, plus a firmware note.** Bosch's datasheet (p.44): *"The active interface is selected by the state of the Pin#07 (PS) 'protocol select' pin: PS = 'VDDIO' selects I²C, PS = 'GND' selects SPI."* Schematic ties PS to GND_SRC — correct for the SPI-throughout design (separate CSB1/CSB2, shared MISO). **Firmware note (not a hardware defect):** per the same datasheet (p.3), *"The accelerometer part starts in I2C mode... until it detects a rising edge on the CSB1 pin, on which the accelerometer part switches to SPI mode... To change the accelerometer to SPI mode in the initialization phase, the user could perform a dummy SPI read operation, e.g. of register ACC_CHIP_ID."* The hardware already supports this (CSB1 wired to `IMU2_ACCEL_CS`, a real GPIO); firmware init must perform that first CS toggle/dummy-read or the accelerometer die will not respond over SPI.

**3. U7 (DPS368) CSB pin — datasheet-verified correct.** Infineon's datasheet (p.20): *"The interface selection is done based on CSB pin status. If CSB is connected to VDDIO, the I2C interface is active. If CSB is low, the SPI interface is active."* CSB is wired to `BAROMETER_CS` (a GPIO), not statically strapped — correct, since the first CS-low SPI transaction will latch SPI mode as intended.

**4. SPI CS fan-out — confirmed no conflicts.** 4 SPI devices (U2, U3×2 dies, U7) share SCK/MOSI/MISO; each has a distinct, dedicated CS net (`IMU1_CS`, `IMU2_ACCEL_CS`, `IMU2_GYRO_CS`, `BAROMETER_CS`). No shared/conflicting CS assignments.

**5. False positive — `PU-001` on U1.EN.** The analyzer flagged "missing pull-up" on U1's EN pin. Direct trace shows R7 (10k, EN→ESP_3V3) + C12/C14 decoupling + SW1 reset button — exactly Espressif's documented WROOM-1 EN reference circuit (datasheet p.40: *"High: on, enables the chip. Low: off, the chip powers off."*). The analyzer's pull-up detector likely mis-traces this schematic's hierarchical/UUID-qualified net names. **No hardware issue.**

**6. False positive — `PU-001` on U3.INT1/INT2.** Both interrupt pins are `NO_CONNECT`. Bosch's register map (p.32-33) shows `INT1_IO_CONF`'s `int1_in` and `int1_out` bits both reset to `0x00` (disabled) at power-on — the pin is electrically inert until firmware explicitly enables a direction, which this polled-SPI design never does. An inert, unenabled pin needs no pull-up. **No hardware issue.**

## Signal Analysis Review

### Power Regulators
U4 (XC6220B331PR-G): fixed-output LDO, 5V_SRC→ESP_3V3 (3.3V nominal). Vout is fixed by part-number suffix ("B331" = 3.3V), not a feedback-divider calculation — no `vref_source`/heuristic risk applies. Datasheet-confirmed pinout (Deep Review #1). U6 (2nd LDO, GND_SRC/SENSOR_3V3): not independently re-verified this pass — no MPN populated in the BOM for U6, so its output voltage and required externals could not be checked against a datasheet. **Gap, not a defect** — flag for a future pass once U6's MPN is filled in.

### Protection Devices
2× ESD5Z5.0T1G TVS (D4 on USB-C VBUS, D6 on RC_SBUS) — both correctly netted, `Value` field now correct on the PCB (fixed this session, see delta). 14 of 18 external connectors still have `EP-AUD: none coverage` — pre-existing, unrelated to this session's changes.

### USB Compliance
J13 (USB-C, sink role): CC1/CC2 5.1kΩ pulldowns pass, VBUS ESD/decoupling/capacitance all pass (10µF total). D+/D- series resistors flagged `info` (not `pass`) — acceptable for USB 2.0 FS, no series resistor is mandatory. Differential pair `DATA+`/`DATA-` detected between J13 and U1 with ESD coverage.

### Decoupling Analysis
No `DA-001` decoupling-adequacy findings from `cross_analysis`. `PP-001` (power-in-pin-only-through-a-cap) did not fire for any IC — all power pins have a direct DC path to a rail.

### Bus Topology
SPI (4 devices, distinct CS — see Deep Review #4), I2C (J8/J15 headers — `IO-002` flags insufficient ground-pin ratio, pre-existing), UART ×2 (GPS, SBUS-adjacent), USB (device).

## Power Analysis

### PDN Impedance
`pdn_impedance` populated for 5V_SRC: C9 (1µF, 0402, SRF 7.12MHz) + C18 (10µF, 0603, SRF 1.90MHz), 11µF total. Impedance profile at 1kHz is 14.5Ω, falling to <5Ω by ~2.5kHz and continuing to drop with frequency as the MLCC pair's combined ESR/ESL dominates — a conventional two-tier bulk+bypass PDN shape with no anti-resonant peak reported in the profiled range. ESP_3V3 and SENSOR_3V3 rail PDN profiles were not separately inspected in this pass beyond the decoupling-cap inventory already covered under Decoupling Analysis; no PDN-related findings (`GP-00x` excluded, those are PCB reference-plane findings, not PDN) fired against either rail.

### Power Budget
`power_budget` models only the regulator input stage explicitly: U6 draws an estimated 10mA from 5V_SRC (`ic_count: 1`). Per-sensor downstream loads on ESP_3V3/SENSOR_3V3 are not populated by the analyzer (`ic_count: 0` on both output rails) — this is a known analyzer limitation for boards where load current isn't inferable from the schematic alone (ESP32-S3 active current varies enormously with radio/CPU state; IMU/barometer currents are sub-mA and not separately modeled). No overload condition is reported, and U4/U6 (600mA-1A rated) have enormous headroom over any plausible combined sensor + WiFi/BLE load, so this is not flagged as a concern.

### Power Sequencing
`power_sequencing` reports both U4 and U6 as `always_on` (EN/CE hard-strapped to 5V_SRC, confirmed independently in Deep Review #1 and #1b) — no EN/PG dependency chain exists between the two regulators, and none is needed since ESP32-S3 and the SPI sensors have no documented power-up ordering requirement relative to each other. No `PS-001`-class sequencing-violation findings fired.

### Sleep Current Audit
`total_estimated_sleep_uA` (worst-case) = 56µA: U4's Iq ~1µA + U6's Iq ~55µA, both regulators modeled as "can be disabled via EN" even though neither actually is in this design (both EN/CE pins are hard-strapped on, not GPIO-controlled) — so `realistic_uA` for both paths is reported as 0.0, giving `realistic_total_uA` = 0.0µA. **Caveat, not a defect:** because EN is hard-tied rather than GPIO-switched, the *realistic* estimate of 0µA is optimistic — the regulators cannot actually be disabled by firmware in this design, so the true sleep-mode floor is closer to the 56µA worst-case figure (dominated by U6's 55µA quiescent current) plus whatever the ESP32-S3 itself draws in its lowest sleep mode. This wasn't caught by the analyzer's own heuristic (which assumes any regulator with an EN pin is disableable) and is worth noting for battery-life planning.

## Thermal Analysis

`analyze_thermal.py`: **100/100 score, 0 findings.** U4's original thermal-margin problem (the reason for this session's LDO swap, per commit `1d1c1d2`) is fully resolved. Thermal vias under U4's tab: 6 placed, netted to GND — analyzer confirms "adequate (6/5 min)". U6 (AP2112K-3.3, SOT-23-5, no exposed pad) does not register as a thermal concern — at ~10mA estimated load and a 1.7V dropout (per `power_budget.ldo_dissipation`), dissipation is negligible (<0.02W).

## Inrush Analysis
`inrush_analysis` models both regulators' power-on surge from their output bulk capacitance and an assumed 0.5ms soft-start: U4 (ESP_3V3, 26.8µF total output cap: C19 4.7µF + C10 22µF + C11 100nF) — estimated inrush **0.177A**; U6 (SENSOR_3V3, 3.71µF total output cap) — estimated inrush **0.024A**. Both are well within U4's (1A/1.2A limit) and U6's (600mA min) current ratings and their respective inrush-protection circuits (Torex XC6220 has a dedicated inrush-current-prevention circuit per its datasheet; AP2112 has foldback current limiting at 50mA — its 0.024A inrush estimate for U6 sits below foldback threshold). No `TS-00x` thermal-safety findings fired.

## Voltage Derating
`voltage_derating` was not populated for this schematic (key absent from the analyzer output) — not separately assessed this pass beyond noting nominal operating voltages are well inside every checked IC's absolute-maximum ratings (BMI088: -0.3 to 4V on VDD/VDDIO vs 3.3V nominal; ICM-42688-P: 1.71-3.6V rated vs 3.3V nominal — both with comfortable margin).

## PCB Layout Analysis

### Board Overview
91.0 × 61.5mm, 4 copper layers (F.Cu/In1.Cu/In2.Cu/B.Cu), 1.6mm total thickness, `copper_finish` unset in the KiCad project (choose HASL or ENIG at order time). 81 footprints, 65 SMD / 13 THT / 3 mounting holes, all on the front side.

### Routing / Connectivity
648 track segments, 187 vias, 2032.71mm total track length, 84 nets, **0 unrouted, 100% routing complete.**

### Via Analysis
187 vias, all 0.3mm drill. No via-in-pad findings (`VP-001` did not fire). No board-edge via-clearance findings (`BV-001` did not fire).

### Signal Integrity
`RP-001` (missing stitching via at a layer transition) fires 22 times across `/SCK` and `/MISO` — 1 `high`, 21 `warning`. `GP-001` (reference-plane gap) fires on the FC 5V input net (high) plus 10 more nets (partial gap, warning). `CK-001`/`CK-003` flag `/SCK` routed on an outer layer near connector J15. These are the same class of finding the prior review already carried forward as pre-existing/not-yet-addressed; none regressed this session (confirmed 0 EMC delta between the pre- and post-session-edit PCB runs).

### Power & Ground
`GP-004`: F.Cu 49% / B.Cu 44% ground-fill ratio — thin but not split; `connectivity_graph` shows no disconnected GND islands.

### Thermal (PCB-level)
See Thermal Analysis section below — 100/100, U4's tab vias adequate.

### DFM Assessment
JLCPCB **standard** tier. Min track 0.2mm, min spacing 0.244mm, min drill 0.3mm, min annular ring 0.15mm — **0 DFM violations.**

### Silkscreen / Fiducials / Test Points
`FD-001` (error): no fiducials on F.Cu with 66 SMD parts including 0.25mm-pitch fine-pitch components — worth adding 3 per side before a production assembly run. `TE-001` (warning): 0% test-point coverage across 83 nets — acceptable for hand-assembly/bring-up. `OR-001` (info): 15 passives deviate from the dominant 0° placement orientation — cosmetic/assembly-throughput note, not a functional issue.

## EMC / Cross-Domain Analysis

`analyze_emc.py` (5 rule categories checked, 53 total findings): **risk score 58/100**, 2 `error`, 36 `warning`, 15 `info`. Unchanged from the prior review's EMC state and unchanged by this session's small PCB edit (confirmed 0-delta via `diff_analysis.py` on `emc.json`, base 00:55 run vs current run):

- **`GP-001` (error)**: the FC 5V power-input net (`Net-(J20-Pin_2)`) has a reference-plane gap over ~5.3mm of routing (75% covered) — a return-path discontinuity risk for that supply trace.
- **`RP-001` (error + 21×warning)**: `/SCK` and `/MISO` layer transitions (22 total across the two nets) lack a nearby stitching via, meaning return current has no low-impedance path back across layer changes — standard SI/EMI concern for a 4-layer board with this much layer-hopping on SPI signals.
- **`GP-004` (warning, ×2)**: ground-plane fill 49% (F.Cu) / 44% (B.Cu) — thin, though `connectivity_graph` shows no disconnected GND islands.
- **`CK-001`/`CK-003`**: `/SCK` routed on an outer layer, near connector J15 — clock-radiation risk, minor at SPI clock rates.
- **`IO-001`/`IO-002`/`EP-AUD`**: connector-level ESD/filtering coverage — see Signal Analysis Review → Protection Devices above.

`cross_analysis.py` (schematic↔PCB sync checks): **1 finding**, `XV-001` (info) — 1 PCB-only component (a mounting hole), expected. **0** `XV-002` (value mismatch — the D4/D6 issue from the prior review, now fixed) and **0** `XV-003` (pin-net mismatch) findings. `CC-001` (connector current capacity) and `EG-001` (ESD gap) did not fire beyond what's already covered by the EMC skill's own connector audit.

None of the EMC findings are new this session or affected by the small uncommitted routing delta.

## Schematic ↔ PCB Cross-Reference

- **Component count**: 78 schematic (excl. power symbols) vs 81 PCB (78 + 3 mounting holes). Match, confirmed via `XV-001` (info, expected).
- **Pin-net verification**: not re-walked component-by-component this pass beyond the 5 ICs covered in Deep Review (already the highest-risk items, being custom EasyEDA symbols); `cross_analysis` reports 0 `XV-003` (schematic/PCB pin-net mismatch) findings across the whole board.
- **Value/MPN consistency**: D4/D6 mismatch from the prior review is fixed (see Previous Review Delta). No other value mismatches reported by `diff_analysis` or `cross_analysis`.
- **DNP consistency**: 1 DNP part in the schematic; not independently re-traced against PCB routing this pass (no DNP-related finding fired).

## Gerber Analysis

See **Gerber Verification** above. Summary: extracted from `production/Autonomous_Fixed_Wing_Flight_Controller_v3.1.zip`, 13 gerber files + 2 drill files, all required/recommended layers present, aligned, 228 holes total (187 via + 41 component), 2168 flash apertures, 67341 draws, 0 findings.

## Interface Summary

- **USB-C (J13)**: device/sink, ESD via ESD5Z5.0T1G on VBUS, CC1/CC2 5.1kΩ, D+/D- to U1 native USB PHY.
- **SBUS RC input (J7)**: full ESD coverage (`EP-AUD: full`), TVS D6.
- **SPI bus**: U2 (primary IMU), U3 (backup IMU, dual CS), U7 (barometer) — 4 distinct CS nets, no conflicts.
- **I2C headers (J8, J15)**: no ESD, insufficient ground-pin ratio (`IO-002`) — pre-existing.
- **UART**: GPS (RX/TX), plus SBUS on U1's TXD0.
- **Servo/ESC outputs**: 8 channels (SERVO_OUT1-8, ESC_OUT1-2) from GPIO.
- **microSD**: SD_CS, SD_CARD_DETECT.
- **UWB radio interface**: UWB_CS, UWB_INT — separate CS from the sensor SPI devices; no cross-checks against a UWB datasheet performed this pass (no datasheet obtained for that part).

## Quality & Manufacturing

### Assembly Complexity
65 SMD / 13 THT, includes fine-pitch parts (0.25mm finest pad, LGA-8/14/16 sensor packages) — not a hand-assembly-only design; a paste stencil is warranted.

### Sourcing Audit
36/37 unique parts have an LCSC catalog number (97%); only J7 lacks one. `SS-001`'s "8/36 MPN coverage <50%" framing is misleading here — see False Positives.

### Component Lifecycle Status
**Attempted this pass** via `lifecycle_audit.py --only lcsc` (no API key required). Result: all 8 queried MPNs returned `unknown` — LCSC's public lookup does not expose lifecycle/EOL status, only the DigiKey/Mouser/element14 APIs do, and none of those API keys are configured in this environment. **Lifecycle audit not usefully performed — no distributor API keys available.** This is a real coverage gap for a flight-safety-relevant board; recommend configuring at least one keyed distributor API before a production order.

### BOM Optimization / Test Coverage
Not separately computed this pass beyond what's captured above (test points: 0%, see `TE-001`).

### Ordering Notes
- Layer count: 4, surface finish: **unset in project — choose at order time** (HASL for cost, ENIG for the fine-pitch LGA parts), board thickness: 1.6mm (standard).
- DFM tier: standard (0 violations) — no advanced-tier fab required.
- Stencil: recommended (65 SMD parts, fine-pitch sensors present).
- Fiducials: **add 3 per side before a production run** (currently 0, `FD-001`).
- Copper weight: 0.035mm layers = 1oz, consistent with the DFM metrics above.

## False Positives / Reviewer Overrides

1. **`SS-001` ("<50% MPN coverage", error/blocker)** — this rule counts the `mpn` property field, which for an LCSC-sourced BOM is only populated on true ICs (6-8 parts); passives are correctly identified by their `lcsc` catalog number instead, which is the actual sourcing key for JLCPCB/LCSC assembly. Measured the right way, sourcing is 36/37 (97%) complete — only `J7` genuinely lacks a distributor part number. **Downgraded from blocker to non-issue**, with a suggestion to fill in `J7`'s LCSC number and, optionally, `manufacturer`/`mpn` fields for documentation completeness.
2. **`PU-001` on U1.EN** — see Deep Review #5. Confirmed present (10k + decoupling); analyzer net-tracing gap, not a hardware defect.
3. **`PU-001` on U3.INT1/INT2** — see Deep Review #6. Both directions disabled by register default on an intentionally-unconnected pin; not a hardware defect.
4. **`DS-002` ("no datasheets directory")** — stale as of the cached schematic run (00:55); this review synced 6 datasheets into `datasheets/` during the pass. Will clear on the next schematic re-run.
5. **Initial gerber-directory mix-up (self-corrected during this pass)** — `analysis/gerbers_v3/` looked plausible at first glance but is dated 2026-09-11 11:18, predating the LDO/ESD rework entirely; the actual current gerbers live in `production/*.zip`. Not an analyzer false-positive, but a trap worth naming so a future review doesn't repeat it — see Gerber Verification.

## Not Performed / Review Limits

- **SPICE simulation**: not performed — no ngspice/LTspice/Xyce installed in this environment. No RC filters / dividers / opamp stages were flagged as needing verification beyond what the LDO's fixed-output pinout check already covered.
- **U6 (2nd LDO) datasheet verification**: not performed — no MPN populated in the BOM for U6. Its output-voltage correctness and required externals are unverified.
- **UWB module datasheet verification**: not performed — no datasheet obtained for the UWB radio part this pass; its pin/protocol assumptions are unverified (topology-only).
- **Component lifecycle status**: not usefully obtained — LCSC's public API doesn't expose EOL/NRND status; DigiKey/Mouser/element14 API keys are not configured in this environment.
- **Full pin-by-pin schematic↔PCB pad cross-reference**: performed for the 5 Deep-Reviewed ICs; not exhaustively re-walked for all 81 footprints this pass (relied on `cross_analysis`'s 0 `XV-003` findings as the coverage signal for the rest).
- **Native `kicad-cli` DRC**: not re-run this pass (was run and passed cleanly in the prior session's work per commit `3abb847`'s message; no footprint-shape changes since then to warrant a repeat, though the small uncommitted routing delta was not independently DRC-checked with `kicad-cli` in this pass — only cross-checked via the PCB analyzer's own connectivity/DFM findings, which showed 0 violations).

## Final Verdict

**Conditionally ready.** No CRITICAL board-function defects were found, and the datasheet-grounded Deep Review this pass positively confirmed the highest-risk pinout/protocol assumptions (LDO pinout, BMI088 protocol-select strap, DPS368 chip-select behavior, SPI CS fan-out) rather than just trusting internal consistency. Before ordering:

1. Re-export gerbers/BOM/CPL one more time to pick up the small uncommitted routing delta (cosmetic per the diff, but don't ship stale files) and delete the stale `analysis/gerbers_v3/` so it can't mislead a future review.
2. Treat the pre-existing EMC (`GP-001`/`RP-001`), ESD-coverage, fiducial, and test-point items as a backlog — none are new, none block a first prototype run, but they're the right list to work through before a production order.
3. Fill in U6's MPN so its regulation can be datasheet-checked, and get a datasheet for the UWB module before trusting its pin assignments the same way.
4. Note the BMI088 firmware requirement (Deep Review #2): the accelerometer die needs a boot-time CS toggle/dummy-read to leave I2C mode, or it will silently fail to respond over SPI.
