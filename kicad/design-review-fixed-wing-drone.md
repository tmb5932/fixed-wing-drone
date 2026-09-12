# Design Review — Autonomous Fixed-Wing Drone Flight Controller

**Board:** `fixed-wing-drone` v3.0 (4-layer, 91.0 × 61.5 mm, 78 footprints)
**Review date:** 2026-09-11
**Tooling:** kicad-happy analyzer suite (`kicad`, `emc` skills) v2.2.1 + manual raw-file verification
**Reviewer basis:** static analysis + direct `.kicad_sch`/`.kicad_pcb` inspection. **No component datasheets were available** (no `datasheets/` directory, no distributor API keys configured) — see *Verification Basis* below. All pin-level and electrical claims in this report are **consistency checks against the schematic/PCB themselves, not manufacturer-datasheet-verified**, except where explicitly marked "domain knowledge, unverified."

## Verdict: **Conditionally ready — fix 3 items before fab, 1 before flight test**

The board is structurally sound (0 unrouted nets, 4-layer stackup with dedicated ground/power planes, clean gerber export) but has one thermal margin problem serious enough to affect flight reliability, a hard sourcing blocker for ordering, and a stale-file hygiene issue worth resolving before anyone edits this design again. Nothing found is a "board is dead on arrival" bug (no swapped IC pins were found in the ICs checked), but the LDO thermal issue below is not cosmetic — it affects the rail that powers the flight computer itself.

---

## Blockers (fix before fab)

| # | Severity | Finding | Evidence |
|---|----------|---------|----------|
| 1 | **HIGH — thermal/safety** | U5 (AP2112K‑3.3, SOT‑23‑5) regulates 5V→3.3V for the **ESP32‑S3 flight computer's own supply** (`ESP_3V3`). At an assumed WiFi‑active load current, estimated Tj ≈ 124 °C against a 125 °C absolute max — **2 °C margin**. The package has no thermal pad (θJA ≈ 250 °C/W) and a 1.7 V dropout, so *any* sustained current draw above a few tens of mA runs it hot. ESP32‑S3 WiFi TX bursts commonly pull 300–500 mA. | `analysis/2026-09-11_1844/thermal.json` (rule TS‑002); schematic `U5` on `fc_ldo.kicad_sch`/`esp32s3.kicad_sch` net `ESP_3V3`. **Caveat:** the load current behind this estimate is a heuristic default, not a measured/derived figure (schematic power-budget module returned `estimated_load_mA: 0` for this rail — see Gaps below) — so treat 124 °C as directionally right, not exact. Given ESP32-S3's known TX current spikes, this is very plausibly correct or optimistic, not pessimistic. |
| 2 | **HIGH — sourcing (pre-fab gate)** | Only 7 of 30 unique parts (23%) have an MPN populated in the schematic. Rule `SS-001`: "Sourcing blocker: BOM has <50% MPN coverage. Board is not pre-fab ready." **Note:** a separate hand-fabrication BOM exists (`production/FULL_POP_..._bom.csv`) with LCSC part numbers for nearly every part — the gap is that this data lives in a CSV outside the KiCad schematic, not in the symbol properties the analyzer reads. | `analysis/2026-09-11_1844/schematic.json` (`bom_coverage.mpn_pct=21.9`); cross-checked against `production/FULL_POP_Autonomous_Fixed_Wing_FC_v3.0_bom.csv`, which does have LCSC numbers for 27 of the ~30 lines. |
| 3 | **MEDIUM — housekeeping / design-sync risk** | `airspeed.kicad_sch` and `lidar.kicad_sch` exist in the project directory but are **not instantiated anywhere in the sheet hierarchy** — verified by grepping every `.kicad_sch` for `(sheet ...)` includes; neither file is referenced from any reachable sheet. **This is not a missing-sensor bug**: the real, wired design already has airspeed and lidar interfaces — `J15` ("Airspeed I2C") and `J8` ("LIDAR I2C") both live in `external_sensors.kicad_sch`, which *is* in the hierarchy, and both appear in the production BOM/position files. `lidar.kicad_sch`'s components (`J8`, `L2`) are literal duplicates of what's already wired in `external_sensors.kicad_sch`. `airspeed.kicad_sch`'s components (`J12`, `L3`) do **not** appear anywhere in the reachable hierarchy at all — this looks like an earlier on-board-sensor-IC design draft that was superseded by the simpler external-connector approach and never deleted. Recommendation: delete both orphan files, or if there was a reason to move to an on-board airspeed sensor with the `L3` filtering shown in the orphan file, wire it in deliberately. Left as-is, these files will confuse the next person who edits the hierarchy or runs ERC sheet-by-sheet. | `grep -oE '"[a-zA-Z_]+\.kicad_sch"' *.kicad_sch` cross-reference (see transcript); `analysis/2026-09-11_1844/schematic.json` `sheets[]` (11 reachable sheets, excludes airspeed/lidar). |

## Fix before flight test (not fab-blocking)

| # | Severity | Finding | Evidence |
|---|----------|---------|----------|
| 4 | WARNING | **VBUS on J13 (USB‑C) has no decoupling and no ESD/TVS protection.** J13 is marked DNP (hand-solder), but if it's ever populated for field debugging, an unprotected USB‑C VBUS on a vehicle that operates outdoors is a real ESD exposure path into the 5V domain. | `schematic.json` rules `UC-001`, `UC-002`. |
| 5 | WARNING | 18 of 18 external/inter-board connectors (servo I/O, ESC I/O, GPS/UART, I2C sensor headers, SPI, debug headers) show **zero ESD protection** (`EP-AUD`, "none coverage" on all). For a design with cables running out to servos/ESCs/GPS in an airframe subject to field handling and connector mating/unmating, this is worth a deliberate risk decision rather than a silent gap — TVS diodes on at least the RC receiver input (J7) and any GPS/telemetry cable are the highest-value additions. | `schematic.json`, 18× `EP-AUD` findings. |
| 6 | INFO (assess as MEDIUM in practice) | J13 (USB‑C receptacle, 16‑pin **SMD**, fine-pitch) is marked `(dnp yes)` — i.e., intended for hand assembly, not JLCPCB SMT placement. Hand-soldering a 16-pin SMD USB-C receptacle without hot air/reflow is failure-prone (bridged VBUS/GND shield pins, tombstoned shield tabs). All the *other* DNP parts (J1–J9, J16, J19, J20, J22 — the servo/ESC/UBEC pin headers, and J7 the RC receiver connector) are through-hole 3‑pin headers, where DNP-for-hand-solder is completely normal/expected. J13 is the odd one out. Confirm this is intentional. | Raw `esp32s3.kicad_sch:3529-3537` — `(dnp yes)(in_bom yes)(on_board yes)` on the `USB_C_Receptacle_USB2.0_16P` symbol; cross-checked against `production/*_desig.csv` (J13 present, position-only) vs `*_bom.csv` (J13 absent — no LCSC part ordered for it). |
| 7 | WARNING (EMC) | EMC pre-compliance risk score **52/100** (moderate-high). Drivers: low ground-plane fill ratio (F.Cu 49%, B.Cu 44% — confirmed directly from zone fill data, not just the EMC heuristic), missing stitching vias at ~20+ signal layer transitions (SPI/I2C/UART buses to the IMUs, barometer, SD card, UWB radio, inter‑ESP link), SCK routed on an outer layer near connectors J10/J15, and one connector (J13) with no local filtering. See EMC section below. | `analysis/2026-09-11_1844/emc.json`; `pcb.json` zones (fill_ratio 0.494/0.441 for GND on F.Cu/B.Cu). |
| 8 | INFO | PCB has an **unfilled zone on net 0** ("no net") spanning all 4 copper layers (`is_filled: false`, no `fill_ratio`). If this is a deliberate keepout/rule area, fine; if it's a stale zone from editing, refill or delete it — stale zone fill data also means any "copper presence" analysis over that area is unreliable until refilled. | `pcb.json` `zones[]` entry with `net=0`, all layers, `is_filled=false`. |

## Positive findings (verified directly, not just "no finding raised")

- **Routing is complete**: 0 unrouted nets across 84 nets, 632 track segments, 178 vias on a 4-layer stack.
- **Custom DRC rule exists and is architecturally sound**: `fixed-wing-drone.kicad_dru` enforces ≥2mm clearance between `/FC_5V` (flight-computer power) and `/actuator_io/esc_io/SERVO_5V` (servo power) — a deliberate isolation choice so servo-current transients/brownouts don't couple into the FC's own supply. Verified both nets do have separate zone polygons on the shared `In2.Cu` (Pwr.Cu) inner layer as the rule implies. **Not independently verified**: actual DRC pass/fail against this rule — `kicad-cli` is not installed in this environment, so native DRC could not be run (see Gaps).
- **ESP32-S3 EN (reset) circuit is correctly designed** — I initially flagged this as a possible false-positive-worth-checking because the analyzer's `PU-001` rule reported "U1 pin EN missing pull-up resistor." Direct pin-net trace shows this is a **false positive**: `EN` is pulled up through `R7` (10k) to `ESP_3V3`, decoupled by `C12` (1µF) and `C14` (100nF), with `SW1` (TS‑1088 tactile switch) providing a manual reset-to-GND path — a textbook-correct ESP32 EN circuit. (`esp32s3.kicad_sch`, `ic_pin_analysis` for U1 pin 3.)
- **U3 (backup IMU) INT1–INT4 "missing pull-up" findings are also false positives/non-issues** — direct inspection shows all four interrupt pins are explicitly marked `NO_CONNECT` (deliberately unused), not floating inputs needing a pull-up. The `PU-001` heuristic doesn't distinguish sensor-output interrupt pins from bus inputs; both flagged instances here are noise, not real findings.
- **Gerber export is clean**: extracted and ran the gerber analyzer against the `production/FULL_POP_..._v3.0.zip` package (15 files: 4 copper layers incl. 2 inner planes, both silkscreens, both masks, both paste, edge cuts, PTH/NPTH drill + drill maps) — 0 findings (no missing layers, no alignment issues, no drill classification problems).
- **Component/footprint reconciliation is clean**: 75 schematic components vs 78 PCB footprints; the +3 is fully accounted for by 3 mounting holes (`M2`, no schematic symbol expected) — no unexplained extras.
- **DNP strategy for THT headers is sound**: all pin-header connectors (servo/ESC/UBEC-input, RC receiver) are marked DNP for SMT assembly with position data retained — standard "assembly house places SMD, builder hand-solders THT headers" practice, not a build-blocking omission.

## Power Tree

```
Battery/BEC (off-board) ──> J20 "FC UBEC Input" ──> FC_5V ─┬──> U5 AP2112K-3.3 ──> ESP_3V3 ──> U1 (ESP32-S3-WROOM-1)
                                                              └──> U6 AP2112K-3.3 ──> SENSOR_3V3 ──> U2/U3 (IMUs), U7 (barometer), external sensor headers
Servo/ESC power path: separate SERVO_5V net (2mm-isolated from FC_5V by custom DRC rule), routed to J19/J22 (ESC1/ESC2) and servo connectors via In2.Cu plane
```

- Two identical AP2112K‑3.3 LDOs (U5, U6) in SOT‑23‑5, no thermal pad, both dropping 5V→3.3V (1.7V dropout each). Only U5's rail was flagged by the thermal analyzer (it's carrying the higher-current MCU load); U6's sensor rail load is presumably lower but uses the *same* thermally marginal topology — worth a sanity check on total sensor-rail current (2× IMU + barometer + any bus pull-ups) if more sensors get added later.
- Regulator Vout for both was derived via `fixed_suffix` (deterministic — the "-3.3" in the part number), not a feedback-divider calculation, so this is a solid data point, not a heuristic guess.
- No PG (power-good) signal traced for either regulator — `PS-001` notes PG status is unknown without a datasheet (AP2112K does not have a PG pin per its part family — this is expected/non-issue, but unverified here since no datasheet was pulled).

## EMC Pre-Compliance Summary

Risk score: **52/100** (moderate-high), 55 findings (3 error, 36 warning, 16 info), trust level **low** (0% datasheet-backed — expected, since no datasheets were synced; findings are topology/heuristic-based).

Top drivers, all corroborated against raw PCB data (not just the EMC heuristic layer):
- **GP‑001/GP‑004** — reference-plane gaps under signal traces + low GND fill ratio, confirmed directly (F.Cu GND zone fill 49.4%, B.Cu 44.1%).
- **RP‑001** — 20+ signal nets (SPI to both IMUs/barometer/SD/UWB, I2C, inter-ESP UART, ESP32 BOOT strap) change layers without a nearby stitching via, weakening return-path continuity — this is the largest single finding category and the most actionable (add stitching vias near layer transitions on these nets).
- **IO‑001** — no EMC filtering near J13 (consistent with finding #4 above).
- **CK‑001/CK‑003** — the SPI `SCK` clock is routed on an outer layer and passes near connectors J10 and J15 — a radiated-emissions risk at harmonics of the SPI clock rate.

## Thermal Hotspot Summary

Score 85/100, 1 error + 4 info findings, all centered on **U5** (see Blocker #1). Four MLCCs (C6, C7, C10, C11 — the very decoupling/bulk caps for this rail) sit within 3–7mm of the hot regulator; at the estimated 124°C ambient-adjacent temperature, X7R caps lose ~15%+ effective capacitance, compounding the margin problem on the rail that's already running hot.

## SPICE Simulation

**Not run — no simulator installed.** Checked `ngspice`, `ltspice`, `xyce` via `which`; none present in this environment. This is a disclosed gap, not a silent skip. If a simulator becomes available, the highest-value targets detected by the schematic analyzer are the RC filters (10 detected) and the two LDO topologies — worth simulating the AP2112K‑3.3 loop stability/transient response given the thermal finding above.

## Component Lifecycle Audit

**Not run — no network/distributor API keys configured** (checked `DIGIKEY_CLIENT_ID`, `MOUSER_SEARCH_API_KEY`, `ELEMENT14_API_KEY`; none set). Disclosed gap. LCSC lookups (no auth required) were also not attempted in this pass — recommend running `lifecycle_audit.py --only lcsc` separately using the LCSC part numbers already present in `production/FULL_POP_..._bom.csv`, since MPN coverage in the schematic itself is too low (23%) for the audit to be useful as-is.

## Datasheet Coverage / Verification Basis

**No `datasheets/` directory existed; no distributor API keys were configured**, so no automated datasheet sync was possible in this environment. `DS-002` fired (schematic analyzer). Consequently:
- All pin-function, pin-mapping, and "matches datasheet" claims in this report are **schematic-internal consistency checks**, not datasheet-verified — the EN circuit and IMU NC-pin findings above were confirmed by tracing the schematic's own netlist, not by checking against an AP2112K-3.3 or IMU datasheet.
- No IC in this design was pin-verified against a manufacturer PDF. The ESP32‑S3‑WROOM‑1 module footprint/pinout, the two IMUs (U2/U3, both unidentified generic values — "Primary IMU"/"Backup IMU" — no MPN in the schematic beyond the production BOM's LCSC numbers C1850418/C194919), and the DPS368-family barometer (U7) were **not** opened against their datasheets in this pass.
- Recommendation: run `sync_datasheets_lcsc.py` against the LCSC part numbers already present in the production BOM CSV (even though they're not in the schematic's MPN properties) before the next review pass — this closes both the datasheet gap and the sourcing-gate blocker (#2) in one step, since those numbers are already known-good enough to have been used for a real fab run.

## Gaps / Not Performed (explicit disclosure)

| Item | Status | Reason |
|------|--------|--------|
| Datasheet sync / per-IC datasheet verification | Not performed | No `datasheets/` dir, no distributor API keys in this environment |
| SPICE simulation | Not performed | No ngspice/LTspice/Xyce installed |
| Component lifecycle audit | Not performed | No network-authenticated distributor API keys (LCSC no-auth path was not attempted this pass) |
| Native KiCad DRC/ERC (`kicad-cli`) | Not performed | `kicad-cli` not on PATH in this environment; relied on the kicad-happy static analyzers' ERC/DRC-equivalent checks instead, which is a narrower rule set than KiCad's own DRC/ERC engine |
| Full per-IC Deep Review pass (all ICs against datasheets) | Partially performed | Only U1 (EN circuit) and U3 (INT pins) were pin-traced in depth, both to resolve/triage specific analyzer findings; U2, U6, U7, and the ESP32-S3's full pinout were not individually walked pin-by-pin against a datasheet due to the datasheet gap above |
| Prior design review delta | Not applicable | No earlier `*review*.md`/`*design-review*.md` file existed in the project directory to diff against |
| PDF reference design comparison | Not applicable | No PDF reference designs or datasheet application circuits found in the project tree or sibling directories (`../docs` contains only `buses.md`, a markdown note, not a schematic PDF) |
| companion `uwb-radio-kicad/` project | Out of scope | Separate KiCad project (its own `.kicad_pro`), not part of `fixed-wing-drone.kicad_pcb`; noted for awareness only, not reviewed |

---

*Full analyzer JSON outputs are cached under `analysis/2026-09-11_1844/` (schematic.json, pcb.json, cross_analysis.json, emc.json, thermal.json, gerber.json) for anyone who wants to re-query specific findings.*
