# Design Review — Autonomous Fixed-Wing Drone Flight Controller (Post-Rework Check)

**Board:** `fixed-wing-drone` v3.0 (4-layer, 91.0 × 61.5 mm)
**Review date:** 2026-09-12
**Location:** `/Users/travis/projects/autonomous-rc-plane/kicad/flight-controller/` (project moved from parent `kicad/` folder this session; all prior edits carried over intact)
**Tooling:** kicad-happy analyzer suite (`kicad`, `emc` skills) v2.2.1 + manual netlist/datasheet verification. No SPICE simulator installed (ngspice/ltspice/xyce all absent) — simulation not performed, same disclosed gap as the prior review.

## Verdict: Conditionally ready — 1 real pre-fab blocker, 2 items needing attention before ordering

The rework since the last review (LDO thermal fix, USB-C/SBUS ESD protection, full BOM part-number audit) is solid — every change was re-verified against KiCad's own netlist/ERC engine and holds up in the current file state. Nothing from that work regressed in the folder move. The board's thermal problem is fully resolved (100/100 thermal score, 0 findings). What's now blocking is procedural, not electrical: **the production/ fab package is stale and predates all of tonight's changes**, and **two component values are desynced between schematic and PCB**.

---

## Blockers (fix before fab)

| # | Severity | Finding | Evidence |
|---|----------|---------|----------|
| 1 | **HIGH — stale fab package** | `production/FULL_POP_Autonomous_Fixed_Wing_FC_v3.0.zip` and its BOM/CPL/position CSVs are dated **2026-09-11 11:18** — before the LDO swap (U5→U4), the USB-C/SBUS TVS additions (D4/D6), and the new caps (C17/C18/C19). Confirmed directly: `production/..._bom.csv` has zero line items for U4, D6, or C19. If this package were sent to a fab today, it would build the **old, thermally-marginal board** with none of tonight's fixes. | `production/*.csv` mtimes vs. `esp32s3.kicad_sch` mtime (00:35 today); grep for U4/D6/C19 in the BOM CSV returns no matching rows. |
| 2 | **MEDIUM — schematic/PCB desync** | D4 and D6 (the two ESD5Z5.0T1G TVS diodes) still show the old placeholder text **"TVS 5V"** in the PCB file's footprint Value field, not the real part number. This is because the schematic edits were made by direct file editing rather than through KiCad's GUI, so "Update PCB from Schematic" was never re-run to propagate the Value text. Purely cosmetic electrically (nets are correct — see below), but it means any BOM/CPL export taken from the PCB right now would show the wrong value for these two parts. | `cross_analysis.json` rule XV-002 (both D4 and D6); confirmed directly by reading the Value property in `fixed-wing-drone.kicad_pcb` for both refs — both literally read `"TVS 5V"`. |

**Fix for both:** open the project in KiCad and run Tools → Update PCB from Schematic (or re-export gerbers/BOM/CPL) before ordering anything.

---

## Verified still-correct (re-checked fresh in the current file state, not just carried over from memory)

- **U4 (Torex XC6220B331PR-G, SOT-89-5)** — pin mapping confirmed via fresh schematic analysis: CE/VIN→`5V_SRC`, VSS(+tab)→`GND_SRC`, VOUT→`ESP_3V3`, NC unconnected. Thermal vias under the tab: 6 placed, netted to GND, analyzer confirms "adequate (6/5 min)".
- **Thermal: 100/100, zero findings.** The LDO thermal margin problem from the original review (U5 at 123.5°C, 1.5°C margin) is fully resolved — U4 doesn't even register as a thermal concern anymore.
- **C18 (10µF input) / C19 (4.7µF output)** — both still correctly netted to the LDO's VIN/VOUT+GND, matching Torex's own reference circuit values.
- **D4/D6 (ESD5Z5.0T1G TVS)** — nets confirmed correct (D4 on USB-C VBUS `__unnamed_4`+GND, D6 on `RC_SBUS`+GND) — the schematic/PCB *connectivity* is right, only the cosmetic Value text lags (see Blocker #2).
- **C17 (Murata 10µF/50V VBUS bulk cap)** — correctly netted to VBUS+GND.
- **D5** (esc_io.kicad_sch) — re-confirmed via fresh netlist: two distinct, legitimate nets (K→`/FC_5V`, A→net shared with Q1), not a duplicate or short.
- **Orphan files** (`airspeed.kicad_sch`, `lidar.kicad_sch`) — confirmed still deleted, not resurrected by the folder move.
- **Routing:** 100% complete, 0 unrouted nets, 81 footprints (78 schematic components + 3 mounting holes, reconciles cleanly — the one PCB-only item, M2, is a mounting hole, confirmed via `cross_analysis` XV-001, info severity, expected).

---

## Pre-existing items (not new, not touched this session — carried over from the original review)

These were already known; re-confirmed present in this pass, listed here for completeness rather than re-litigated in depth:

- **EMC risk score: 58/100** (was 52/100 in the original review — the increase is from the new USB-C/SBUS TVS branches and LDO changes adding a small number of new signal-adjacent findings, not a regression in the existing layout). Two `error`-severity items:
  - `GP-001`: `Net-(J20-Pin_2)` (FC UBEC power input) has a 75%-covered reference-plane gap over 5.3mm of routing.
  - `RP-001`: `/SCK` (SPI clock) has 4 layer transitions with no nearby ground stitching via — return-path/EMI risk, same class of finding as the original review flagged across ~20 nets.
- **Ground plane fill still low**: `GP-004` still fires (matches the original review's F.Cu 49% / B.Cu 44% fill finding).
- **No EMC filtering / low ESD coverage on most external connectors** (`IO-001`, 10 connectors) and **insufficient ground pins on J8/J15** (`IO-002`) — this is the same "18 connectors, no ESD protection" finding from the original review; J7 (RC receiver) and J13 (USB-C) are now the two exceptions since this session's work.
- **DFM, assembly-level (pre-existing, unrelated to tonight's changes):**
  - `FD-001` (error): **no fiducial markers on F.Cu**, 66 SMD components including fine-pitch QFN/BGA-class parts (0.25mm finest pad). Worth adding 3 fiducials per side before a production assembly run — affects pick-and-place placement accuracy, not board function.
  - `TE-001` (warning): **0% test point coverage** across 83 nets — no in-circuit test access. Fine for hand-assembly/bring-up, worth considering if this ever goes to a contract assembler.

## Not performed / gaps (same as prior review, still disclosed)

| Item | Status | Reason |
|---|---|---|
| SPICE simulation | Not performed | No ngspice/LTspice/Xyce installed in this environment |
| Native `kicad-cli` DRC | Not performed this pass | ERC and netlist were run via `kicad-cli` in earlier session work and passed cleanly; full DRC re-run not repeated this pass, no schematic/footprint-shape changes since then to warrant it |
| Gerber re-analysis | Not performed | The only gerber package present (`production/*.zip`) is the stale pre-rework one (see Blocker #1) — analyzing it would just re-confirm it's outdated, not tell you anything about the current design |
| Component lifecycle audit | Not performed | No distributor API keys configured, same as original review |

---

*Full analyzer JSON for this pass is cached under `analysis/2026-09-12_0055*` (schematic, pcb, cross_analysis, emc, thermal) for anyone who wants to re-query specific findings.*
