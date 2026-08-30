#!/usr/bin/env python3
"""SITL regression suite: runs sitl_sim.exe/nav_sim.exe against the current
firmware PID gains across a fixed set of scenarios and checks each one's
"Result:" line against a pass/fail threshold. Exits nonzero if any fail.

Usage: python tests/run_tests.py   (run from sitl/, or anywhere -- paths are
resolved relative to this file)
"""
import math
import re
import subprocess
import sys
from pathlib import Path

SITL_DIR = Path(__file__).resolve().parent.parent
SITL_SIM = SITL_DIR / "sitl_sim.exe"
NAV_SIM = SITL_DIR / "nav_sim.exe"

# Current firmware gains -- see main/main.c's ROLL_PID_CFG/PITCH_PID_CFG and
# main/nav.c's HEADING_PID_CFG. This suite validates *these* values, not a
# gain search -- if you change a gain, re-run this and expect failures to
# point at what broke, then update thresholds only if the new behavior is
# genuinely better, not just to make the suite pass.
ROLL_KP, ROLL_KI, ROLL_KD, ROLL_ILIMIT = 5.0, 0.0, 0.4, 250.0
PITCH_KP, PITCH_KI, PITCH_KD, PITCH_ILIMIT = 5.0, 0.0, 0.4, 250.0

RESULT_RE = re.compile(r"Result:\s*(.+)")


def parse_result(stdout):
    m = RESULT_RE.search(stdout)
    if not m:
        return None
    fields = {}
    for tok in m.group(1).split():
        key, _, val = tok.partition("=")
        fields[key] = val
    return fields


def run(cmd):
    proc = subprocess.run(cmd, cwd=SITL_DIR, capture_output=True, text=True, timeout=30)
    return proc.returncode, proc.stdout, proc.stderr


def offset_latlon(lat0, lon0, bearing_deg, dist_m):
    """Mirrors nav_sitl_main.c's offset_latlon() so mission waypoints can be
    specified as (bearing, distance) legs instead of hand-picked lat/lon."""
    R = 6371000.0
    b = math.radians(bearing_deg)
    lat0_r = math.radians(lat0)
    lat = lat0 + math.degrees((dist_m * math.cos(b)) / R)
    lon = lon0 + math.degrees((dist_m * math.sin(b)) / (R * math.cos(lat0_r)))
    return lat, lon


def chain_waypoints(legs):
    lat, lon = 0.0, 0.0
    wps = []
    for bearing, dist in legs:
        lat, lon = offset_latlon(lat, lon, bearing, dist)
        wps.append((lat, lon))
    return wps


# ---------------------------------------------------------------------------
# Attitude scenarios (sitl_sim.exe)
# ---------------------------------------------------------------------------

def attitude_scenarios():
    scenarios = []
    for goal in (10, 20, 30, 45):
        for sign in (1, -1):
            scenarios.append(("roll", f"roll clean step goal={goal * sign:+.0f}deg",
                               goal * sign, 0.0, 1))
    for goal in (10, 20):
        for sign in (1, -1):
            scenarios.append(("pitch", f"pitch clean step goal={goal * sign:+.0f}deg",
                               goal * sign, 0.0, 1))
    for noise in (1, 2):
        for seed in (1, 2, 3):
            scenarios.append(("roll", f"roll noise={noise}deg seed={seed}", 45, noise, seed))
    for noise in (1, 2):
        for seed in (1, 2, 3):
            scenarios.append(("pitch", f"pitch noise={noise}deg seed={seed}", 20, noise, seed))
    return scenarios


def check_attitude(axis, goal_deg, noise_deg, fields):
    diverged = fields.get("diverged") == "yes"
    final_err = float(fields["final_error_deg"])
    settle = float(fields["settle_time_s"])
    max_abs = float(fields["max_abs_angle_deg"])

    if diverged:
        return False, f"DIVERGED max_abs_angle_deg={max_abs:.2f}"

    if noise_deg == 0.0:
        # Clean step: calibrated against actual observed behavior at k_p=5/k_d=0.4
        # (roll converges to ~0 steady error; pitch settles near a small
        # nonzero offset from its restoring/spring term with zero k_i -- see
        # project-pid-gain-history memory. Bounds carry margin above that.)
        final_err_max = 0.5 if axis == "roll" else 3.0
        settle_max = 1.5 if axis == "roll" else 2.5
        ok = final_err <= final_err_max and 0 <= settle <= settle_max
        return ok, f"final_err={final_err:.2f}deg settle={settle:.2f}s"
    else:
        # Noisy: a tight settle band is never meaningfully "settled" under
        # sustained sensor noise, so bound peak excursion and final error
        # instead, with margin above observed noise=2deg behavior.
        max_abs_ceiling = 100.0 if axis == "roll" else 50.0
        final_err_max = 15.0 if axis == "roll" else 10.0
        ok = max_abs <= max_abs_ceiling and final_err <= final_err_max
        return ok, f"final_err={final_err:.2f}deg max_abs_angle={max_abs:.2f}deg"


def run_attitude_scenario(axis, goal_deg, noise_deg, seed):
    kp, ki, kd = (ROLL_KP, ROLL_KI, ROLL_KD) if axis == "roll" else (PITCH_KP, PITCH_KI, PITCH_KD)
    ilimit = ROLL_ILIMIT if axis == "roll" else PITCH_ILIMIT
    cmd = [str(SITL_SIM), str(kp), str(ki), str(kd), "5.0", "0.0",
           f"--axis={axis}", f"--goal={goal_deg}", f"--i_limit={ilimit}",
           f"--noise_deg={noise_deg}", f"--seed={seed}"]
    rc, out, err = run(cmd)
    if rc != 0:
        return False, f"exit {rc}: {err.strip()}"
    fields = parse_result(out)
    if not fields:
        return False, "no Result line in output"
    return check_attitude(axis, goal_deg, noise_deg, fields)


# ---------------------------------------------------------------------------
# Nav scenarios (nav_sim.exe)
# ---------------------------------------------------------------------------

BEARINGS_8 = (0, 45, 90, 135, 180, 225, 270, 315)
DISTANCES_4 = (50, 150, 300, 600)


def nav_scenarios():
    scenarios = []
    for bearing in BEARINGS_8:
        for dist in DISTANCES_4:
            scenarios.append({
                "name": f"grid bearing={bearing}deg dist={dist}m",
                "args": [f"--target_bearing={bearing}", f"--target_dist_m={dist}",
                         "--duration_s=90", "--seed=1"],
                "check": "reached",
            })
    for noise in (5, 10):
        for bearing in (90, 180):
            for seed in (1, 2, 3, 4, 5):
                scenarios.append({
                    "name": f"noise heading_noise={noise}deg bearing={bearing}deg seed={seed}",
                    "args": [f"--target_bearing={bearing}", "--target_dist_m=300",
                             "--duration_s=90", f"--heading_noise_deg={noise}", f"--seed={seed}"],
                    "check": "reached",
                })

    scenarios.append({
        "name": "edge: 180deg reversal",
        "args": ["--target_bearing=180", "--target_dist_m=300", "--duration_s=90", "--seed=1"],
        "check": "reached",
    })
    scenarios.append({
        "name": "edge: immediate-arrival waypoint (25m, inside acceptance radius)",
        "args": ["--target_bearing=90", "--target_dist_m=25", "--duration_s=30", "--seed=1"],
        "check": "reached_immediate",
    })

    wps_3 = chain_waypoints([(90, 200), (0, 200), (270, 150)])
    wp_args = [f"--wp={lat:.7f},{lon:.7f}" for lat, lon in wps_3]
    scenarios.append({
        "name": "edge: 3-waypoint mission",
        "args": wp_args + ["--duration_s=120", "--seed=1"],
        "check": "reached",
    })

    wps_sq = chain_waypoints([(90, 80), (0, 80), (270, 80), (180, 80)])
    wp_args = [f"--wp={lat:.7f},{lon:.7f}" for lat, lon in wps_sq]
    scenarios.append({
        "name": "edge: square mission, 80m sides (4 consecutive ~90deg turns)",
        "args": wp_args + ["--duration_s=60", "--seed=1"],
        "check": "reached",
    })

    return scenarios


def check_nav(check_kind, fields):
    reached = fields.get("reached_final") == "yes"
    if not reached:
        return False, f"did not reach final waypoint (min_dist_m={fields.get('min_dist_m')})"
    if check_kind == "reached_immediate":
        t = float(fields["first_acceptance_t_s"])
        if t > 1.0:
            return False, f"reached, but not immediately (first_acceptance_t_s={t:.2f}s)"
        return True, f"first_acceptance_t_s={t:.2f}s"
    return True, f"min_dist_m={fields['min_dist_m']}"


def run_nav_scenario(sc):
    cmd = [str(NAV_SIM)] + sc["args"]
    rc, out, err = run(cmd)
    if rc != 0:
        return False, f"exit {rc}: {err.strip()}"
    fields = parse_result(out)
    if not fields:
        return False, "no Result line in output"
    return check_nav(sc["check"], fields)


# ---------------------------------------------------------------------------

def main():
    if not SITL_SIM.exists() or not NAV_SIM.exists():
        print(f"Missing binaries -- run `mingw32-make` in {SITL_DIR} first.", file=sys.stderr)
        return 1

    results = []

    for axis, name, goal, noise, seed in attitude_scenarios():
        ok, detail = run_attitude_scenario(axis, goal, noise, seed)
        results.append((name, ok, detail))

    for sc in nav_scenarios():
        ok, detail = run_nav_scenario(sc)
        results.append((sc["name"], ok, detail))

    passed = sum(1 for _, ok, _ in results if ok)
    width = max(len(name) for name, _, _ in results)
    for name, ok, detail in results:
        status = "PASS" if ok else "FAIL"
        print(f"[{status}] {name.ljust(width)}  {detail}")

    print(f"\n{passed}/{len(results)} scenarios passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
