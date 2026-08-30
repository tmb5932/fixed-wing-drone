# SITL (software-in-the-loop) for the attitude + nav PIDs

Host-side simulators for tuning `ROLL_PID_CFG` / `PITCH_PID_CFG` (`sitl_sim.exe`) and
`HEADING_PID_CFG` (`nav_sim.exe`) before the plane exists to fly. Both link directly against
`../main/pid.c` and `../main/gps_math.c` — the exact same source files the firmware compiles —
so a gain that behaves well here is a gain that will behave the same way on the real controller.

## Build & run

Requires a host C compiler (this repo's ESP-IDF toolchain is a cross-compiler for the ESP32S3 and
can't build a Windows executable; see project memory / setup notes for the MSYS2/mingw-w64 install
used here).

```sh
cd sitl
mingw32-make
./sitl_sim.exe [k_p] [k_i] [k_d] [duration_s] [initial_disturbance_deg] [--axis=roll|pitch] [--goal=deg] [--i_limit=us] [--noise_deg=stddev] [--servo_rate=deg_per_s] [--seed=n]
python plot.py output.csv     # or: .venv\Scripts\python.exe plot.py output.csv
```

CLI defaults are `k_p=10 k_i=0 k_d=0`, 5s, starting 20° off level, roll axis, goal 0° (i.e. recover
to level — the firmware's only behavior until something calls `set_goal_roll_deg`/`set_goal_pitch_deg`).

Or use `python gui.py` (`.venv\Scripts\python.exe gui.py`) for an interactive version: pick an axis,
enter gains, hit Run, see the plot update immediately, with the previous run kept as a faint
reference line so you can see the effect of a change rather than comparing two separate plot windows
from memory. (The reference line clears when you switch axes — roll and pitch use different plant
dynamics now, so overlaying them wouldn't mean anything.) The noise/servo-rate/seed fields live in
their own "Sensor / actuator realism" group, separate from the tuning fields, since they describe
imperfections in the simulated world rather than the controller itself.

`nav_sim.exe` simulates the outer waypoint-following loop added in `nav.c`: it cascades
`HEADING_PID_CFG` into the same roll-attitude loop as `sitl_sim.exe` (reusing its plant/actuator
model), flying a simple coordinated-turn/flat-earth world model toward one or more waypoints.

```sh
./nav_sim.exe [--target_bearing=deg] [--target_dist_m=m] [--wp=lat,lon (repeatable, overrides target_bearing/dist)] [--start_lat=deg] [--start_lon=deg] [--heading0=deg] [--airspeed=mps] [--duration_s=s] [--heading_kp=] [--heading_ki=] [--heading_kd=] [--heading_ilimit=] [--roll_kp=] [--roll_ki=] [--roll_kd=] [--roll_ilimit=] [--roll_noise_deg=] [--heading_noise_deg=] [--servo_rate=deg_per_s] [--seed=n]
python plot_nav.py nav_output.csv
```

With no `--wp`, it generates a single waypoint at `--target_bearing`/`--target_dist_m` from the
start position, which is usually more convenient for gain-sweeping than hand-picked lat/lon pairs.

`mingw32-make test` builds both binaries and runs `tests/run_tests.py`, an 80-scenario regression
suite against the current firmware gains (clean step responses, noise-robustness sweeps, and
waypoint-mission edge cases) — exits nonzero if anything fails.
