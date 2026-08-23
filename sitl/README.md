# SITL (software-in-the-loop) for the attitude PID

A host-side simulator for tuning `ROLL_PID_CFG` / `PITCH_PID_CFG` before the plane exists to fly.
It links directly against `../main/pid.c` — the exact same source file the firmware compiles —
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
