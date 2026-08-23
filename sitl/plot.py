import csv
import sys

import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "output.csv"

t, goal, angle, measured_angle, rate, pid_out, commanded_deflection, actual_deflection = (
    [], [], [], [], [], [], [], []
)
with open(path, newline="") as f:
    for row in csv.DictReader(f):
        t.append(float(row["t_s"]))
        goal.append(float(row["goal_deg"]))
        angle.append(float(row["angle_deg"]))
        measured_angle.append(float(row["measured_angle_deg"]))
        rate.append(float(row["rate_dps"]))
        pid_out.append(float(row["pid_output_us"]))
        commanded_deflection.append(float(row["commanded_deflection_deg"]))
        actual_deflection.append(float(row["actual_deflection_deg"]))

fig, (ax_angle, ax_rate, ax_output) = plt.subplots(3, 1, sharex=True, figsize=(9, 7))

ax_angle.plot(t, measured_angle, color="lightcoral", linewidth=0.8, alpha=0.7, label="measured angle (deg)")
ax_angle.plot(t, angle, label="true angle (deg)")
ax_angle.plot(t, goal, "--", color="gray", label="goal (deg)")
ax_angle.set_ylabel("angle (deg)")
ax_angle.legend()
ax_angle.grid(True)

ax_rate.plot(t, rate, color="tab:orange")
ax_rate.set_ylabel("rate (deg/s)")
ax_rate.grid(True)

ax_output.plot(t, pid_out, label="pid output (us)")
ax_output.plot(t, commanded_deflection, "--", alpha=0.6, label="commanded deflection (deg)")
ax_output.plot(t, actual_deflection, label="actual deflection (deg)")
ax_output.set_ylabel("output")
ax_output.set_xlabel("time (s)")
ax_output.legend()
ax_output.grid(True)

fig.suptitle(path)
fig.tight_layout()
plt.show()
