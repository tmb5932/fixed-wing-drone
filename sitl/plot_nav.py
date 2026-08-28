import csv
import math
import sys

import matplotlib.pyplot as plt

path = sys.argv[1] if len(sys.argv) > 1 else "nav_output.csv"

t, lat, lon, heading, desired_heading, heading_err, goal_roll, roll, dist, wp_idx = (
    [], [], [], [], [], [], [], [], [], []
)
with open(path, newline="") as f:
    for row in csv.DictReader(f):
        t.append(float(row["t_s"]))
        lat.append(float(row["lat"]))
        lon.append(float(row["lon"]))
        heading.append(float(row["heading_deg"]))
        desired_heading.append(float(row["desired_heading_deg"]))
        heading_err.append(float(row["heading_err_deg"]))
        goal_roll.append(float(row["goal_roll_deg"]))
        roll.append(float(row["roll_deg"]))
        dist.append(float(row["dist_to_wp_m"]))
        wp_idx.append(int(row["wp_idx"]))

fig, ((ax_track, ax_dist), (ax_heading, ax_roll)) = plt.subplots(2, 2, figsize=(11, 8))

# Ground track in local meters (flat-earth, fine over the short distances SITL
# missions use), colored by which waypoint leg is currently active.
lat0, lon0 = lat[0], lon[0]
east_m = [(lo - lon0) * 111320.0 * math.cos(math.radians(lat0)) for lo in lon]
north_m = [(la - lat0) * 111320.0 for la in lat]
sc = ax_track.scatter(east_m, north_m, c=wp_idx, cmap="viridis", s=4)
ax_track.plot(0, 0, "k^", markersize=10, label="start")
ax_track.set_xlabel("east (m)")
ax_track.set_ylabel("north (m)")
ax_track.set_title("ground track (color = active waypoint index)")
ax_track.set_aspect("equal", adjustable="datalim")
ax_track.grid(True)
ax_track.legend()
fig.colorbar(sc, ax=ax_track, label="wp_idx")

ax_dist.plot(t, dist)
ax_dist.set_xlabel("time (s)")
ax_dist.set_ylabel("distance to active waypoint (m)")
ax_dist.grid(True)

ax_heading.plot(t, heading, label="true heading (deg)")
ax_heading.plot(t, desired_heading, "--", alpha=0.7, label="desired heading (deg)")
ax_heading.set_xlabel("time (s)")
ax_heading.set_ylabel("heading (deg)")
ax_heading.legend(fontsize="small")
ax_heading.grid(True)

ax_roll.plot(t, goal_roll, "--", alpha=0.7, label="goal roll (deg)")
ax_roll.plot(t, roll, label="actual roll (deg)")
ax_roll.set_xlabel("time (s)")
ax_roll.set_ylabel("roll (deg)")
ax_roll.legend(fontsize="small")
ax_roll.grid(True)

fig.suptitle(path)
fig.tight_layout()
plt.show()
