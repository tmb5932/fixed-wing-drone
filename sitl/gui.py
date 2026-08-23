import csv
import os
import subprocess
import tkinter as tk
from tkinter import ttk

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

SITL_DIR = os.path.dirname(os.path.abspath(__file__))
EXE_PATH = os.path.join(SITL_DIR, "sitl_sim.exe")
CSV_PATH = os.path.join(SITL_DIR, "output.csv")

DEFAULTS = {
    "k_p": "10", "k_i": "0", "k_d": "0", "i_limit": "250",
    "duration_s": "5", "disturbance_deg": "20", "goal_deg": "0",
}
LABELS = {
    "k_p": "k_p",
    "k_i": "k_i",
    "k_d": "k_d",
    "i_limit": "i_limit (us)",
    "duration_s": "duration (s)",
    "disturbance_deg": "initial disturbance (deg)",
    "goal_deg": "goal / bank to (deg)",
}

REALISM_DEFAULTS = {
    "noise_deg": "0", "servo_rate": "600", "seed": "1",
}
REALISM_LABELS = {
    "noise_deg": "IMU noise stddev (deg)",
    "servo_rate": "servo slew rate (deg/s)",
    "seed": "noise seed (0=random)",
}


def build_sim():
    """(Re)build sitl_sim.exe via the Makefile. Returns (ok, message)."""
    try:
        result = subprocess.run(
            ["mingw32-make"], cwd=SITL_DIR, capture_output=True, text=True
        )
    except FileNotFoundError:
        return False, "mingw32-make not found on PATH -- see sitl/README.md for toolchain setup."
    if result.returncode != 0:
        return False, f"Build failed:\n{result.stderr.strip()}"
    return True, "Build OK."


def read_csv(path):
    cols = {
        name: [] for name in (
            "t_s", "goal_deg", "angle_deg", "measured_angle_deg", "rate_dps",
            "pid_output_us", "commanded_deflection_deg", "actual_deflection_deg",
        )
    }
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            for name in cols:
                cols[name].append(float(row[name]))
    return cols


class SitlGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PID SITL tuner")
        self.geometry("1100x800")

        # Kept so the plot can show the last run as a faint reference line --
        # the point of a GUI over one-off CLI runs is seeing *the effect of a
        # change*, which needs the before and after on screen together. Cleared
        # whenever the axis changes -- roll and pitch use different plant
        # dynamics now, so overlaying them would be comparing apples to oranges.
        self.previous_run = None
        self.previous_axis = None

        self.vars = {key: tk.StringVar(value=val) for key, val in DEFAULTS.items()}
        self.realism_vars = {key: tk.StringVar(value=val) for key, val in REALISM_DEFAULTS.items()}
        self.axis_var = tk.StringVar(value="roll")

        form = ttk.Frame(self, padding=10)
        form.pack(side=tk.TOP, fill=tk.X)

        axis_frame = ttk.Frame(form)
        axis_frame.grid(row=0, column=0, columnspan=2, padx=(0, 12))
        ttk.Radiobutton(axis_frame, text="Roll", variable=self.axis_var, value="roll").pack(side=tk.LEFT)
        ttk.Radiobutton(axis_frame, text="Pitch", variable=self.axis_var, value="pitch").pack(side=tk.LEFT)

        for i, key in enumerate(DEFAULTS):
            ttk.Label(form, text=LABELS[key]).grid(row=0, column=2 + 2 * i, padx=(0, 4))
            entry = ttk.Entry(form, textvariable=self.vars[key], width=6)
            entry.grid(row=0, column=2 + 2 * i + 1, padx=(0, 12))
            entry.bind("<Return>", lambda _e: self.run())

        self.run_btn = ttk.Button(form, text="Run", command=self.run)
        self.run_btn.grid(row=0, column=2 + 2 * len(DEFAULTS), padx=(8, 0))

        realism_frame = ttk.LabelFrame(form, text="Sensor / actuator realism", padding=(8, 4))
        realism_frame.grid(row=1, column=0, columnspan=2 + 2 * len(DEFAULTS) + 1,
                            sticky="w", pady=(8, 0))
        for i, key in enumerate(REALISM_DEFAULTS):
            ttk.Label(realism_frame, text=REALISM_LABELS[key]).grid(row=0, column=2 * i, padx=(0, 4))
            entry = ttk.Entry(realism_frame, textvariable=self.realism_vars[key], width=6)
            entry.grid(row=0, column=2 * i + 1, padx=(0, 12))
            entry.bind("<Return>", lambda _e: self.run())

        self.status = ttk.Label(self, text="Checking build...", padding=(10, 4))
        self.status.pack(side=tk.TOP, fill=tk.X)

        self.fig = Figure(figsize=(8, 6))
        self.ax_angle = self.fig.add_subplot(3, 1, 1)
        self.ax_rate = self.fig.add_subplot(3, 1, 2, sharex=self.ax_angle)
        self.ax_output = self.fig.add_subplot(3, 1, 3, sharex=self.ax_angle)
        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.after(10, self.startup_build)

    def startup_build(self):
        if not os.path.exists(EXE_PATH):
            ok, msg = build_sim()
            if not ok:
                self.status.config(text=msg)
                self.run_btn.state(["disabled"])
                return
        self.status.config(text="Ready. Enter gains and press Run (or hit Enter in a field).")

    def run(self):
        try:
            values = {key: float(var.get()) for key, var in self.vars.items()}
            realism = {key: float(var.get()) for key, var in self.realism_vars.items()}
        except ValueError:
            self.status.config(text="All fields must be numbers.")
            return

        axis = self.axis_var.get()
        if axis != self.previous_axis:
            self.previous_run = None

        args = [
            EXE_PATH,
            str(values["k_p"]), str(values["k_i"]), str(values["k_d"]),
            str(values["duration_s"]), str(values["disturbance_deg"]),
            f"--axis={axis}",
            f"--goal={values['goal_deg']}",
            f"--i_limit={values['i_limit']}",
            f"--noise_deg={realism['noise_deg']}",
            f"--servo_rate={realism['servo_rate']}",
            f"--seed={int(realism['seed'])}",
        ]
        result = subprocess.run(args, cwd=SITL_DIR, capture_output=True, text=True)
        if result.returncode != 0:
            self.status.config(text=f"sitl_sim.exe failed: {result.stderr.strip()}")
            return

        data = read_csv(CSV_PATH)
        self.plot(data)
        self.status.config(text=result.stdout.strip())
        self.previous_run = (data["t_s"], data["angle_deg"])
        self.previous_axis = axis

    def plot(self, data):
        for ax in (self.ax_angle, self.ax_rate, self.ax_output):
            ax.clear()

        t = data["t_s"]

        if self.previous_run is not None:
            prev_t, prev_angle = self.previous_run
            self.ax_angle.plot(prev_t, prev_angle, color="lightgray", linewidth=1.5, label="previous run")

        # Only draw the measured-angle trace when noise is actually nonzero --
        # otherwise it's identical to the true-angle line and just adds visual
        # clutter to the common (noiseless) case.
        if any(m != a for m, a in zip(data["measured_angle_deg"], data["angle_deg"])):
            self.ax_angle.plot(t, data["measured_angle_deg"], color="lightcoral",
                                linewidth=0.8, alpha=0.7, label="measured angle (deg)")
        self.ax_angle.plot(t, data["goal_deg"], "--", color="gray", linewidth=1, label="goal (deg)")
        self.ax_angle.plot(t, data["angle_deg"], color="tab:blue", label="true angle (deg)")
        self.ax_angle.set_ylabel("angle (deg)")
        self.ax_angle.legend(loc="upper right", fontsize="small")
        self.ax_angle.grid(True)

        self.ax_rate.plot(t, data["rate_dps"], color="tab:orange")
        self.ax_rate.set_ylabel("rate (deg/s)")
        self.ax_rate.grid(True)

        self.ax_output.plot(t, data["pid_output_us"], label="pid output (us)")
        self.ax_output.plot(t, data["commanded_deflection_deg"], "--", alpha=0.6,
                             label="commanded deflection (deg)")
        self.ax_output.plot(t, data["actual_deflection_deg"], label="actual deflection (deg)")
        self.ax_output.set_ylabel("output")
        self.ax_output.set_xlabel("time (s)")
        self.ax_output.legend(loc="upper right", fontsize="small")
        self.ax_output.grid(True)

        self.fig.tight_layout()
        self.canvas.draw()


if __name__ == "__main__":
    SitlGui().mainloop()
