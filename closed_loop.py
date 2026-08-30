"""
closed_loop.py — "Closed-Loop Control" notebook tab.

Closes a displacement loop entirely in Python:

    uMD interferometer --MQTT--> DisplayFrame.read_latest()
                                        |
                                   control.py
                                        |
                          MokuWaveformFrame.set_dc_voltage()

Two algorithms are selectable at runtime and share this identical plumbing, so
switching between them changes only the control law:

  * Software PID    — the classical baseline
  * Preview + ADRC  — exact-inversion feedforward from the known future
                      setpoint, plus a disturbance-and-rate observer

"Run A/B" drives both, back to back, over the same trajectory, and reports the
two side by side -- the comparison this port exists to make.

This is distinct from the Moku tab's "PID Smoothing": that one configures a
PID inside the Moku FPGA which only ever sees the waveform generator's own
output, never the interferometer. The two are mutually exclusive because both
drive Output 1.
"""
from __future__ import annotations

import csv
import os
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

# Widgets that take a `bootstyle` must come from ttkbootstrap itself -- on
# ttkbootstrap 2.x plain ttk widgets reject the option. Plain ttk is still used
# for labels/entries/frames, which take no bootstyle.
import ttkbootstrap as tb

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import app_settings
from control import (
    WAVEFORMS,
    PlantModel,
    PreviewADRC,
    SoftwarePID,
    TrajectoryGenerator,
    fit_fopdt_step,
    imc_tune,
)
from process_raw import get_downloads_dir, select_folder_native

ALGORITHMS = ("Software PID", "Preview + ADRC")

SETTINGS_FILE = "closed_loop.json"

# Realistic ceiling for this loop: every tick costs one Moku network
# round-trip (~5-20 ms) and the interferometer streams at ~610 Hz through
# MQTT, so the loop runs at tens of Hz, not kHz.
DEFAULT_DT_MS = 50.0

# Stop if the measurement goes this many ticks stale -- but never sooner than
# WATCHDOG_MIN_S. The interferometer streams at ~610 Hz on its own MQTT thread,
# so freshness is really a property of that stream, not of the control period;
# scaling the timeout purely by dt makes a fast loop trip on ordinary GUI
# scheduling jitter rather than on an actual sensor failure.
WATCHDOG_TICKS = 3.0
WATCHDOG_MIN_S = 0.25

# Fraction by which the measured tick period may drift from nominal before
# warning. The controller matrices are discretised once at the nominal dt.
JITTER_WARN_FRAC = 0.20

# Floor on the settle time at 0 V between the two halves of an A/B run. The
# actual wait is max(AB_SETTLE_S, AB_SETTLE_TAUS * tau): both halves must start
# from the same state or the comparison measures initial conditions as much as
# it measures the control law, and how long the piezo takes to fall back to
# rest is a property of the plant, not a constant.
AB_SETTLE_S = 1.5
AB_SETTLE_TAUS = 5.0

# Rolling window shown in the live plot.
PLOT_WINDOW_S = 30.0

# Hard ceiling on the v_max a user may type, independent of the piezo model.
V_MAX_CEILING = 10.0


# ─────────────────────────────────────────────────────────────────────────────
# Live plot
# ─────────────────────────────────────────────────────────────────────────────

class LivePlot:
    """
    Three stacked axes: tracking (r vs y), error, and command voltage.

    A control loop cannot be tuned from a status line -- the shape of the error
    is what tells you whether a gain is too low, too high, or the wrong kind of
    correction entirely. For Preview+ADRC the disturbance estimate res1 is
    drawn on the error axis too, since that is the signal doing the work.

    Live updates rewrite each line's data rather than clearing and replotting.
    The clear-and-replot version measurably starved the control thread: a
    redraw holding the GIL for tens of milliseconds delays the next tick, and
    the preview feedforward is discretised for a fixed dt, so that jitter shows
    up directly as tracking error.
    """

    # Cap on points drawn per line. Beyond this the trace is decimated: more
    # samples than horizontal pixels buys nothing but redraw time.
    MAX_POINTS = 800

    def __init__(self, parent):
        self.fig = Figure(figsize=(6.4, 6.6), dpi=100, layout="constrained")
        self.ax_track = self.fig.add_subplot(3, 1, 1)
        self.ax_err = self.fig.add_subplot(3, 1, 2, sharex=self.ax_track)
        self.ax_volt = self.fig.add_subplot(3, 1, 3, sharex=self.ax_track)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self._lines = None
        self._mode = None
        self._style_axes()
        self.clear()

    def _style_axes(self):
        self.ax_track.set_ylabel("position (nm)")
        self.ax_err.set_ylabel("error (nm)")
        self.ax_volt.set_ylabel("command (V)")
        self.ax_volt.set_xlabel("time (s)")
        for ax in (self.ax_track, self.ax_err, self.ax_volt):
            ax.grid(True, alpha=0.3)
            ax.tick_params(labelsize=8)
            ax.yaxis.label.set_size(8)
        self.ax_volt.xaxis.label.set_size(8)

    def _reset_axes(self, title=""):
        for ax in (self.ax_track, self.ax_err, self.ax_volt):
            ax.clear()
        self._style_axes()
        self.ax_err.axhline(0.0, color="grey", lw=0.8, alpha=0.6)
        if title:
            self.ax_track.set_title(title, fontsize=9)
        self._lines = None
        self._mode = None

    def clear(self, title: str = ""):
        self._reset_axes(title)
        self.canvas.draw_idle()

    def _ensure_lines(self, show_res1: bool, title: str):
        """Create the line artists once per run; reuse them every frame after."""
        mode = (show_res1, title)
        if self._lines is not None and self._mode == mode:
            return
        self._reset_axes(title)
        self._lines = {
            "r": self.ax_track.plot([], [], lw=1.2, color="#888", ls="--", label="setpoint")[0],
            "y": self.ax_track.plot([], [], lw=1.3, color="#1f77b4", label="measured")[0],
            "e": self.ax_err.plot([], [], lw=1.2, color="#d62728", label="error")[0],
            "v": self.ax_volt.plot([], [], lw=1.2, color="#9467bd", label="command")[0],
        }
        if show_res1:
            self._lines["res1"] = self.ax_err.plot(
                [], [], lw=1.0, color="#2ca02c", alpha=0.85, label="disturbance est.")[0]
        for ax in (self.ax_track, self.ax_err):
            ax.legend(fontsize=7, loc="upper right", framealpha=0.85)
        self._mode = mode

    @classmethod
    def _decimate(cls, seq):
        n = len(seq)
        if n <= cls.MAX_POINTS:
            return list(seq)
        step = n // cls.MAX_POINTS + 1
        return list(seq)[::step]

    def update_live(self, t, r, y, e, v, res1=None, title=""):
        """Redraw from the current run's arrays (already trimmed to a window)."""
        show_res1 = res1 is not None and len(res1) == len(t)
        self._ensure_lines(show_res1, title)

        t_d = self._decimate(t)
        self._lines["r"].set_data(t_d, self._decimate(r))
        self._lines["y"].set_data(t_d, self._decimate(y))
        self._lines["e"].set_data(t_d, self._decimate(e))
        self._lines["v"].set_data(t_d, self._decimate(v))
        if show_res1:
            self._lines["res1"].set_data(t_d, self._decimate(res1))

        for ax in (self.ax_track, self.ax_err, self.ax_volt):
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw_idle()

    def show_comparison(self, runs):
        """
        Overlay the finished A/B runs. `runs` is a list of (label, dict) where
        each dict holds the t/r/y/e/v arrays for one algorithm. One-shot, so
        clearing and replotting is fine here.
        """
        self._reset_axes("A/B comparison — same trajectory, same plant")

        colors = ["#1f77b4", "#ff7f0e"]
        if runs:
            first = runs[0][1]
            self.ax_track.plot(self._decimate(first["t"]), self._decimate(first["r"]),
                               lw=1.2, color="#888", ls="--", label="setpoint")
        for (label, run), c in zip(runs, colors):
            t_d = self._decimate(run["t"])
            self.ax_track.plot(t_d, self._decimate(run["y"]), lw=1.3, color=c, label=label)
            self.ax_err.plot(t_d, self._decimate(run["e"]), lw=1.2, color=c, label=label)
            self.ax_volt.plot(t_d, self._decimate(run["v"]), lw=1.2, color=c, label=label)

        for ax in (self.ax_track, self.ax_err, self.ax_volt):
            ax.legend(fontsize=7, loc="upper right", framealpha=0.85)
        self.canvas.draw_idle()


# ─────────────────────────────────────────────────────────────────────────────
# Tab
# ─────────────────────────────────────────────────────────────────────────────

class ClosedLoopFrame(ttk.Frame):
    """Displacement closed-loop control with switchable algorithm."""

    def __init__(self, parent, moku_tab, display_tab):
        super().__init__(parent, padding=12)
        self.moku_tab = moku_tab
        self.display_tab = display_tab

        # ── Algorithm ────────────────────────────────────────────────
        self.algo_var = tk.StringVar(value=ALGORITHMS[1])

        # ── Plant model ──────────────────────────────────────────────
        default_model = PlantModel()
        self.k_var            = tk.DoubleVar(value=default_model.K_nm_per_V)
        self.tau_var          = tk.DoubleVar(value=default_model.tau_s * 1e3)     # ms
        self.theta_plant_var  = tk.DoubleVar(value=default_model.theta_plant_s * 1e3)
        self.theta_sensor_var = tk.DoubleVar(value=default_model.theta_sensor_s * 1e3)
        self.vmax_var         = tk.DoubleVar(value=default_model.v_max)
        self.vdead_var        = tk.DoubleVar(value=default_model.v_dead)

        # ── Gains ────────────────────────────────────────────────────
        self.kp_var  = tk.DoubleVar(value=0.002)
        self.ki_var  = tk.DoubleVar(value=0.027)
        self.kd_var  = tk.DoubleVar(value=0.0)
        self.dfn_var = tk.DoubleVar(value=20.0)
        self.w0_var  = tk.DoubleVar(value=100.0)

        # ── Trajectory ───────────────────────────────────────────────
        self.wave_var = tk.StringVar(value="Sine")
        self.amp_var  = tk.DoubleVar(value=200.0)
        self.freq_var = tk.DoubleVar(value=0.2)
        self.off_var  = tk.DoubleVar(value=700.0)

        # ── Loop ─────────────────────────────────────────────────────
        self.dt_var       = tk.DoubleVar(value=DEFAULT_DT_MS)
        self.channel_var  = tk.IntVar(value=1)
        self.duration_var = tk.DoubleVar(value=30.0)
        self.log_dir      = tk.StringVar(value=get_downloads_dir())
        self.log_enabled  = tk.BooleanVar(value=True)

        # ── Status ───────────────────────────────────────────────────
        self.status_var  = tk.StringVar(value="Status: idle")
        self.live_var    = tk.StringVar(value="—")
        self.metrics_var = tk.StringVar(value="")

        # ── Worker state ─────────────────────────────────────────────
        self._stop_flag = threading.Event()
        self._worker = None
        self._live = {}           # snapshot written by worker, read by UI pump
        self._lock = threading.Lock()
        self._pump_running = False
        self._trace = {}          # rolling arrays for the live plot
        self._ab_runs = []        # finished A/B runs awaiting display

        self._pid_widgets = []
        self._adrc_widgets = []

        self._build_ui()
        self._load_settings()
        self._on_algo_changed()

    # ==================================================================
    # Settings persistence
    # ==================================================================
    def _setting_vars(self) -> dict:
        return {
            "algorithm": self.algo_var,
            "K_nm_per_V": self.k_var,
            "tau_ms": self.tau_var,
            "theta_plant_ms": self.theta_plant_var,
            "theta_sensor_ms": self.theta_sensor_var,
            "v_max": self.vmax_var,
            "v_dead": self.vdead_var,
            "kp": self.kp_var,
            "ki": self.ki_var,
            "kd": self.kd_var,
            "d_filter_n": self.dfn_var,
            "w0": self.w0_var,
            "waveform": self.wave_var,
            "amplitude_nm": self.amp_var,
            "frequency_hz": self.freq_var,
            "offset_nm": self.off_var,
            "dt_ms": self.dt_var,
            "channel": self.channel_var,
            "duration_s": self.duration_var,
            "log_dir": self.log_dir,
            "log_enabled": self.log_enabled,
        }

    def _load_settings(self):
        data = app_settings.load(SETTINGS_FILE)
        if not data:
            return
        applied = 0
        for key, var in self._setting_vars().items():
            if key not in data:
                continue
            try:                       # one bad key must not lose the rest
                var.set(data[key])
                applied += 1
            except Exception:
                pass
        if applied:
            self.status_var.set(f"Status: restored {applied} saved settings")

    def _save_settings(self, announce: bool = False):
        data = {}
        for key, var in self._setting_vars().items():
            try:
                data[key] = var.get()
            except Exception:
                pass                   # a half-typed field is simply not saved
        ok = app_settings.save(SETTINGS_FILE, data)
        if announce:
            self.status_var.set(
                f"Status: settings saved to {app_settings.settings_path(SETTINGS_FILE)}"
                if ok else "Status: could not write settings file"
            )
        return ok

    # ==================================================================
    # Validated field access
    # ==================================================================
    @staticmethod
    def _num(var, label, lo=None, hi=None):
        """
        Read a numeric Tk variable, raising a ValueError that names the field.

        A DoubleVar whose entry holds "" or "1.2.3" raises TclError on get();
        without this the failure surfaces as a bare Tcl message with no clue
        which of the twenty entry boxes is at fault.
        """
        try:
            value = float(var.get())
        except Exception:
            raise ValueError(f"{label}: not a valid number")
        if not np.isfinite(value):
            raise ValueError(f"{label}: must be finite")
        if lo is not None and value < lo:
            raise ValueError(f"{label}: must be >= {lo:g} (got {value:g})")
        if hi is not None and value > hi:
            raise ValueError(f"{label}: must be <= {hi:g} (got {value:g})")
        return value

    def _read_model(self) -> PlantModel:
        return PlantModel(
            K_nm_per_V=self._num(self.k_var, "K", lo=1e-9),
            tau_s=self._num(self.tau_var, "tau", lo=1e-6) / 1e3,
            theta_plant_s=self._num(self.theta_plant_var, "theta plant", lo=0.0) / 1e3,
            theta_sensor_s=self._num(self.theta_sensor_var, "theta sensor", lo=0.0) / 1e3,
            v_max=self._num(self.vmax_var, "v_max", lo=1e-3, hi=V_MAX_CEILING),
            v_dead=self._num(self.vdead_var, "v_dead", lo=0.0),
        )

    def _read_dt(self) -> float:
        return self._num(self.dt_var, "dt", lo=1.0, hi=10_000.0) / 1e3

    def _read_traj(self) -> TrajectoryGenerator:
        return TrajectoryGenerator(
            waveform=self.wave_var.get(),
            amplitude_nm=self._num(self.amp_var, "amplitude"),
            frequency_hz=self._num(self.freq_var, "frequency", lo=0.0),
            offset_nm=self._num(self.off_var, "offset"),
        )

    def _build_controller(self, algo: str, model: PlantModel, dt: float):
        if algo == "Software PID":
            return SoftwarePID(
                model,
                kp=self._num(self.kp_var, "kp"),
                ki=self._num(self.ki_var, "ki", lo=0.0),
                kd=self._num(self.kd_var, "kd"),
                d_filter_n=self._num(self.dfn_var, "D filter N", lo=0.0),
            )
        return PreviewADRC(model, w0=self._num(self.w0_var, "w0", lo=1e-9), dt=dt)

    # ==================================================================
    # UI
    # ==================================================================
    def _build_ui(self):
        """
        Left column: settings (scrollable) with a pinned action bar.
        Right column: the live plot.

        The settings stack is taller than the window on a 1000 px-high screen,
        so it lives in a scrolling canvas -- otherwise Start/Stop fall off the
        bottom edge and the tab cannot be operated at all. The action bar is
        packed against the bottom first so it is always reachable regardless of
        how far the settings are scrolled.
        """
        self.columnconfigure(0, weight=0, minsize=460)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        right = ttk.Frame(self)
        right.grid(row=0, column=1, sticky="nsew")
        self.plot = LivePlot(right)

        actions = ttk.Frame(left)
        actions.pack(side=tk.BOTTOM, fill=tk.X, pady=(6, 0))
        self._build_control_section(actions)

        canvas = tk.Canvas(left, highlightthickness=0, borderwidth=0, width=445)
        vsb = ttk.Scrollbar(left, orient="vertical", command=canvas.yview)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        canvas.configure(yscrollcommand=vsb.set)

        inner = ttk.Frame(canvas)
        window = canvas.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfigure(window, width=e.width))
        self._bind_mousewheel(canvas)
        inner.columnconfigure(0, weight=1)

        ttk.Label(inner, text="Closed-Loop Control", font=("Segoe UI", 13, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            inner,
            text=(
                "Feedback from the interferometer (uMD tab), command out as a DC\n"
                "level on the Moku. Independent of the Moku tab's hardware PID —\n"
                "only one may drive Output 1."
            ),
            font=("Segoe UI", 8), foreground="grey", justify="left",
        ).grid(row=1, column=0, sticky="w", pady=(2, 8))

        row = 2
        row = self._build_algorithm_section(inner, row)
        row = self._build_model_section(inner, row)
        row = self._build_gains_section(inner, row)
        row = self._build_trajectory_section(inner, row)
        self._build_loop_section(inner, row)

    @staticmethod
    def _bind_mousewheel(canvas):
        """Scroll the settings column while the pointer is over it."""
        def on_wheel(event):
            if event.num == 4:            # X11 scroll up
                canvas.yview_scroll(-1, "units")
            elif event.num == 5:          # X11 scroll down
                canvas.yview_scroll(1, "units")
            else:                         # macOS / Windows
                canvas.yview_scroll(-1 * int(event.delta), "units")

        def bind(_e):
            canvas.bind_all("<MouseWheel>", on_wheel)
            canvas.bind_all("<Button-4>", on_wheel)
            canvas.bind_all("<Button-5>", on_wheel)

        def unbind(_e):
            canvas.unbind_all("<MouseWheel>")
            canvas.unbind_all("<Button-4>")
            canvas.unbind_all("<Button-5>")

        canvas.bind("<Enter>", bind)
        canvas.bind("<Leave>", unbind)

    def _grid_fields(self, frame, fields, start_row=0):
        widgets = []
        for r, (label, var, hint) in enumerate(fields, start=start_row):
            lbl = ttk.Label(frame, text=label)
            lbl.grid(row=r, column=0, sticky="e", padx=(0, 4), pady=1)
            entry = ttk.Entry(frame, textvariable=var, width=11)
            entry.grid(row=r, column=1, sticky="w", pady=1)
            hint_lbl = ttk.Label(frame, text=hint, font=("Segoe UI", 7), foreground="grey")
            hint_lbl.grid(row=r, column=2, sticky="w", padx=(6, 0))
            widgets.extend([lbl, entry, hint_lbl])
        return widgets

    def _build_algorithm_section(self, parent, row):
        f = ttk.LabelFrame(parent, text="Algorithm", padding=6)
        f.grid(row=row, column=0, sticky="ew", pady=(0, 6))

        ttk.Label(f, text="Control law:").grid(row=0, column=0, sticky="e", padx=(0, 4))
        combo = tb.Combobox(f, textvariable=self.algo_var, values=list(ALGORITHMS),
                            state="readonly", width=17)
        combo.grid(row=0, column=1, sticky="w")
        combo.bind("<<ComboboxSelected>>", self._on_algo_changed)

        self._algo_note = ttk.Label(f, text="", font=("Segoe UI", 8),
                                    foreground="grey", justify="left")
        self._algo_note.grid(row=1, column=0, columnspan=3, sticky="w", pady=(4, 0))
        return row + 1

    def _build_model_section(self, parent, row):
        f = ttk.LabelFrame(parent, text="Plant model (FOPDT)", padding=6)
        f.grid(row=row, column=0, sticky="ew", pady=(0, 6))

        self._grid_fields(f, [
            ("K (nm/V):",          self.k_var,            "Static gain"),
            ("tau (ms):",          self.tau_var,          "Time constant"),
            ("theta plant (ms):",  self.theta_plant_var,  "Actuator dead time"),
            ("theta sensor (ms):", self.theta_sensor_var, "Feedback dead time"),
            ("v_max (V):",         self.vmax_var,         "Protects piezo"),
            ("v_dead (V):",        self.vdead_var,        "Dead-band"),
        ])
        self._identify_btn = tb.Button(f, text="Identify plant (step)", bootstyle="secondary",
                                       command=self._identify_plant)
        self._identify_btn.grid(row=6, column=0, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Label(f, text="Open-loop step, fits K/tau/theta",
                  font=("Segoe UI", 7), foreground="grey").grid(
            row=6, column=2, sticky="w", padx=(6, 0), pady=(6, 0))
        return row + 1

    def _build_gains_section(self, parent, row):
        f = ttk.LabelFrame(parent, text="Gains", padding=6)
        f.grid(row=row, column=0, sticky="ew", pady=(0, 6))

        self._pid_widgets = self._grid_fields(f, [
            ("kp (V/nm):",   self.kp_var,  "Proportional"),
            ("ki (V/nm/s):", self.ki_var,  "Integral"),
            ("kd:",          self.kd_var,  "Derivative (on measurement)"),
            ("D filter N:",  self.dfn_var, "Derivative filter"),
        ])
        self._adrc_widgets = self._grid_fields(f, [
            ("w0 (rad/s):", self.w0_var, "Observer bandwidth"),
        ], start_row=4)

        tb.Button(f, text="Auto-tune (IMC)", bootstyle="secondary",
                  command=self._auto_tune).grid(row=5, column=0, columnspan=2,
                                                sticky="w", pady=(6, 0))
        ttk.Label(f, text="Gains from the plant model above",
                  font=("Segoe UI", 7), foreground="grey").grid(
            row=5, column=2, sticky="w", padx=(6, 0), pady=(6, 0))
        return row + 1

    def _build_trajectory_section(self, parent, row):
        f = ttk.LabelFrame(parent, text="Reference trajectory (generated in Python)", padding=6)
        f.grid(row=row, column=0, sticky="ew", pady=(0, 6))

        ttk.Label(f, text="Waveform:").grid(row=0, column=0, sticky="e", padx=(0, 4), pady=1)
        tb.Combobox(f, textvariable=self.wave_var, values=list(WAVEFORMS),
                    state="readonly", width=11).grid(row=0, column=1, sticky="w", pady=1)
        ttk.Label(f, text="Preview needs a Python-owned clock",
                  font=("Segoe UI", 7), foreground="grey").grid(row=0, column=2,
                                                                sticky="w", padx=(6, 0))
        self._grid_fields(f, [
            ("Amplitude (nm):", self.amp_var,  "Target displacement"),
            ("Frequency (Hz):", self.freq_var, "0 = static setpoint"),
            ("Offset (nm):",    self.off_var,  "Piezo is unipolar: keep r >= 0"),
        ], start_row=1)
        return row + 1

    def _build_loop_section(self, parent, row):
        f = ttk.LabelFrame(parent, text="Loop", padding=6)
        f.grid(row=row, column=0, sticky="ew", pady=(0, 6))

        self._grid_fields(f, [
            ("dt (ms):",        self.dt_var,       "Controller period"),
            ("Output channel:", self.channel_var,  "Moku output"),
            ("Duration (s):",   self.duration_var, "0 = until stopped"),
        ])
        tb.Checkbutton(f, text="Log to CSV", variable=self.log_enabled).grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Entry(f, textvariable=self.log_dir, width=34).grid(
            row=4, column=0, columnspan=2, sticky="w", pady=1)
        tb.Button(f, text="Browse…", bootstyle="secondary", command=self._pick_folder).grid(
            row=4, column=2, sticky="w", padx=(6, 0))
        return row + 1

    def _build_control_section(self, f):
        f.columnconfigure(4, weight=1)

        self._start_btn = tb.Button(f, text="Start", bootstyle="primary", command=self.start)
        self._start_btn.grid(row=0, column=0, sticky="w")
        self._ab_btn = tb.Button(f, text="Run A/B", bootstyle="info", command=self.start_ab)
        self._ab_btn.grid(row=0, column=1, sticky="w", padx=(6, 0))
        self._stop_btn = tb.Button(f, text="Stop", bootstyle="danger",
                                   command=self.stop, state="disabled")
        self._stop_btn.grid(row=0, column=2, sticky="w", padx=(6, 0))
        tb.Button(f, text="Save settings", bootstyle="secondary",
                  command=lambda: self._save_settings(announce=True)).grid(
            row=0, column=3, sticky="w", padx=(6, 0))

        ttk.Label(f, textvariable=self.status_var, font=("Segoe UI", 9),
                  wraplength=410, justify="left").grid(
            row=1, column=0, columnspan=4, sticky="w", pady=(6, 0))
        ttk.Label(f, textvariable=self.live_var, font=("Consolas", 8),
                  foreground="teal").grid(row=2, column=0, columnspan=4, sticky="w", pady=(2, 0))
        ttk.Label(f, textvariable=self.metrics_var, font=("Segoe UI", 8),
                  foreground="grey", wraplength=410, justify="left").grid(
            row=3, column=0, columnspan=4, sticky="w", pady=(2, 0))

    # ==================================================================
    # UI callbacks
    # ==================================================================
    def _on_algo_changed(self, _event=None):
        is_pid = self.algo_var.get() == "Software PID"
        for w in self._pid_widgets:
            w.grid() if is_pid else w.grid_remove()
        for w in self._adrc_widgets:
            w.grid_remove() if is_pid else w.grid()

        self._algo_note.config(text=(
            "Classical discrete PI(D): reacts to error already made.\n"
            "Anti-windup, derivative filtered and on the measurement."
            if is_pid else
            "Exact-inversion feedforward from the known future setpoint,\n"
            "corrected by a disturbance-and-rate observer. Only w0 tunes it."
        ))

    def _pick_folder(self):
        chosen = select_folder_native("Select CSV log folder")
        if chosen:
            self.log_dir.set(chosen)

    def _auto_tune(self):
        try:
            gains = imc_tune(self._read_model(), self._read_dt())
        except Exception as e:
            messagebox.showerror("Auto-tune failed", str(e))
            return

        self.kp_var.set(round(gains["kp"], 9))
        self.ki_var.set(round(gains["ki"], 9))
        self.kd_var.set(round(gains["kd"], 9))
        self.dfn_var.set(gains["d_filter_n"])
        self.w0_var.set(round(gains["w0"], 4))
        self.status_var.set(
            f"Status: IMC gains applied — kp={gains['kp']:.3g}, "
            f"ki={gains['ki']:.3g}, w0={gains['w0']:.4g}"
        )

    # ==================================================================
    # Plant identification
    # ==================================================================
    def _identify_plant(self):
        if self._busy():
            return
        ok, why = self._preflight()
        if not ok:
            messagebox.showwarning("Cannot identify", why)
            return
        try:
            v_max = self._num(self.vmax_var, "v_max", lo=1e-3, hi=V_MAX_CEILING)
            v_dead = self._num(self.vdead_var, "v_dead", lo=0.0)
            channel = int(self._num(self.channel_var, "Output channel", lo=1, hi=4))
        except ValueError as e:
            messagebox.showerror("Invalid settings", str(e))
            return

        # Snapshot for the worker: Tk variables are main-thread only.
        self._identify_v_max = v_max
        self._identify_v_dead = v_dead

        v_step = min(1.0, v_max * 0.4)
        if not messagebox.askokcancel(
            "Identify plant",
            f"This drives Output {channel} open-loop:\n"
            f"  0 V for 1 s, then {v_step:.2f} V for 3 s, then back to 0 V.\n\n"
            "Make sure the piezo can move freely. Continue?",
        ):
            return

        self._begin_run()
        self.status_var.set("Status: identifying plant…")
        self._worker = threading.Thread(
            target=self._run_identify, args=(v_step, channel), daemon=True)
        self._worker.start()
        self._start_pump()

    def _run_identify(self, v_step: float, channel: int):
        sample_dt = 0.02        # 50 Hz — comfortably under the ~610 Hz sensor rate
        settle_s, step_s = 1.0, 3.0
        t_rel, y_rel = [], []

        try:
            self.moku_tab.begin_software_control(channel)

            self.moku_tab.set_dc_voltage(0.0, channel)
            baseline = self._collect(settle_s, sample_dt)
            if self._stop_flag.is_set():
                return self._finish("Status: identification stopped by user.")
            if len(baseline) < 3:
                return self._finish("Status: identification failed — no measurements arrived.")
            y0 = float(np.mean(baseline[-10:]))

            self.moku_tab.set_dc_voltage(v_step, channel)
            t0 = time.perf_counter()
            t_end = t0 + step_s
            while time.perf_counter() < t_end and not self._stop_flag.is_set():
                s = self.display_tab.read_latest()
                if s is not None:
                    t_rel.append(time.perf_counter() - t0)
                    y_rel.append(s[1])
                time.sleep(sample_dt)

            self.moku_tab.set_dc_voltage(0.0, channel)

            if self._stop_flag.is_set():
                return self._finish("Status: identification stopped by user.")
            if len(t_rel) < 8:
                return self._finish(
                    f"Status: identification failed — only {len(t_rel)} samples "
                    f"in {step_s:g} s. Is the uMD stream running?")

            # Did the piezo actually respond? Fitting an FOPDT to a flat trace
            # returns numbers that look plausible but mean nothing, and those
            # numbers would then drive the feedforward.
            travel = float(np.mean(y_rel[-max(len(y_rel) // 5, 3):]) - y0)
            noise = float(np.std(baseline)) if len(baseline) > 2 else 0.0
            if abs(travel) < max(5.0 * noise, 10.0):
                return self._finish(
                    f"Status: identification failed — displacement moved only "
                    f"{travel:.1f} nm for {v_step:.2f} V (baseline noise "
                    f"{noise:.1f} nm). Check the piezo drive and the beam path.")
            if travel < 0:
                return self._finish(
                    f"Status: identification failed — displacement moved "
                    f"{travel:.1f} nm, i.e. backwards for a positive step. "
                    "Check the wiring polarity or the interferometer sign.")

            K, tau, theta = fit_fopdt_step(t_rel, y_rel, v_step, y0_nm=y0)
        except Exception as e:
            self._safe_zero(channel)
            return self._finish(f"Status: identification error — {e}")
        finally:
            try:
                self.moku_tab.end_software_control(channel)
            except Exception:
                pass

        stroke = K * max(0.0, self._identify_v_max - self._identify_v_dead)
        self._finish(
            f"Status: identified K={K:.1f} nm/V, tau={tau*1e3:.2f} ms, "
            f"theta={theta*1e3:.2f} ms  (step moved {travel:.0f} nm; "
            f"full stroke ≈ {stroke:.0f} nm)",
            model=(K, tau, theta),
        )

    def _collect(self, seconds: float, sample_dt: float) -> list:
        out = []
        t_end = time.perf_counter() + seconds
        while time.perf_counter() < t_end and not self._stop_flag.is_set():
            s = self.display_tab.read_latest()
            if s is not None:
                out.append(s[1])
            time.sleep(sample_dt)
        return out

    # ==================================================================
    # Control loop
    # ==================================================================
    def _busy(self) -> bool:
        if self._worker is not None and self._worker.is_alive():
            messagebox.showwarning("Busy", "A run is already in progress.")
            return True
        return False

    def _preflight(self) -> tuple[bool, str]:
        if not self.moku_tab.is_connected():
            return False, "Connect to the Moku:Go first (Moku:Go Waveform tab)."
        if self.display_tab.read_latest() is None:
            return False, (
                "No displacement measurement yet. Launch uMD_GUI from the uMD GUI tab "
                "and confirm the displacement readout is updating."
            )
        return True, ""

    def _prepare(self):
        """Validate everything and return (model, traj, dt, cfg), or None."""
        ok, why = self._preflight()
        if not ok:
            messagebox.showwarning("Cannot start", why)
            return None
        try:
            model = self._read_model()
            dt = self._read_dt()
            traj = self._read_traj()
            cfg = {
                "channel": int(self._num(self.channel_var, "Output channel", lo=1, hi=4)),
                "duration": self._num(self.duration_var, "Duration", lo=0.0),
                "log": bool(self.log_enabled.get()),
                "log_dir": self.log_dir.get(),
            }
        except ValueError as e:
            messagebox.showerror("Invalid settings", str(e))
            return None
        except Exception as e:
            messagebox.showerror("Invalid settings", str(e))
            return None

        peak = abs(traj.offset_nm) + abs(traj.amplitude_nm)
        if peak > model.stroke_nm:
            if not messagebox.askokcancel(
                "Setpoint exceeds stroke",
                f"Peak setpoint {peak:.0f} nm is beyond the {model.stroke_nm:.0f} nm "
                f"reachable at v_max={model.v_max:g} V. The output will saturate.\n\nContinue?",
            ):
                return None
        trough = traj.offset_nm - abs(traj.amplitude_nm)
        if traj.waveform != "Pulse" and trough < -1e-9:
            if not messagebox.askokcancel(
                "Setpoint goes negative",
                f"The trajectory dips to {trough:.0f} nm, but the piezo is unipolar "
                f"(v >= 0 means y >= 0), so negative displacement is unreachable.\n\n"
                f"Raise the offset to at least {abs(traj.amplitude_nm):.0f} nm.\n\nContinue anyway?",
            ):
                return None
        if traj.bandwidth_warning(model):
            self.status_var.set(
                f"Status: note — {traj.frequency_hz:.3g} Hz exceeds the plant's "
                f"{1.0/(6.2832*model.tau_s):.3g} Hz bandwidth; amplitude will be attenuated."
            )
        return model, traj, dt, cfg

    def start(self):
        if self._busy():
            return
        prepared = self._prepare()
        if prepared is None:
            return
        model, traj, dt, cfg = prepared
        try:
            controller = self._build_controller(self.algo_var.get(), model, dt)
        except ValueError as e:
            messagebox.showerror("Invalid settings", str(e))
            return

        self._begin_run()
        self._worker = threading.Thread(
            target=self._run_single, args=(controller, traj, model, dt, cfg), daemon=True)
        self._worker.start()
        self._start_pump()

    @staticmethod
    def _ab_settle_seconds(model: PlantModel) -> float:
        """How long to hold 0 V between the halves of an A/B run."""
        return max(AB_SETTLE_S, AB_SETTLE_TAUS * model.tau_s)

    def start_ab(self):
        """Run both algorithms back to back over the same trajectory."""
        if self._busy():
            return
        prepared = self._prepare()
        if prepared is None:
            return
        model, traj, dt, cfg = prepared

        if cfg["duration"] <= 0.0:
            messagebox.showwarning(
                "Duration required",
                "An A/B comparison needs a fixed duration so both halves run "
                "for the same time. Set Duration to a positive value.")
            return
        try:
            controllers = [(a, self._build_controller(a, model, dt)) for a in ALGORITHMS]
        except ValueError as e:
            messagebox.showerror("Invalid settings", str(e))
            return

        settle = self._ab_settle_seconds(model)
        if not messagebox.askokcancel(
            "A/B comparison",
            f"Runs both algorithms for {cfg['duration']:g} s each "
            f"({2*cfg['duration'] + settle:.0f} s total), with the output held "
            f"at 0 V for {settle:.1f} s in between so both start from rest.\n\n"
            "Continue?",
        ):
            return

        self._begin_run()
        self._worker = threading.Thread(
            target=self._run_ab, args=(controllers, traj, model, dt, cfg), daemon=True)
        self._worker.start()
        self._start_pump()

    def stop(self):
        self._stop_flag.set()
        self.status_var.set("Status: stopping…")

    # ── worker bodies ─────────────────────────────────────────────────
    def _run_single(self, controller, traj, model, dt, cfg):
        channel = cfg["channel"]
        try:
            self.moku_tab.begin_software_control(channel)
            run, reason = self._drive(controller, traj, model, dt, cfg)
        except Exception as e:
            run, reason = None, f"Status: loop error — {e}"
        finally:
            self._safe_zero(channel)
            try:
                self.moku_tab.end_software_control(channel)
            except Exception:
                pass
        if run is not None and run["n"]:
            reason += (f"  [{controller.name}: RMSE {run['rmse']:.1f} nm, "
                       f"settled {run['settled_rmse']:.1f} nm]")
        self._finish(reason)

    def _run_ab(self, controllers, traj, model, dt, cfg):
        channel = cfg["channel"]
        runs = []
        reason = "Status: A/B comparison finished."
        try:
            self.moku_tab.begin_software_control(channel)
            for i, (name, controller) in enumerate(controllers):
                if self._stop_flag.is_set():
                    reason = "Status: A/B stopped by user."
                    break
                with self._lock:
                    self._live["phase"] = f"{name}  ({i+1}/{len(controllers)})"
                run, why = self._drive(controller, traj, model, dt, cfg)
                if run is not None and run["n"]:
                    runs.append((name, run))
                if "stopped" in why or "stale" in why or "error" in why:
                    reason = why
                    break
                # Settle back to rest so the second half starts as the first did.
                if i + 1 < len(controllers):
                    self._hold_at_zero(self._ab_settle_seconds(model), dt, channel)
        except Exception as e:
            reason = f"Status: A/B error — {e}"
        finally:
            self._safe_zero(channel)
            try:
                self.moku_tab.end_software_control(channel)
            except Exception:
                pass

        with self._lock:
            self._live.pop("phase", None)
            self._ab_runs = runs
        self._finish(reason, ab=runs)

    def _hold_at_zero(self, seconds: float, dt: float, channel: int):
        """
        Actively command 0 V for `seconds` between the halves of an A/B run.

        This keeps ticking at the loop rate rather than sleeping. Sleeping
        leaves the measurement untouched for the whole settle, so the next run
        opens by reading a sample that is already `seconds` old and trips its
        own staleness watchdog before it has taken a single step.
        """
        with self._lock:
            self._live["phase"] = f"settling {seconds:.1f}s at 0 V"
        t_end = time.perf_counter() + seconds
        while time.perf_counter() < t_end and not self._stop_flag.is_set():
            try:
                self.moku_tab.set_dc_voltage(0.0, channel)
            except Exception:
                break
            time.sleep(max(dt, 0.005))

    def _drive(self, controller, traj, model, dt, cfg):
        """
        Run one controller for cfg["duration"] seconds.

        Returns (run_dict_or_None, status_message). Never touches Tk.
        """
        duration = cfg["duration"]
        is_preview = isinstance(controller, PreviewADRC)
        watchdog_s = max(WATCHDOG_TICKS * dt, WATCHDOG_MIN_S)
        controller.reset()

        writer = log_file = None
        T, R, Y, E, V, RES = [], [], [], [], [], []
        jitter_warned = False
        reason = "Status: finished."

        with self._lock:
            self._trace = {}

        try:
            if cfg["log"]:
                os.makedirs(cfg["log_dir"], exist_ok=True)
                slug = "preview_adrc" if is_preview else "software_pid"
                path = os.path.join(
                    cfg["log_dir"],
                    f"closedloop_{slug}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
                log_file = open(path, "w", newline="", encoding="utf-8")
                writer = csv.writer(log_file)
                writer.writerow([
                    f"# algorithm={controller.name}",
                    f"K={model.K_nm_per_V}", f"tau_s={model.tau_s}",
                    f"theta_plant_s={model.theta_plant_s}",
                    f"theta_sensor_s={model.theta_sensor_s}",
                    f"dt_s={dt}", f"waveform={traj.waveform}",
                    f"amp_nm={traj.amplitude_nm}", f"freq_hz={traj.frequency_hz}",
                    f"offset_nm={traj.offset_nm}",
                ])
                writer.writerow(["t_s", "r_nm", "y_nm", "e_nm", "v_V", "dt_meas_s", "res1_nm"])
                with self._lock:
                    self._live["log_path"] = path

            t0 = time.perf_counter()
            next_tick = t0
            t_prev = t0

            while not self._stop_flag.is_set():
                if duration > 0.0 and (time.perf_counter() - t0) >= duration:
                    break

                sample = self.display_tab.read_latest()
                if sample is None:
                    reason = "Status: stopped — measurement stream went away."
                    break
                sample_ts, y_meas = sample
                age = time.time() - sample_ts
                if age > watchdog_s:
                    reason = (f"Status: stopped — measurement stale by {age*1e3:.0f} ms "
                              f"(watchdog limit {watchdog_s*1e3:.0f} ms).")
                    break

                now = time.perf_counter()
                t = now - t0
                dt_meas = now - t_prev
                t_prev = now
                dt_eff = dt_meas if 0.0 < dt_meas < 10.0 * dt else dt

                r = traj.r(t)
                if is_preview:
                    t_d, t_d1 = controller.preview_times(t)
                    v = controller.update(traj.r(t_d), traj.r(t_d1), y_meas, dt_eff)
                    res1 = controller.diagnostics()["res1"]
                else:
                    v = controller.update(r, y_meas, dt_eff)
                    res1 = float("nan")

                self.moku_tab.set_dc_voltage(v, cfg["channel"])

                e = r - y_meas
                T.append(t); R.append(r); Y.append(y_meas); E.append(e); V.append(v)
                RES.append(res1)

                if writer is not None:
                    writer.writerow([f"{t:.6f}", f"{r:.4f}", f"{y_meas:.4f}",
                                     f"{e:.4f}", f"{v:.6f}", f"{dt_meas:.6f}",
                                     f"{res1:.4f}" if is_preview else ""])

                if (not jitter_warned and len(T) > 5
                        and abs(dt_meas - dt) > JITTER_WARN_FRAC * dt):
                    jitter_warned = True
                    with self._lock:
                        self._live["jitter"] = dt_meas

                n = len(E)
                rmse = float(np.sqrt(np.mean(np.square(E))))
                with self._lock:
                    self._live.update({
                        "t": t, "r": r, "y": y_meas, "e": e, "v": v,
                        "dt_meas": dt_meas, "n": n, "rmse": rmse,
                        "max_err": float(np.max(np.abs(E))),
                        "algo": controller.name,
                    })
                    win = max(1, int(PLOT_WINDOW_S / max(dt, 1e-6)))
                    self._trace = {
                        "t": T[-win:], "r": R[-win:], "y": Y[-win:],
                        "e": E[-win:], "v": V[-win:],
                        "res1": RES[-win:] if is_preview else None,
                        "title": controller.name,
                    }

                next_tick += dt
                sleep_for = next_tick - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_tick = time.perf_counter()   # fell behind: resync

            if self._stop_flag.is_set():
                reason = "Status: stopped by user."
        finally:
            if log_file is not None:
                try:
                    log_file.close()
                except Exception:
                    pass

        if not E:
            return None, reason

        e_arr = np.asarray(E)
        half = len(e_arr) // 2
        return {
            "name": controller.name,
            "t": np.asarray(T), "r": np.asarray(R), "y": np.asarray(Y),
            "e": e_arr, "v": np.asarray(V), "n": len(e_arr),
            "rmse": float(np.sqrt(np.mean(e_arr ** 2))),
            "settled_rmse": float(np.sqrt(np.mean(e_arr[half:] ** 2))) if half else float("nan"),
            "max_err": float(np.max(np.abs(e_arr))),
            "iae": float(np.sum(np.abs(e_arr)) * dt),
        }, reason

    def _safe_zero(self, channel: int):
        try:
            self.moku_tab.set_dc_voltage(0.0, channel)
        except Exception:
            pass

    def _finish(self, message: str, model=None, ab=None):
        """Publish the worker's result for the main-thread pump to apply.

        Tk objects may only be touched from the thread running the event loop,
        and that includes `after()` itself (it registers a Tcl command). The
        pump already polls on the main thread, so the worker just leaves its
        result here and returns.
        """
        with self._lock:
            self._live["final"] = message
            if model is not None:
                self._live["final_model"] = model
            if ab is not None:
                self._live["final_ab"] = ab

    # ==================================================================
    # UI pump
    # ==================================================================
    def _begin_run(self):
        self._stop_flag.clear()
        with self._lock:
            self._live = {}
            self._trace = {}
            self._ab_runs = []
        self.plot.clear()
        self._set_running(True)

    def _set_running(self, running: bool):
        state = "disabled" if running else "normal"
        for btn in (self._start_btn, self._ab_btn, self._identify_btn):
            btn.config(state=state)
        self._stop_btn.config(state="normal" if running else "disabled")

    def _start_pump(self):
        if not self._pump_running:
            self._pump_running = True
            self._pump()

    def _pump(self):
        with self._lock:
            live = dict(self._live)
            trace = dict(self._trace) if self._trace else None

        if live.get("n"):
            self.live_var.set(
                f"t={live['t']:7.2f}s  r={live['r']:9.1f}  y={live['y']:9.1f}  "
                f"e={live['e']:8.1f} nm  v={live['v']:6.3f} V"
            )
            msg = (f"{live.get('algo','')}   n={live['n']}   "
                   f"RMSE={live['rmse']:.1f} nm   max|e|={live['max_err']:.1f} nm   "
                   f"dt={live['dt_meas']*1e3:.1f} ms")
            if live.get("phase"):
                msg = f"[{live['phase']}]  " + msg
            if "jitter" in live:
                msg += (f"\n⚠ tick drifted to {live['jitter']*1e3:.0f} ms vs nominal "
                        f"{float(self.dt_var.get()):.0f} ms — gains were discretised for the nominal dt")
            if "log_path" in live:
                msg += f"\n{os.path.basename(live['log_path'])}"
            self.metrics_var.set(msg)

        if trace and trace.get("t"):
            self.plot.update_live(
                trace["t"], trace["r"], trace["y"], trace["e"], trace["v"],
                res1=trace.get("res1"), title=trace.get("title", ""))

        if self._worker is not None and self._worker.is_alive():
            self.after(200, self._pump)
            return

        # Worker has exited: apply whatever it left behind, on this thread.
        self._pump_running = False
        with self._lock:
            final = self._live.pop("final", None)
            fitted = self._live.pop("final_model", None)
            ab = self._live.pop("final_ab", None)

        if fitted is not None:
            K, tau, theta = fitted
            self.k_var.set(round(K, 3))
            self.tau_var.set(round(tau * 1e3, 4))
            self.theta_plant_var.set(round(theta * 1e3, 4))
        if ab:
            self.plot.show_comparison(ab)
            self.metrics_var.set(self._format_ab(ab))
        if final is not None:
            self.status_var.set(final)
        self._save_settings()
        self._set_running(False)

    @staticmethod
    def _format_ab(runs) -> str:
        head = f"{'algorithm':16s}{'RMSE':>10s}{'settled':>10s}{'max|e|':>10s}{'IAE':>11s}"
        lines = [head]
        for name, r in runs:
            lines.append(f"{name:16s}{r['rmse']:9.1f}{r['settled_rmse']:10.1f}"
                         f"{r['max_err']:10.1f}{r['iae']:11.1f}")
        if len(runs) == 2:
            a, b = runs[0][1], runs[1][1]
            if a["settled_rmse"] > 0:
                ratio = a["settled_rmse"] / max(b["settled_rmse"], 1e-12)
                better = runs[1][0] if ratio > 1 else runs[0][0]
                factor = ratio if ratio > 1 else 1.0 / max(ratio, 1e-12)
                lines.append(f"→ {better} better by {factor:.1f}× on settled RMSE")
        return "\n".join(lines)

    # ==================================================================
    # Lifecycle
    # ==================================================================
    def shutdown(self):
        """Stop the loop and zero the output. Called on app close."""
        self._stop_flag.set()
        worker = self._worker
        if worker is not None and worker.is_alive():
            worker.join(timeout=3.0)
        try:
            channel = int(self.channel_var.get())
        except Exception:
            channel = 1
        self._safe_zero(channel)
        self._save_settings()
