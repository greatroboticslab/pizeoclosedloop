"""
Integration tests for closed_loop.py — exercises the real worker thread,
watchdog, CSV logging and hardware hand-off against a simulated piezo.

These need a display (Tk). They are skipped automatically where there is none.

Run:  python3 -m pytest test_closed_loop.py -v
"""
from __future__ import annotations

import csv
import glob
import os
import time

import numpy as np
import pytest

ttk = pytest.importorskip("tkinter.ttk")
tb = pytest.importorskip("ttkbootstrap")

from control import PlantModel, SimulatedPlant

MODEL = PlantModel(K_nm_per_V=410.0, tau_s=0.08, theta_plant_s=0.0, v_max=5.0)
DT = 0.02
TIMEOUT_S = 25.0


@pytest.fixture(autouse=True)
def isolated_settings(tmp_path_factory, monkeypatch):
    """Keep the tab's JSON settings out of the developer's real home dir."""
    import app_settings
    d = tmp_path_factory.mktemp("settings")
    monkeypatch.setattr(app_settings, "settings_dir", lambda: d)
    return d


@pytest.fixture(scope="module")
def root():
    try:
        r = tb.Window(themename="flatly")
    except Exception as e:                      # headless CI
        pytest.skip(f"no display: {e}")
    r.withdraw()
    yield r
    try:
        r.destroy()
    except Exception:
        pass


class SimRig:
    """
    Stands in for both the Moku tab and the uMD tab at once, closing the loop
    in RAM: a voltage written here immediately advances the simulated piezo,
    and the next measurement read reflects it.
    """

    def __init__(self, noise_nm=2.0, hysteresis=False, stale=False):
        self.plant = SimulatedPlant(MODEL, noise_nm=noise_nm,
                                    bw_enable=hysteresis, bw_D=100.0)
        self.plant.reset(DT)
        self.y = 0.0
        self.ts = time.time()
        self.stale = stale
        self.began = self.ended = 0
        self.last_v = None

    # ── actuator side (MokuWaveformFrame) ──
    def is_connected(self): return True
    def begin_software_control(self, ch=1): self.began += 1
    def end_software_control(self, ch=1): self.ended += 1

    def set_dc_voltage(self, v, ch=1):
        self.last_v = v
        _, self.y = self.plant.step(v, DT)
        self.ts = time.time()

    # ── sensor side (DisplayFrame) ──
    def read_latest(self):
        if self.stale:
            return (time.time() - 10.0, 500.0)
        return (self.ts, self.y)


def make_frame(root, tmp_path, algo):
    from closed_loop import ClosedLoopFrame
    f = ClosedLoopFrame(ttk.Notebook(root), moku_tab=None, display_tab=None)
    f.dt_var.set(DT * 1e3)
    f.wave_var.set("Sine")
    f.amp_var.set(300.0)
    f.freq_var.set(0.3)
    f.off_var.set(700.0)          # unipolar piezo: reference must stay >= 0
    f.log_dir.set(str(tmp_path))
    f.log_enabled.set(True)
    f.algo_var.set(algo)
    f._on_algo_changed()
    f._auto_tune()
    return f


def run_loop(root, f, rig, seconds):
    f.moku_tab = rig
    f.display_tab = rig
    f.duration_var.set(seconds)
    f.start()
    t0 = time.time()
    while f._worker.is_alive() and time.time() - t0 < TIMEOUT_S:
        root.update()
        time.sleep(0.01)
    f._worker.join(timeout=5)
    for _ in range(5):            # let the main-thread pump apply the result
        root.update()
        time.sleep(0.05)
    assert not f._worker.is_alive(), "worker did not terminate"
    return dict(f._live)


def settled_rmse(csv_path, frac=0.5):
    rows = list(csv.reader(open(csv_path)))[2:]
    e = np.array([float(r[3]) for r in rows])
    return float(np.sqrt(np.mean(e[int(len(e) * frac):] ** 2)))


@pytest.mark.parametrize("algo", ["Software PID", "Preview + ADRC"])
def test_loop_runs_and_releases_hardware(root, tmp_path, algo):
    f = make_frame(root, tmp_path, algo)
    rig = SimRig()

    live = run_loop(root, f, rig, 4.0)

    assert live["n"] > 50, f"only {live.get('n')} ticks"
    assert rig.began == 1 and rig.ended == 1, "hardware hand-off unbalanced"
    assert rig.last_v == 0.0, "output was not zeroed on exit"
    assert f.status_var.get().startswith("Status: finished."), f.status_var.get()
    assert str(f._start_btn["state"]) == "normal"
    assert str(f._stop_btn["state"]) == "disabled"


def test_csv_log_is_well_formed_and_voltage_stays_in_bounds(root, tmp_path):
    f = make_frame(root, tmp_path, "Preview + ADRC")
    run_loop(root, f, SimRig(), 4.0)

    logs = glob.glob(str(tmp_path / "*.csv"))
    assert len(logs) == 1
    rows = list(csv.reader(open(logs[0])))
    assert rows[0][0].startswith("# algorithm=Preview + ADRC")
    assert rows[1] == ["t_s", "r_nm", "y_nm", "e_nm", "v_V", "dt_meas_s", "res1_nm"]
    assert len(rows) - 2 > 50

    v = [float(r[4]) for r in rows[2:]]
    assert all(0.0 <= x <= MODEL.v_max for x in v), "voltage escaped [0, v_max]"


def test_preview_beats_pid_through_the_real_worker(root, tmp_path):
    """The end-to-end claim: same plumbing, same plant, only the law differs."""
    results = {}
    for algo, slug in (("Software PID", "software_pid"), ("Preview + ADRC", "preview_adrc")):
        d = tmp_path / slug
        d.mkdir()
        f = make_frame(root, d, algo)
        run_loop(root, f, SimRig(), 6.0)
        results[algo] = settled_rmse(glob.glob(str(d / "*.csv"))[0])

    assert results["Preview + ADRC"] < 0.5 * results["Software PID"], results


def test_preview_rejects_hysteresis_through_the_real_worker(root, tmp_path):
    results = {}
    for algo, slug in (("Software PID", "software_pid"), ("Preview + ADRC", "preview_adrc")):
        d = tmp_path / slug
        d.mkdir()
        f = make_frame(root, d, algo)
        run_loop(root, f, SimRig(hysteresis=True), 10.0)
        results[algo] = settled_rmse(glob.glob(str(d / "*.csv"))[0])

    assert results["Preview + ADRC"] < results["Software PID"], results


def test_watchdog_stops_on_stale_measurement(root, tmp_path):
    f = make_frame(root, tmp_path, "Preview + ADRC")
    rig = SimRig(stale=True)

    run_loop(root, f, rig, 5.0)

    assert "stale" in f.status_var.get(), f.status_var.get()
    assert rig.ended == 1, "must hand Output 1 back even on a watchdog trip"
    assert rig.last_v == 0.0, "must zero the output on a watchdog trip"
    assert str(f._start_btn["state"]) == "normal"


def test_start_is_refused_without_hardware(root, tmp_path):
    f = make_frame(root, tmp_path, "Preview + ADRC")

    class Disconnected:
        def is_connected(self): return False
        def read_latest(self): return None

    f.moku_tab = f.display_tab = Disconnected()
    ok, why = f._preflight()
    assert not ok and "Moku" in why


def test_shutdown_stops_a_running_loop(root, tmp_path):
    f = make_frame(root, tmp_path, "Preview + ADRC")
    rig = SimRig()
    f.moku_tab = f.display_tab = rig
    f.duration_var.set(0.0)             # 0 => run until stopped
    f.start()

    t0 = time.time()
    while (f._live.get("n", 0) < 10) and time.time() - t0 < 10:
        root.update()
        time.sleep(0.01)

    f.shutdown()
    assert not f._worker.is_alive(), "shutdown left the worker running"
    assert rig.last_v == 0.0, "shutdown left voltage on the piezo"


# ─────────────────────────────────────────────────────────────────────────────
# A/B comparison
# ─────────────────────────────────────────────────────────────────────────────

def test_ab_runs_both_algorithms_and_ranks_them(root, tmp_path, monkeypatch):
    import closed_loop
    monkeypatch.setattr(closed_loop.messagebox, "askokcancel", lambda *a, **k: True)
    monkeypatch.setattr(closed_loop, "AB_SETTLE_S", 0.2)
    monkeypatch.setattr(closed_loop, "AB_SETTLE_TAUS", 0.5)

    f = make_frame(root, tmp_path, "Preview + ADRC")
    rig = SimRig()
    f.moku_tab = f.display_tab = rig
    f.duration_var.set(4.0)
    f.start_ab()

    t0 = time.time()
    while f._worker.is_alive() and time.time() - t0 < 60:
        root.update()
        time.sleep(0.01)
    f._worker.join(timeout=5)
    for _ in range(5):
        root.update()
        time.sleep(0.05)

    assert f.status_var.get().startswith("Status: A/B comparison finished"), f.status_var.get()
    # Both halves ran, and the hardware was taken and released exactly once.
    assert rig.began == 1 and rig.ended == 1
    assert rig.last_v == 0.0
    assert len(glob.glob(str(tmp_path / "*.csv"))) == 2

    table = f.metrics_var.get()
    for algo in ("Software PID", "Preview + ADRC"):
        assert algo in table, table
    assert "better by" in table, table
    assert "Preview + ADRC better by" in table, table


def test_ab_requires_a_finite_duration(root, tmp_path, monkeypatch):
    import closed_loop
    warned = []
    monkeypatch.setattr(closed_loop.messagebox, "showwarning",
                        lambda title, msg, *a, **k: warned.append(title))
    monkeypatch.setattr(closed_loop.messagebox, "askokcancel", lambda *a, **k: True)

    f = make_frame(root, tmp_path, "Preview + ADRC")
    f.moku_tab = f.display_tab = SimRig()
    f.duration_var.set(0.0)             # 0 = run until stopped
    f.start_ab()

    assert warned == ["Duration required"], warned
    assert f._worker is None or not f._worker.is_alive()


def test_ab_settle_time_scales_with_plant_tau():
    """Both halves must start from rest, and how long that takes is the plant's."""
    from closed_loop import ClosedLoopFrame, AB_SETTLE_S
    from control import PlantModel

    fast = PlantModel(tau_s=0.010)          # 5*tau = 50 ms -> floor wins
    slow = PlantModel(tau_s=2.0)            # 5*tau = 10 s  -> tau wins
    assert ClosedLoopFrame._ab_settle_seconds(fast) == pytest.approx(AB_SETTLE_S)
    assert ClosedLoopFrame._ab_settle_seconds(slow) == pytest.approx(10.0)


def test_ab_second_run_starts_from_rest(root, tmp_path, monkeypatch):
    import closed_loop
    monkeypatch.setattr(closed_loop.messagebox, "askokcancel", lambda *a, **k: True)

    f = make_frame(root, tmp_path, "Preview + ADRC")
    rig = SimRig()
    f.moku_tab = f.display_tab = rig
    f.duration_var.set(3.0)
    f.start_ab()

    t0 = time.time()
    while f._worker.is_alive() and time.time() - t0 < 90:
        root.update()
        time.sleep(0.01)
    f._worker.join(timeout=5)
    for _ in range(5):
        root.update()
        time.sleep(0.05)

    logs = sorted(glob.glob(str(tmp_path / "*.csv")))
    assert len(logs) == 2
    starts = []
    for lg in logs:
        rows = list(csv.reader(open(lg)))[2:]
        starts.append(abs(float(rows[0][2])))       # |y| at t=0
    # tau=80 ms default, so 5*tau = 400 ms -> floor of 1.5 s applies: ~19 taus,
    # far enough for the piezo to be back at rest for the second half too.
    assert max(starts) < 30.0, f"a run started away from rest: {starts}"


def test_ab_table_formats_both_rows():
    from closed_loop import ClosedLoopFrame
    runs = [
        ("Software PID",   dict(rmse=20.0, settled_rmse=20.0, max_err=40.0, iae=90.0)),
        ("Preview + ADRC", dict(rmse=5.0,  settled_rmse=4.0,  max_err=9.0,  iae=30.0)),
    ]
    table = ClosedLoopFrame._format_ab(runs)
    assert "Software PID" in table and "Preview + ADRC" in table
    assert "Preview + ADRC better by 5.0×" in table, table


# ─────────────────────────────────────────────────────────────────────────────
# Settings persistence
# ─────────────────────────────────────────────────────────────────────────────

def test_settings_round_trip_across_restart(root, tmp_path):
    f = make_frame(root, tmp_path, "Software PID")
    f.k_var.set(377.5)
    f.tau_var.set(42.0)
    f.w0_var.set(123.75)
    f.wave_var.set("Ramp")
    assert f._save_settings()

    from closed_loop import ClosedLoopFrame
    reborn = ClosedLoopFrame(ttk.Notebook(root), moku_tab=None, display_tab=None)
    assert reborn.k_var.get() == pytest.approx(377.5)
    assert reborn.tau_var.get() == pytest.approx(42.0)
    assert reborn.w0_var.get() == pytest.approx(123.75)
    assert reborn.wave_var.get() == "Ramp"
    assert reborn.algo_var.get() == "Software PID"


def test_corrupt_settings_file_does_not_block_startup(root, tmp_path, isolated_settings):
    import app_settings, closed_loop
    (isolated_settings / closed_loop.SETTINGS_FILE).write_text("{not json at all")

    f = closed_loop.ClosedLoopFrame(ttk.Notebook(root), moku_tab=None, display_tab=None)
    assert f.k_var.get() > 0            # fell back to defaults
    assert app_settings.load(closed_loop.SETTINGS_FILE) == {}


def test_a_run_persists_settings_without_pressing_save(root, tmp_path):
    import app_settings, closed_loop
    f = make_frame(root, tmp_path, "Preview + ADRC")
    f.k_var.set(311.0)
    run_loop(root, f, SimRig(), 2.0)

    assert app_settings.load(closed_loop.SETTINGS_FILE)["K_nm_per_V"] == pytest.approx(311.0)


# ─────────────────────────────────────────────────────────────────────────────
# Input validation
# ─────────────────────────────────────────────────────────────────────────────

def test_bad_field_is_named_in_the_error(root, tmp_path, monkeypatch):
    import closed_loop
    errors = []
    monkeypatch.setattr(closed_loop.messagebox, "showerror",
                        lambda title, msg, *a, **k: errors.append(msg))

    f = make_frame(root, tmp_path, "Preview + ADRC")
    f.moku_tab = f.display_tab = SimRig()

    f.tau_var.set(0.0)                  # tau must be > 0
    f.start()
    assert errors and "tau" in errors[-1], errors

    errors.clear()
    f.tau_var.set(80.0)
    f.vmax_var.set(999.0)               # beyond the hard ceiling
    f.start()
    assert errors and "v_max" in errors[-1], errors


def test_unparseable_entry_is_reported_not_crashed(root, tmp_path, monkeypatch):
    import closed_loop
    errors = []
    monkeypatch.setattr(closed_loop.messagebox, "showerror",
                        lambda title, msg, *a, **k: errors.append(msg))

    f = make_frame(root, tmp_path, "Preview + ADRC")
    f.moku_tab = f.display_tab = SimRig()
    f.k_var._root.globalsetvar(f.k_var._name, "not-a-number")

    f.start()                            # must not raise
    assert errors and "K" in errors[-1], errors


def test_negative_setpoint_is_flagged_as_unreachable(root, tmp_path, monkeypatch):
    """A unipolar piezo cannot follow a reference that dips below zero."""
    import closed_loop
    asked = []
    monkeypatch.setattr(closed_loop.messagebox, "askokcancel",
                        lambda title, msg, *a, **k: (asked.append(title), False)[1])

    f = make_frame(root, tmp_path, "Preview + ADRC")
    f.moku_tab = f.display_tab = SimRig()
    f.off_var.set(0.0)                   # sine centred on 0 => dips to -amplitude
    f.amp_var.set(300.0)
    f.start()

    assert "Setpoint goes negative" in asked, asked


# ─────────────────────────────────────────────────────────────────────────────
# Watchdog scaling
# ─────────────────────────────────────────────────────────────────────────────

def test_watchdog_has_a_floor_so_fast_loops_do_not_trip_on_jitter(root, tmp_path):
    """
    3*dt at dt=20 ms is 60 ms -- shorter than ordinary GUI scheduling jitter.
    Freshness is a property of the ~610 Hz sensor stream, not the control rate.
    """
    import closed_loop
    assert closed_loop.WATCHDOG_MIN_S >= 0.2

    f = make_frame(root, tmp_path, "Preview + ADRC")
    f.dt_var.set(5.0)                    # 5 ms => 3*dt is only 15 ms
    live = run_loop(root, f, SimRig(), 3.0)

    assert "stale" not in f.status_var.get(), f.status_var.get()
    assert live["n"] > 20


# ─────────────────────────────────────────────────────────────────────────────
# Plot
# ─────────────────────────────────────────────────────────────────────────────

def test_plot_decimates_long_traces():
    from closed_loop import LivePlot
    assert len(LivePlot._decimate(list(range(50)))) == 50
    long = LivePlot._decimate(list(range(100_000)))
    assert len(long) <= LivePlot.MAX_POINTS


def test_plot_reuses_line_artists_between_frames(root, tmp_path):
    """Clearing and replotting each frame starved the control thread."""
    f = make_frame(root, tmp_path, "Preview + ADRC")
    t = list(np.linspace(0, 1, 40))
    z = [0.0] * 40

    f.plot.update_live(t, z, z, z, z, res1=z, title="Preview + ADRC")
    first = dict(f.plot._lines)
    f.plot.update_live(t, z, z, z, z, res1=z, title="Preview + ADRC")

    assert f.plot._lines["y"] is first["y"], "line artists were recreated"
    assert "res1" in f.plot._lines, "preview run must plot the disturbance estimate"


def test_plot_drops_res1_for_pid(root, tmp_path):
    f = make_frame(root, tmp_path, "Software PID")
    t = list(np.linspace(0, 1, 20)); z = [0.0] * 20
    f.plot.update_live(t, z, z, z, z, res1=None, title="Software PID")
    assert "res1" not in f.plot._lines


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
