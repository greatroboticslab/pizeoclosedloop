"""
Headless tests for control.py — no hardware, no Tk, no MQTT.

Run:  python3 -m pytest test_control.py -v
  or: python3 test_control.py

The Preview-vs-PID acceptance criterion mirrors meter/test_simulate.m:192
(`test_runSim_Preview_exactModel_tracksBetterThanPID`): with an exact model and
no noise, the preview feedforward must track substantially better than PID.
"""
from __future__ import annotations

import math
import os
import re

import numpy as np
import pytest

from control import (
    FRINGE_NM,
    MeasurementGuard,
    PlantModel,
    PreviewADRC,
    SimulatedPlant,
    SoftwarePID,
    TrajectoryGenerator,
    contribution_shares,
    fit_fopdt_step,
    imc_tune,
    lift_reference,
    plant_fit_usable,
    simulate,
)

DT = 0.01
DURATION = 6.0

# The piezo is unipolar: v is clamped to [0, v_max], so y can only be >= 0. A
# reference must therefore sit on a positive offset to be reachable at all --
# a sine centred on 0 nm would demand negative displacement for half of every
# cycle and no controller could track it.
REF_OFFSET_NM = 800.0
REF_AMP_NM = 400.0


def tracking_traj(freq_hz: float = 0.5) -> TrajectoryGenerator:
    return TrajectoryGenerator("Sine", amplitude_nm=REF_AMP_NM,
                               frequency_hz=freq_hz, offset_nm=REF_OFFSET_NM)


def settled_rmse(out, frac: float = 0.5) -> float:
    """
    RMSE over the last `1-frac` of the run.

    The piezo starts at 0 nm while the reference starts at REF_OFFSET_NM, so
    every run opens with an unavoidable approach transient governed by the
    plant's own time constant. Steady-state tracking is what distinguishes the
    control laws, so the transient is excluded -- the same split meter uses
    when it reports convergence and tracking as separate protocols.
    """
    e = out["e"]
    return float(np.sqrt(np.mean(e[int(len(e) * frac):] ** 2)))


def make_model(**kw) -> PlantModel:
    base = dict(K_nm_per_V=410.0, tau_s=0.08, theta_plant_s=0.0,
                theta_sensor_s=0.0, v_max=5.0, v_dead=0.0)
    base.update(kw)
    return PlantModel(**base)


def tuned_pair(model: PlantModel, dt: float = DT):
    g = imc_tune(model, dt)
    pid = SoftwarePID(model, kp=g["kp"], ki=g["ki"], kd=g["kd"], d_filter_n=g["d_filter_n"])
    adrc = PreviewADRC(model, w0=g["w0"], dt=dt)
    return pid, adrc


# ─────────────────────────────────────────────────────────────────────────────
# PlantModel
# ─────────────────────────────────────────────────────────────────────────────

def test_plant_model_rejects_nonsense():
    for bad in (dict(K_nm_per_V=0.0), dict(tau_s=0.0), dict(v_max=-1.0)):
        with pytest.raises(ValueError):
            make_model(**bad)


def test_discrete_matches_zoh_solution():
    m = make_model(tau_s=0.08, theta_plant_s=0.02)
    a, b, d = m.discrete(0.01)
    assert a == pytest.approx(math.exp(-0.01 / 0.08))
    assert b == pytest.approx(410.0 * (1 - a))
    assert d == 2                       # round(0.02/0.01)


def test_stroke_accounts_for_dead_band():
    assert make_model(v_dead=1.0).stroke_nm == pytest.approx(410.0 * 4.0)


# ─────────────────────────────────────────────────────────────────────────────
# TrajectoryGenerator
# ─────────────────────────────────────────────────────────────────────────────

def test_sine_is_closed_form_and_periodic():
    tr = TrajectoryGenerator("Sine", amplitude_nm=100.0, frequency_hz=2.0)
    assert tr.r(0.0) == pytest.approx(0.0, abs=1e-9)
    assert tr.r(0.125) == pytest.approx(100.0)       # quarter period
    assert tr.r(3.7) == pytest.approx(tr.r(3.7 + 0.5))

def test_arbitrary_future_time_is_evaluable():
    """The whole point of a closed form: no lookahead buffer, no horizon limit."""
    tr = TrajectoryGenerator("Sine", amplitude_nm=100.0, frequency_hz=1.0)
    assert tr.r(1e6) == pytest.approx(tr.r(0.0), abs=1e-6)


def test_square_respects_duty():
    tr = TrajectoryGenerator("Square", amplitude_nm=50.0, frequency_hz=1.0, duty_pct=25.0)
    assert tr.r(0.1) == pytest.approx(50.0)
    assert tr.r(0.5) == pytest.approx(-50.0)


def test_ramp_is_continuous_triangle():
    tr = TrajectoryGenerator("Ramp", amplitude_nm=100.0, frequency_hz=1.0, symmetry_pct=50.0)
    assert tr.r(0.0) == pytest.approx(-100.0)
    assert tr.r(0.5) == pytest.approx(100.0)
    assert tr.r(0.25) == pytest.approx(0.0)


def test_pulse_rests_at_offset():
    tr = TrajectoryGenerator("Pulse", amplitude_nm=80.0, frequency_hz=1.0,
                             offset_nm=10.0, duty_pct=20.0)
    assert tr.r(0.1) == pytest.approx(90.0)
    assert tr.r(0.5) == pytest.approx(10.0)


def test_zero_frequency_holds_static():
    tr = TrajectoryGenerator("Sine", amplitude_nm=100.0, frequency_hz=0.0, offset_nm=42.0)
    assert tr.r(0.0) == pytest.approx(42.0)
    assert tr.r(99.0) == pytest.approx(42.0)


def test_bandwidth_warning_fires_above_plant_bandwidth():
    m = make_model(tau_s=0.08)          # ~2 Hz
    assert TrajectoryGenerator("Sine", frequency_hz=50.0).bandwidth_warning(m)
    assert not TrajectoryGenerator("Sine", frequency_hz=0.1).bandwidth_warning(m)


def test_rejects_unknown_waveform():
    with pytest.raises(ValueError):
        TrajectoryGenerator("Sawtooth")


# ─────────────────────────────────────────────────────────────────────────────
# Controller invariants (meter/test_simulate.m:171,181)
# ─────────────────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("which", ["pid", "adrc"])
def test_output_stays_in_voltage_bounds_and_finite(which):
    model = make_model()
    pid, adrc = tuned_pair(model)
    ctrl = pid if which == "pid" else adrc
    traj = tracking_traj(freq_hz=1.0)

    out = simulate(ctrl, SimulatedPlant(model), traj, DT, DURATION)

    assert np.all(np.isfinite(out["v"])), "voltage went NaN/inf"
    assert np.all(np.isfinite(out["y"])), "displacement went NaN/inf"
    assert out["v"].min() >= -1e-12
    assert out["v"].max() <= model.v_max + 1e-12


@pytest.mark.parametrize("which", ["pid", "adrc"])
def test_saturating_setpoint_does_not_destabilise(which):
    """Command far beyond the reachable stroke; output must clamp, not diverge."""
    model = make_model()
    pid, adrc = tuned_pair(model)
    ctrl = pid if which == "pid" else adrc
    traj = TrajectoryGenerator("DC", amplitude_nm=10 * model.stroke_nm, frequency_hz=0.0)

    out = simulate(ctrl, SimulatedPlant(model), traj, DT, DURATION)

    assert np.all(np.isfinite(out["v"]))
    assert out["v"].max() <= model.v_max + 1e-12
    assert out["y"].max() <= model.stroke_nm * 1.05


# ─────────────────────────────────────────────────────────────────────────────
# The headline claim (meter/test_simulate.m:192)
# ─────────────────────────────────────────────────────────────────────────────

def test_preview_adrc_beats_pid_on_tracking_with_exact_model():
    model = make_model()
    pid, adrc = tuned_pair(model)
    traj = tracking_traj()

    pid_out = simulate(pid, SimulatedPlant(model), traj, DT, 8.0)
    adrc_out = simulate(adrc, SimulatedPlant(model), traj, DT, 8.0)

    assert adrc_out["iae"] < pid_out["iae"], (
        f"preview IAE {adrc_out['iae']:.1f} not better than PID {pid_out['iae']:.1f}"
    )
    assert settled_rmse(adrc_out) < 0.25 * settled_rmse(pid_out), (
        f"preview settled RMSE {settled_rmse(adrc_out):.2f} nm "
        f"vs PID {settled_rmse(pid_out):.2f} nm"
    )


def test_preview_adrc_rejects_hysteresis_disturbance():
    """The residual observer's whole purpose: cancel an additive disturbance."""
    model = make_model()
    pid, adrc = tuned_pair(model)
    traj = tracking_traj()

    adrc_out = simulate(adrc, SimulatedPlant(model, bw_enable=True, bw_D=100.0), traj, DT, 12.0)
    pid_out = simulate(pid, SimulatedPlant(model, bw_enable=True, bw_D=100.0), traj, DT, 12.0)

    # The observer estimates the additive disturbance and cancels it with
    # res1/K volts. The PI integrator cannot: it chases a residual that barely
    # responds to its own correction (meter/docs_why_preview_pid_diverges.md).
    assert settled_rmse(adrc_out) < 40.0, f"{settled_rmse(adrc_out):.1f} nm"
    assert settled_rmse(adrc_out) < 0.3 * settled_rmse(pid_out)


def test_whole_fringe_reading_cannot_pin_the_trim_at_the_rail():
    """A count stuck on one fringe must not wind the disturbance up to v_max.

    The 2026-10-27 run sat at 5 V across the crest because the observer treated
    the unseen part of a fringe as a real disturbance of several hundred nm.
    """
    k = FRINGE_NM
    model = PlantModel(K_nm_per_V=k, tau_s=0.00234, theta_plant_s=0.0, v_max=5.0)
    adrc = PreviewADRC(model, w0=6.6, dt=0.02)
    y = 3.0 * FRINGE_NM  # one fringe under the setpoint: quantization, not a miss
    r = 1000.0
    v = 0.0
    for _ in range(200):
        v = adrc.update(r, r, y, 0.02)
    assert abs(adrc.command_terms()["dist"]) <= FRINGE_NM / k + 1e-6
    assert v < model.v_max - 0.2

    # A reading that is not on the fringe grid is a real position, and a
    # multi-fringe disturbance must still be corrected.
    adrc.reset()
    y_fine = 2.5 * FRINGE_NM
    for _ in range(400):
        adrc.update(0.0, 0.0, y_fine, 0.02)
    assert abs(adrc.command_terms()["dist"]) > 1.5 * FRINGE_NM / k


def test_preview_adrc_converges_on_a_step():
    model = make_model()
    _, adrc = tuned_pair(model)
    traj = TrajectoryGenerator("DC", amplitude_nm=500.0, frequency_hz=0.0)

    out = simulate(adrc, SimulatedPlant(model), traj, DT, DURATION)

    assert abs(out["e"][-1]) < 5.0, f"final error {out['e'][-1]:.2f} nm"


def test_both_controllers_track_a_static_setpoint():
    model = make_model()
    pid, adrc = tuned_pair(model)
    traj = TrajectoryGenerator("DC", amplitude_nm=500.0, frequency_hz=0.0)

    for ctrl in (pid, adrc):
        out = simulate(ctrl, SimulatedPlant(model), traj, DT, 15.0)
        assert abs(out["e"][-1]) < 20.0, f"{ctrl.name} settled at {out['e'][-1]:.2f} nm"


# ─────────────────────────────────────────────────────────────────────────────
# Preview structure — the three things that must not be "simplified"
# ─────────────────────────────────────────────────────────────────────────────

def test_observer_input_matrix_is_single_column():
    """No b0*u term: nothing but the exogenous disturbance drives this channel."""
    adrc = PreviewADRC(make_model(), w0=100.0, dt=DT)
    assert adrc.Ad_res.shape == (2, 2)
    assert adrc.Bd_res.shape == (2,)


def test_feedforward_alone_inverts_the_plant_exactly():
    """With w0 tiny the observer barely acts, so uFF must carry the tracking."""
    model = make_model()
    adrc = PreviewADRC(model, w0=1e-6, dt=DT)

    out = simulate(adrc, SimulatedPlant(model), tracking_traj(), DT, 8.0)

    # Exact model + exact inversion => the only error is the opening transient.
    assert settled_rmse(out) < 1e-6, f"settled RMSE {settled_rmse(out):.3g} nm"


def test_preview_horizon_grows_with_plant_dead_time():
    dt = 0.01
    assert PreviewADRC(make_model(theta_plant_s=0.0), 100.0, dt).d == 0
    assert PreviewADRC(make_model(theta_plant_s=0.03), 100.0, dt).d == 3

    a = PreviewADRC(make_model(theta_plant_s=0.03), 100.0, dt)
    t_d, t_d1 = a.preview_times(1.0)
    assert t_d == pytest.approx(1.03)
    assert t_d1 == pytest.approx(1.04)


def test_preview_survives_plant_dead_time():
    model = make_model(theta_plant_s=0.03)
    pid, adrc = tuned_pair(model)
    traj = tracking_traj()

    adrc_out = simulate(adrc, SimulatedPlant(model), traj, DT, 8.0)
    pid_out = simulate(pid, SimulatedPlant(model), traj, DT, 8.0)

    assert np.all(np.isfinite(adrc_out["v"]))
    # The feedforward reads the setpoint d+1 ticks ahead, so dead time costs it
    # much less than it costs the purely reactive PID.
    assert settled_rmse(adrc_out) < settled_rmse(pid_out)


def test_sensor_delay_buffer_length_tracks_theta_sensor():
    assert PreviewADRC(make_model(theta_sensor_s=0.0), 100.0, 0.01)._sensor_delay_steps == 0
    assert PreviewADRC(make_model(theta_sensor_s=0.05), 100.0, 0.01)._sensor_delay_steps == 5


def test_reset_clears_all_controller_state():
    model = make_model()
    pid, adrc = tuned_pair(model)
    traj = TrajectoryGenerator("Sine", amplitude_nm=500.0, frequency_hz=1.0)

    for ctrl in (pid, adrc):
        first = simulate(ctrl, SimulatedPlant(model), traj, DT, 2.0)
        second = simulate(ctrl, SimulatedPlant(model), traj, DT, 2.0)
        assert np.allclose(first["v"], second["v"]), f"{ctrl.name} kept stale state"


# ─────────────────────────────────────────────────────────────────────────────
# PID specifics
# ─────────────────────────────────────────────────────────────────────────────

def test_integral_anti_windup_clamps():
    model = make_model()
    pid = SoftwarePID(model, kp=0.0, ki=1.0, kd=0.0)
    for _ in range(500):
        pid.update(setpoint_nm=1e6, y_meas_nm=0.0, dt=DT)
    assert pid.diagnostics()["integral"] <= model.v_max / 1.0 + 1e-9


def test_feedforward_holds_a_fast_plant_tighter_than_the_integrator():
    """
    tau of a few ms is over before the next 50 ms tick. Inverting the gain
    should track a sine far more tightly than waiting for the IMC integrator.
    """
    model = PlantModel(K_nm_per_V=FRINGE_NM, tau_s=0.0024, theta_plant_s=0.0, v_max=5.0)
    dt = 0.05
    g = imc_tune(model, dt)
    traj = TrajectoryGenerator("Sine", amplitude_nm=600.0, frequency_hz=0.2, offset_nm=900.0)
    reactive = SoftwarePID(model, kp=g["kp"], ki=g["ki"], kd=g["kd"], d_filter_n=g["d_filter_n"])
    ff = SoftwarePID(
        model, kp=g["kp"], ki=g["ki"], kd=g["kd"], d_filter_n=g["d_filter_n"],
        feedforward=True, deadband_nm=0.5 * FRINGE_NM,
    )
    plant = SimulatedPlant(model)
    reactive_out = simulate(reactive, plant, traj, dt, 8.0)
    ff_out = simulate(ff, SimulatedPlant(model), traj, dt, 8.0)
    assert settled_rmse(ff_out) < 30.0, settled_rmse(ff_out)
    assert settled_rmse(ff_out) < 0.5 * settled_rmse(reactive_out)


def test_half_fringe_deadband_does_not_integrate():
    """An error inside one interferometer bin is not a reason to change voltage."""
    model = PlantModel(K_nm_per_V=FRINGE_NM, tau_s=0.0024, theta_plant_s=0.0, v_max=5.0)
    pid = SoftwarePID(
        model, kp=0.001, ki=0.05, kd=0.0,
        feedforward=True, deadband_nm=0.5 * FRINGE_NM,
    )
    # First sample sits on the setpoint, so the trim has nothing to do.
    pid.update(1000.0, 1000.0, 0.05, setpoint_next_nm=1000.0)
    assert pid.diagnostics()["integral"] == 0.0
    # 100 nm is inside half a fringe. The voltage may change with the learned
    # offset; the integrator must not.
    pid.update(1000.0, 900.0, 0.05, setpoint_next_nm=1000.0)
    assert pid.diagnostics()["integral"] == 0.0


def test_feedforward_learns_the_return_offset_at_zero_volts():
    """After a move, 0 V no longer reads 0 nm. The trim has to learn that."""
    model = PlantModel(K_nm_per_V=FRINGE_NM, tau_s=0.0024, theta_plant_s=0.0, v_max=5.0)
    pid = SoftwarePID(
        model, kp=0.0, ki=0.0, kd=0.0,
        feedforward=True, deadband_nm=0.5 * FRINGE_NM,
    )
    for _ in range(30):
        pid.update(0.0, 300.0, 0.05, setpoint_next_nm=0.0)
    assert pid.diagnostics()["bias"] > 200.0
    assert pid.last_v == 0.0


def test_feedforward_does_not_wind_up_at_the_voltage_rail():
    model = PlantModel(K_nm_per_V=FRINGE_NM, tau_s=0.0024, theta_plant_s=0.0, v_max=5.0)
    pid = SoftwarePID(
        model, kp=0.001, ki=0.05, kd=0.0,
        feedforward=True, deadband_nm=0.5 * FRINGE_NM,
    )
    for _ in range(50):
        # Setpoint far past the stroke, measurement stuck at full scale.
        pid.update(4000.0, 5.0 * FRINGE_NM, 0.05, setpoint_next_nm=4000.0)
    assert pid.last_v == pytest.approx(5.0)
    assert pid.diagnostics()["integral"] == pytest.approx(0.0)


def test_derivative_acts_on_measurement_not_error():
    """A setpoint jump must not produce a derivative kick."""
    model = make_model()
    pid = SoftwarePID(model, kp=0.0, ki=0.0, kd=1.0, d_filter_n=20.0)
    pid.update(0.0, 0.0, DT)
    pid.update(0.0, 0.0, DT)
    assert pid.diagnostics()["d_filt"] == pytest.approx(0.0)

    pid.update(1000.0, 0.0, DT)         # setpoint jumps, measurement does not
    assert pid.diagnostics()["d_filt"] == pytest.approx(0.0)


def test_contribution_shares_keep_opposing_signs():
    shares = contribution_shares({"ff": 4.0, "p": -1.0, "i": 0.0})
    assert shares["ff"] == pytest.approx(80.0)
    assert shares["p"] == pytest.approx(-20.0)
    assert shares["i"] == pytest.approx(0.0)
    assert sum(abs(v) for v in shares.values()) == pytest.approx(100.0)


def test_contribution_shares_of_a_zero_command_are_zero():
    assert contribution_shares({"p": 0.0, "i": 0.0, "d": 0.0}) == {
        "p": 0.0, "i": 0.0, "d": 0.0,
    }


def _clipped(raw, model):
    return min(max(raw, 0.0), model.v_max)


def test_pid_terms_reconstruct_the_command():
    model = make_model()
    pid = SoftwarePID(model, kp=0.002, ki=0.01, kd=0.0)
    v = pid.update(500.0, 100.0, DT)
    terms = pid.command_terms()
    assert set(terms) == {"p", "i", "d"}
    assert _clipped(sum(terms.values()), model) == pytest.approx(v)
    assert sum(abs(x) for x in contribution_shares(terms).values()) == pytest.approx(100.0)


def test_feedforward_terms_include_the_inverted_setpoint():
    model = PlantModel(K_nm_per_V=200.0, tau_s=0.001, theta_plant_s=0.0, v_max=5.0)
    pid = SoftwarePID(model, kp=0.0, ki=0.0, kd=0.0, feedforward=True)
    v = pid.update(0.0, 0.0, 0.05, setpoint_next_nm=400.0)
    terms = pid.command_terms()
    assert set(terms) == {"ff", "p", "i", "d"}
    assert terms["ff"] == pytest.approx(2.0)
    assert _clipped(sum(terms.values()), model) == pytest.approx(v)
    assert contribution_shares(terms)["ff"] == pytest.approx(100.0)


def test_preview_terms_split_feedforward_and_disturbance():
    model = make_model()
    adrc = PreviewADRC(model, w0=40.0, dt=DT)
    v = adrc.update(800.0, 900.0, 0.0, DT)
    terms = adrc.command_terms()
    assert set(terms) == {"ff", "dist"}
    assert _clipped(sum(terms.values()), model) == pytest.approx(v)
    assert sum(abs(x) for x in contribution_shares(terms).values()) == pytest.approx(100.0)


# ─────────────────────────────────────────────────────────────────────────────
# Tuning and identification
# ─────────────────────────────────────────────────────────────────────────────

def test_imc_tune_matches_meter_formulas():
    model = make_model(tau_s=0.08, theta_plant_s=0.01, theta_sensor_s=0.0)
    dt, fs = 0.05, 1000.0
    g = imc_tune(model, dt, fs_sample_hz=fs)

    # tunedPID.m
    theta_eff = 0.01 + 0.0 + dt / 2 + 0.5 / fs
    denom = 0.08 + theta_eff / 2
    assert g["kp"] == pytest.approx(denom / (410.0 * (2 * theta_eff + theta_eff / 2)))
    assert g["ki"] == pytest.approx(g["kp"] / denom)

    # tunedADRC.m — theta_plant absent, the feedforward predicts it away
    theta_eff_adrc = 0.0 + dt / 2 + 0.5 / fs
    assert g["w0"] == pytest.approx(5.0 / max(0.08 + theta_eff_adrc, 1e-9))


def test_imc_gains_are_usable():
    g = imc_tune(make_model(), DT)
    assert all(g[k] > 0 for k in ("kp", "ki", "w0"))
    assert g["d_filter_n"] >= 1


def test_fit_fopdt_recovers_known_parameters():
    K, tau, theta, dv = 410.0, 0.08, 0.02, 1.0
    t = np.linspace(0, 1.0, 400)
    y = np.where(t > theta, K * dv * (1 - np.exp(-(t - theta) / tau)), 0.0)

    K_hat, tau_hat, theta_hat = fit_fopdt_step(t, y, dv)

    assert K_hat == pytest.approx(K, rel=0.02)
    assert tau_hat == pytest.approx(tau, rel=0.05)
    assert theta_hat == pytest.approx(theta, abs=0.01)


def test_fit_fopdt_survives_noise():
    K, tau, theta, dv = 410.0, 0.08, 0.01, 1.0
    t = np.linspace(0, 1.0, 400)
    clean = np.where(t > theta, K * dv * (1 - np.exp(-(t - theta) / tau)), 0.0)
    y = clean + np.random.default_rng(0).normal(0, 5.0, t.shape)

    K_hat, tau_hat, _ = fit_fopdt_step(t, y, dv)

    assert K_hat == pytest.approx(K, rel=0.05)
    assert tau_hat == pytest.approx(tau, rel=0.20)


def test_fit_fopdt_rejects_degenerate_input():
    t = np.linspace(0, 1, 100)
    with pytest.raises(ValueError):
        fit_fopdt_step(t, t, delta_v=0.0)
    with pytest.raises(ValueError):
        fit_fopdt_step(t[:3], t[:3], delta_v=1.0)


# ─────────────────────────────────────────────────────────────────────────────
# display.py MQTT payload parsing (the prerequisite fix)
# ─────────────────────────────────────────────────────────────────────────────

def test_dn_payload_regex_matches_what_vb_publishes():
    """uMD_GUI publishes D, an optional phase word, and N (MainForm.vb)."""
    src = open("display.py", encoding="utf-8").read()
    pattern = re.search(r'_DN_RE = re\.compile\(r"([^"]+)"\)', src).group(1)
    dn_re = re.compile(pattern)

    m = dn_re.match("D:12345667 N:8901234")
    assert m is not None
    assert int(m.group(1)) == 12345667
    assert m.group(2) is None
    assert int(m.group(3)) == 8901234

    m = dn_re.match("D:100 P:128 N:5")
    assert int(m.group(1)) == 100
    assert int(m.group(2)) == 128
    assert int(m.group(3)) == 5
    assert m.group(4) is None

    m = dn_re.match("D:100 P:128 N:5 Y:31450.2500")
    assert float(m.group(4)) == pytest.approx(31450.25)

    assert dn_re.match("D:-42 P:-10 N:7") is not None
    assert dn_re.match("  D:1 N:2  ") is not None
    assert dn_re.match("1,2,3") is None
    assert dn_re.match("123.4") is None


def test_phase_word_splits_a_fringe_into_256_steps():
    from process_raw import phase_fraction, raw_to_nm

    assert phase_fraction(0) == 0.0
    assert phase_fraction(128) == pytest.approx(0.5)
    assert phase_fraction(0x7E00) is None          # invalid interpolation word
    # Unsigned 16-bit form of a negative word: both beams valid.
    assert phase_fraction(0xFF80) == pytest.approx(-128 / 256)
    whole = raw_to_nm(10)
    half = raw_to_nm(10, phase=phase_fraction(128))
    assert whole - half == pytest.approx(632.991372 / 4.0)


# ─────────────────────────────────────────────────────────────────────────────
# app_settings — the JSON store behind "settings survive a restart"
# ─────────────────────────────────────────────────────────────────────────────

def test_settings_round_trip(tmp_path, monkeypatch):
    import app_settings
    monkeypatch.setattr(app_settings, "settings_dir", lambda: tmp_path)

    payload = {"K_nm_per_V": 410.0, "waveform": "Sine", "log_enabled": True}
    assert app_settings.save("t.json", payload)
    assert app_settings.load("t.json") == payload


def test_settings_missing_file_is_empty(tmp_path, monkeypatch):
    import app_settings
    monkeypatch.setattr(app_settings, "settings_dir", lambda: tmp_path)
    assert app_settings.load("nope.json") == {}


@pytest.mark.parametrize("content", ["{not json", "[1,2,3]", '"a string"', ""])
def test_settings_corrupt_file_is_empty_not_fatal(tmp_path, monkeypatch, content):
    """A bad settings file must never stop the app from starting."""
    import app_settings
    monkeypatch.setattr(app_settings, "settings_dir", lambda: tmp_path)
    (tmp_path / "bad.json").write_text(content)
    assert app_settings.load("bad.json") == {}


def test_settings_creates_directory(tmp_path, monkeypatch):
    import app_settings
    nested = tmp_path / "a" / "b"
    monkeypatch.setattr(app_settings, "settings_dir", lambda: nested)
    assert app_settings.save("t.json", {"x": 1})
    assert (nested / "t.json").exists()


def test_settings_save_failure_is_reported_not_raised(tmp_path, monkeypatch):
    import app_settings
    monkeypatch.setattr(app_settings, "settings_dir", lambda: tmp_path)
    # Not JSON-serialisable -- save must return False rather than propagate.
    assert app_settings.save("t.json", {"bad": object()}) is False


def test_settings_write_is_atomic(tmp_path, monkeypatch):
    """An interrupted write must not truncate the previous good file."""
    import app_settings
    monkeypatch.setattr(app_settings, "settings_dir", lambda: tmp_path)
    app_settings.save("t.json", {"keep": 1})

    real_replace = os.replace
    monkeypatch.setattr(os, "replace",
                        lambda *a: (_ for _ in ()).throw(OSError("boom")))
    assert app_settings.save("t.json", {"keep": 2}) is False
    monkeypatch.setattr(os, "replace", real_replace)

    assert app_settings.load("t.json") == {"keep": 1}


def test_plant_fit_rejects_an_unsettled_step():
    """tau of several seconds is a fringe-count drift, not a piezo."""
    assert plant_fit_usable(0.08, 0.01)
    assert not plant_fit_usable(2.86, 0.416)
    assert not plant_fit_usable(0.08, 0.5)


def test_command_stays_on_the_setpoint_voltage_when_the_count_runs_off():
    """Past the crest, follow the setpoint voltage. Do not park at 0 V."""
    from control import command_on_wave, setpoint_span_nm

    lo, hi = setpoint_span_nm(700.0, 300.0, "Sine", 0.3)
    assert lo == pytest.approx(400.0)
    assert hi == pytest.approx(1000.0)
    # On the top fringe, 5 V is past the crest (about 3.2 V at 316 nm/V).
    assert command_on_wave(5.0, 949.0, hi, v_setpoint=2.2, v_crest=3.2) == pytest.approx(3.2)
    assert command_on_wave(2.0, 600.0, hi, v_setpoint=2.2, v_crest=3.2) == pytest.approx(2.0)
    # The 11:50 log forced 0 V here and left it there. Stay on the setpoint.
    assert command_on_wave(0.0, 1582.0, hi, v_setpoint=1.45, v_crest=3.2) == pytest.approx(1.45)
    # Still a fringe below the live setpoint: the crest cap is what left the
    # 12:06 run on 316 nm. Let the trim use the full voltage range.
    assert command_on_wave(
        5.0, 316.5, hi, v_setpoint=3.1, v_crest=3.2, v_max=5.0, setpoint_nm=986.0,
    ) == pytest.approx(5.0)
    assert command_on_wave(
        5.0, 949.5, hi, v_setpoint=3.1, v_crest=3.2, v_max=5.0, setpoint_nm=986.0,
    ) == pytest.approx(3.2)


def test_fringe_step_follows_the_setpoint_and_ignores_a_bounce():
    """The trace steps toward the setpoint's fringe, and a one-sample bounce does not."""
    from control import choose_fringe_step

    low, mid, high = 1.0 * FRINGE_NM, 2.0 * FRINGE_NM, 3.0 * FRINGE_NM
    # Setpoint near 1000 nm snaps to the 3rd fringe. Climbing is immediate.
    step, away = choose_fringe_step(low, mid, 986.0, 0)
    assert step == pytest.approx(mid)
    assert away == 0
    step, away = choose_fringe_step(mid, high, 986.0, 0)
    assert step == pytest.approx(high)
    # A single drop back toward a lower fringe does not move the trace.
    step, away = choose_fringe_step(high, mid, 986.0, 0)
    assert step == pytest.approx(high)
    assert away == 1
    # Setpoint near the trough snaps the other way as soon as the count arrives.
    step, away = choose_fringe_step(high, low, 420.0, 0)
    assert step == pytest.approx(low)


def test_multi_fringe_overshoot_can_cut_the_command():
    """A count far above the setpoint must be able to pull a saturated command off the rail."""
    model = PlantModel(K_nm_per_V=10.914, tau_s=0.75, theta_plant_s=0.02, v_max=5.0)
    adrc = PreviewADRC(model, w0=6.6, dt=0.02)
    v = 5.0
    for _ in range(150):
        v = adrc.update(700.0, 720.0, 8.0 * FRINGE_NM, 0.02)
    assert v < 0.5


def test_whole_fringe_nm_snaps_a_phase_fraction_back_to_the_count():
    from control import whole_fringe_nm

    assert whole_fringe_nm(2.0 * FRINGE_NM) == pytest.approx(2.0 * FRINGE_NM)
    assert whole_fringe_nm(2.0 * FRINGE_NM - 40.0) == pytest.approx(2.0 * FRINGE_NM)
    assert whole_fringe_nm(2.2 * FRINGE_NM) == pytest.approx(2.0 * FRINGE_NM)


def test_guard_accepts_a_step_the_voltage_can_produce():
    """5 V on a 1-fringe-per-volt plant moves five fringes, and that is real."""
    model = PlantModel(K_nm_per_V=FRINGE_NM, tau_s=0.0024, theta_plant_s=0.0, v_max=5.0)
    guard = MeasurementGuard(model)
    guard.update(0.0, 0.05)
    guard.note_voltage(5.0)
    y, ok = guard.update(5.0 * FRINGE_NM, 0.05)
    assert ok
    assert y == pytest.approx(5.0 * FRINGE_NM)


def test_guard_keeps_a_few_fringes_of_hysteresis():
    """The 13:14 log sits up to about 3 fringes off the prediction while tracking."""
    model = PlantModel(K_nm_per_V=FRINGE_NM, tau_s=0.0024, theta_plant_s=0.0, v_max=5.0)
    guard = MeasurementGuard(model)
    guard.update(0.0, 0.05)
    guard.note_voltage(0.0)
    y, ok = guard.update(3.0 * FRINGE_NM, 0.05)
    assert ok
    assert y == pytest.approx(3.0 * FRINGE_NM)


def test_guard_drops_a_fringe_runaway_and_takes_the_count_back():
    """
    Fifty fringes at 0 V is the t≈10 s event. The controller must see the
    predicted position, and a count that walks home must be trusted again.
    """
    model = PlantModel(K_nm_per_V=FRINGE_NM, tau_s=0.0024, theta_plant_s=0.0, v_max=5.0)
    guard = MeasurementGuard(model)
    guard.update(0.0, 0.05)
    guard.note_voltage(0.0)

    y, ok = guard.update(54.0 * FRINGE_NM, 0.05)
    assert not ok
    assert y == pytest.approx(0.0, abs=1.0)
    assert guard.rejected == 1

    # Still lost: keep substituting the prediction, and follow a new voltage.
    guard.note_voltage(2.0)
    y, ok = guard.update(54.0 * FRINGE_NM, 0.05)
    assert not ok
    assert y == pytest.approx(2.0 * FRINGE_NM, abs=1.0)

    # Count returns to where that voltage actually puts the piezo.
    y, ok = guard.update(2.0 * FRINGE_NM, 0.05)
    assert ok
    assert y == pytest.approx(2.0 * FRINGE_NM, abs=1.0)


def test_lift_reference_raises_a_wave_sitting_on_the_rest_position():
    """A setpoint at or below the 0 V reading cannot be reached: voltage clips to 0."""
    # Trough is 695 nm, rest is 705 nm, and the wave needs 25 nm of clearance.
    assert lift_reference(700.0, 5.0, "Sine", 5.0, y_rest_nm=705.0) == pytest.approx(35.0)
    # Already clear of the floor: 700 - 200 = 500, rest is 0.
    assert lift_reference(700.0, 200.0, "Sine", 0.2, y_rest_nm=0.0) == 0.0


def test_relative_sine_commands_voltage_above_a_nonzero_rest_reading():
    """
    Absolute nm used to cancel the feedforward: the model starts at 0 nm, the
    observer subtracts the whole interferometer reading, and the command stays
    at 0 V whenever the setpoint sits on that reading.
    """
    model = make_model()
    dt = 0.05
    y_rest = 705.0
    shift = lift_reference(700.0, 200.0, "Sine", 0.2, y_rest)
    traj = TrajectoryGenerator(
        "Sine", amplitude_nm=200.0, frequency_hz=0.2, offset_nm=700.0 + shift,
    )
    ctrl = PreviewADRC(model, w0=47.25, dt=dt)
    plant = SimulatedPlant(model)
    plant.reset(dt)
    y = 0.0
    volts = []
    for i in range(60):
        t = i * dt
        t_d, t_d1 = ctrl.preview_times(t)
        v = ctrl.update(traj.r(t_d) - y_rest, traj.r(t_d1) - y_rest, y, dt)
        _, y = plant.step(v, dt)
        volts.append(v)

    assert max(volts) > 0.2, max(volts)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
