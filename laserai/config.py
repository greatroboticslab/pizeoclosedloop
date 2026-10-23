"""Portable rig profiles and machine-local preferences, with fail-closed validation."""

from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path
import tempfile


class ConfigurationError(ValueError):
    pass


def settings_dir() -> Path:
    return Path.home() / ".laserai"


def default_profile() -> dict:
    return {
        "schema_version": 1,
        "simulation": False,
        "device": {"address": None, "hardware": "mokugo", "slots": 3},
        "inputs": {
            "standard": None, "receiver": None, "carrier_hz": None,
            "level_vpp": None, "impedance": "1MOhm", "coupling": "AC",
            "range": "10Vpp", "pll_bandwidth": "1kHz",
            "phase_scale_v_per_cycle": None, "phase_period_counts": None,
            "signal_threshold_counts": None, "signal_timeout_s": None,
        },
        "calibration": {"nm_per_cycle": None, "direction": None},
        "drive": {
            "piezo_model": None, "capacitance_f": None, "rated_voltage_v": None,
            "minimum_v": None, "maximum_v": None, "safe_v": None,
            "slew_v_per_s": None, "max_abs_error_nm": None,
            "max_current_a": None, "counts_per_volt": 6550.4,
        },
        "model": {"gain_nm_per_v": None, "tau_s": None, "plant_delay_s": None,
                  "sensor_delay_s": None},
        "controller": {"rate_hz": 1000, "kp": None, "ki": None, "kd": None,
                       "d_filter_n": None, "observer_w0": None},
        "trajectory": {"waveform": "sine", "frequency_hz": 20.0,
                       "amplitude_nm": 500.0, "offset_nm": None,
                       "phase_deg": 0.0, "duty_pct": 50.0, "symmetry_pct": 50.0,
                       "pulse_width_s": 0.01, "edge_time_s": 0.0, "noise_seed": 0},
        "disturbance": {"enabled": False, "waveform": "step", "amplitude_v": 0.0,
                        "frequency_hz": 1.0, "start_s": 0.0, "duration_s": 1.0,
                        "phase_deg": 0.0, "seed": 0},
        "simulation_options": {"measurement_noise_nm": 0.0, "noise_seed": 0,
                               "hysteresis_enabled": False, "hysteresis_a": 1.0,
                               "hysteresis_beta": 0.5, "hysteresis_gamma": 0.5,
                               "hysteresis_nm_per_v": 0.0, "dead_band_v": 0.0},
        "experiment": {"duration_s": 10.0, "settle_s": 2.0, "repeats": 3,
                       "target_rmse_nm": 50.0},
        "validation": {key: False for key in (
            "signal_conditioning", "phase_scale", "phase_wrap", "latency",
            "loss_of_lock", "polarity", "direct_drive", "independent_phase_channels",
        )},
        "fpga": {"bitstream_path": None, "manifest_path": None},
    }


def simulation_profile() -> dict:
    p = default_profile()
    p["simulation"] = True
    p["device"]["address"] = "simulation"
    p["inputs"].update(standard="SIMULATED", receiver="SIMULATED", carrier_hz=1e6,
                       level_vpp=1.0, phase_scale_v_per_cycle=1.0,
                       phase_period_counts=6550, signal_threshold_counts=10,
                       signal_timeout_s=0.02)
    p["calibration"].update(nm_per_cycle=316.4, direction=1)
    p["drive"].update(piezo_model="SIMULATION ONLY", capacitance_f=1e-7,
                      rated_voltage_v=5.0, minimum_v=0.0, maximum_v=5.0,
                      safe_v=0.0, slew_v_per_s=1000.0,
                      max_abs_error_nm=10000.0, max_current_a=0.02)
    p["model"].update(gain_nm_per_v=410.0, tau_s=0.008,
                      plant_delay_s=0.002, sensor_delay_s=0.0)
    p["controller"].update(kp=0.002, ki=0.2, kd=0.0,
                           d_filter_n=100.0, observer_w0=50.0)
    p["trajectory"]["offset_nm"] = 1000.0
    return p


def normalize_profile(profile: dict) -> dict:
    """Migrate only additive feature defaults; never invent rig calibration."""
    value = copy.deepcopy(profile)
    if not isinstance(value, dict):
        return value
    template = default_profile()
    additions = {"trajectory": ("phase_deg", "duty_pct", "symmetry_pct",
                                 "pulse_width_s", "edge_time_s", "noise_seed"),
                 "disturbance": tuple(template["disturbance"]),
                 "simulation_options": tuple(template["simulation_options"])}
    for section, keys in additions.items():
        if section not in value and section != "trajectory":
            value[section] = {}
        if isinstance(value.get(section), dict):
            for key in keys:
                value[section].setdefault(key, copy.deepcopy(template[section][key]))
    return value


def _reject_constant(value):
    raise ConfigurationError(f"Non-finite JSON value: {value}")


def load_profile(path: str | Path) -> dict:
    try:
        value = json.loads(Path(path).expanduser().read_text(encoding="utf-8"),
                           parse_constant=_reject_constant)
    except (OSError, ValueError) as exc:
        raise ConfigurationError(f"Cannot read hardware profile {path}: {exc}") from exc
    value = normalize_profile(value)
    problems = validate_profile(value)
    if problems:
        raise ConfigurationError("; ".join(problems))
    return value


def _atomic_json(path: Path, value: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         suffix=".tmp", delete=False) as stream:
            name = stream.name
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)
    finally:
        if name is not None:
            Path(name).unlink(missing_ok=True)


def save_profile(path: str | Path, profile: dict):
    profile = normalize_profile(profile)
    errors = validate_profile(profile)
    if errors:
        raise ConfigurationError("; ".join(errors))
    _atomic_json(Path(path).expanduser(), profile)


def load_local_settings() -> dict:
    path = settings_dir() / "local.json"
    if not path.exists():
        return {}
    value = json.loads(path.read_text(encoding="utf-8"), parse_constant=_reject_constant)
    if not isinstance(value, dict):
        raise ConfigurationError("local.json must contain an object")
    return value


def save_local_settings(value: dict):
    _atomic_json(settings_dir() / "local.json", value)


def validate_profile(profile: dict, for_output: bool = False) -> list[str]:
    profile = normalize_profile(profile)
    errors = []
    template = default_profile()
    if not isinstance(profile, dict):
        return ["Profile must be a JSON object"]
    if type(profile.get("schema_version")) is not int or profile["schema_version"] != 1:
        errors.append("schema_version must be 1")
    if type(profile.get("simulation")) is not bool:
        errors.append("simulation must be boolean")
    for section, defaults in template.items():
        if not isinstance(defaults, dict):
            continue
        obj = profile.get(section)
        if not isinstance(obj, dict):
            errors.append(f"{section} must be an object")
            continue
        for key in defaults:
            if key not in obj:
                errors.append(f"Missing {section}.{key}")
    if errors:
        return errors
    numeric = {
        "inputs": ("carrier_hz", "level_vpp", "phase_scale_v_per_cycle", "phase_period_counts",
                   "signal_threshold_counts", "signal_timeout_s"),
        "calibration": ("nm_per_cycle", "direction"),
        "drive": ("capacitance_f", "rated_voltage_v", "minimum_v", "maximum_v", "safe_v",
                  "slew_v_per_s", "max_abs_error_nm", "max_current_a", "counts_per_volt"),
        "model": tuple(template["model"]), "controller": tuple(template["controller"]),
        "trajectory": ("frequency_hz", "amplitude_nm", "offset_nm", "phase_deg", "duty_pct",
                       "symmetry_pct", "pulse_width_s", "edge_time_s", "noise_seed"),
        "disturbance": ("amplitude_v", "frequency_hz", "start_s", "duration_s", "phase_deg", "seed"),
        "simulation_options": tuple(key for key in template["simulation_options"] if key != "hysteresis_enabled"),
        "experiment": tuple(template["experiment"]),
    }
    for section, keys in numeric.items():
        for key in keys:
            value = profile[section][key]
            if value is None:
                if for_output:
                    errors.append(f"Required: {section}.{key}")
            elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                errors.append(f"{section}.{key} must be a finite number or null")
    for key, value in profile["validation"].items():
        if type(value) is not bool:
            errors.append(f"validation.{key} must be boolean")
    if profile["device"]["hardware"] != "mokugo" or profile["device"]["slots"] != 3:
        errors.append("This implementation requires mokugo with 3 slots")
    if profile["trajectory"]["waveform"] not in ("dc", "sine", "square", "ramp", "pulse", "noise"):
        errors.append("Unsupported trajectory.waveform")
    if profile["disturbance"]["waveform"] not in ("step", "sine", "square", "noise"):
        errors.append("Unsupported disturbance.waveform")
    for section, key in (("disturbance", "enabled"), ("simulation_options", "hysteresis_enabled")):
        if type(profile[section][key]) is not bool:
            errors.append(f"{section}.{key} must be boolean")
    if errors:
        return errors
    t, disturbance, simulation = (profile[key] for key in ("trajectory", "disturbance", "simulation_options"))
    added_numeric = {
        "trajectory": ("phase_deg", "duty_pct", "symmetry_pct", "pulse_width_s", "edge_time_s", "noise_seed"),
        "disturbance": tuple(key for key in template["disturbance"] if key not in ("enabled", "waveform")),
        "simulation_options": tuple(key for key in template["simulation_options"] if key != "hysteresis_enabled"),
    }
    for section, keys in added_numeric.items():
        for key in keys:
            if profile[section][key] is None:
                errors.append(f"Required: {section}.{key}")
    if errors:
        return errors
    feature_checks = [
        (0 < t["duty_pct"] < 100, "trajectory.duty_pct must be in (0, 100)"),
        (0 < t["symmetry_pct"] < 100, "trajectory.symmetry_pct must be in (0, 100)"),
        (t["pulse_width_s"] > 0 and t["edge_time_s"] >= 0, "Pulse width must be positive and edge time nonnegative"),
        (-5 <= disturbance["amplitude_v"] <= 5, "disturbance.amplitude_v must be in [-5, 5]"),
        (disturbance["frequency_hz"] > 0 and disturbance["start_s"] >= 0 and disturbance["duration_s"] > 0,
         "Disturbance frequency/duration must be positive and start nonnegative"),
        (simulation["measurement_noise_nm"] >= 0 and 0 <= simulation["dead_band_v"] <= 5,
         "Simulation noise must be nonnegative and dead band in [0, 5] V"),
        (min(simulation[key] for key in ("hysteresis_a", "hysteresis_beta", "hysteresis_gamma", "hysteresis_nm_per_v")) >= 0,
         "Hysteresis coefficients and gain must be nonnegative"),
    ]
    for section, key in (("trajectory", "noise_seed"), ("disturbance", "seed"), ("simulation_options", "noise_seed")):
        seed = profile[section][key]
        feature_checks.append((type(seed) is int and 0 <= seed < 2**32, f"{section}.{key} must be an unsigned 32-bit integer"))
    frequency = t["frequency_hz"]
    if isinstance(frequency, (int, float)) and frequency > 0:
        duty = t["pulse_width_s"] * frequency if t["waveform"] == "pulse" else t["duty_pct"] / 100
        if t["waveform"] in ("square", "pulse"):
            feature_checks.append((0 < duty < 1, "Pulse width must be shorter than one period"))
            feature_checks.append((t["edge_time_s"] * frequency <= min(duty, 1 - duty),
                                   "Waveform edge ramps overlap; reduce edge time"))
    errors.extend(message for valid, message in feature_checks if not valid)
    if errors or not for_output:
        return errors
    d, t, c, m, inp = (profile[k] for k in ("drive", "trajectory", "controller", "model", "inputs"))
    checks = [
        (-5 <= d["minimum_v"] <= d["safe_v"] <= d["maximum_v"] <= 5
         and d["minimum_v"] < d["maximum_v"], "drive voltages must satisfy -5 <= min <= safe <= max <= 5"),
        (d["rated_voltage_v"] >= max(abs(d["minimum_v"]), abs(d["maximum_v"])), "Piezo voltage rating exceeded"),
        (0 < d["max_current_a"] <= 0.025, "max_current_a must be in (0, 0.025]"),
        (d["capacitance_f"] > 0 and d["slew_v_per_s"] > 0
         and d["capacitance_f"] * d["slew_v_per_s"] <= d["max_current_a"], "C * slew exceeds drive current limit"),
        (c["rate_hz"] == 1000, "ABI v2 currently supports a 1000 Hz control rate"),
        (0 < t["frequency_hz"] < c["rate_hz"] / 2, "Trajectory frequency must be below Nyquist"),
        (t["amplitude_nm"] >= 0, "amplitude_nm must be nonnegative"),
        (m["gain_nm_per_v"] > 0 and m["tau_s"] > 0, "Model gain and tau must be positive"),
        (0 <= m["plant_delay_s"] <= .255 and 0 <= m["sensor_delay_s"] <= .255, "Model delays must be in [0, .255] s"),
        (c["kp"] >= 0 and c["ki"] >= 0 and c["kd"] >= 0 and c["d_filter_n"] > 0
         and c["observer_w0"] > 0, "Invalid controller gains/filter/observer bandwidth"),
        (profile["calibration"]["direction"] in (-1, 1) and profile["calibration"]["nm_per_cycle"] > 0,
         "Calibration needs positive nm_per_cycle and direction +/-1"),
        (1000 <= inp["carrier_hz"] <= 30e6 and inp["level_vpp"] > 0, "Carrier must be 1 kHz..30 MHz with positive level"),
        (2 <= inp["phase_period_counts"] <= 65536 and int(inp["phase_period_counts"]) == inp["phase_period_counts"], "Invalid integer phase period"),
        (inp["phase_scale_v_per_cycle"] > 0 and inp["signal_threshold_counts"] > 0
         and inp["signal_timeout_s"] > 0, "Phase scale and signal detection settings must be positive"),
        (0 <= profile["experiment"]["settle_s"] < profile["experiment"]["duration_s"] <= 3600,
         "Experiment must satisfy 0 <= settle < duration <= 3600 s"),
        (type(profile["experiment"]["repeats"]) is int and profile["experiment"]["repeats"] >= 1,
         "experiment.repeats must be a positive integer"),
        (not disturbance["enabled"] or disturbance["frequency_hz"] < c["rate_hz"] / 2,
         "Disturbance frequency must be below Nyquist"),
        (not disturbance["enabled"] or disturbance["start_s"] < profile["experiment"]["duration_s"],
         "Enabled disturbance must start before the experiment ends"),
    ]
    errors.extend(message for valid, message in checks if not valid)
    if not profile["simulation"]:
        for key, verified in profile["validation"].items():
            if not verified:
                errors.append(f"Hardware validation required: {key}")
        for section, key in (("inputs", "standard"), ("inputs", "receiver"),
                             ("drive", "piezo_model"), ("fpga", "bitstream_path"),
                             ("fpga", "manifest_path")):
            if not isinstance(profile[section][key], str) or not profile[section][key].strip():
                errors.append(f"Required: {section}.{key}")
    return errors


def clone_profile(profile: dict) -> dict:
    return copy.deepcopy(profile)
