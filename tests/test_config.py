"""Profile migration preserves unknown calibration and validates new controls."""
import copy

import pytest

from laserai.config import (ConfigurationError, load_profile, normalize_profile,
                            save_profile, simulation_profile, validate_profile)


def test_additive_defaults_preserve_original_unknowns_and_do_not_mutate():
    profile = simulation_profile()
    for key in ("phase_deg", "duty_pct", "symmetry_pct", "pulse_width_s", "edge_time_s", "noise_seed"):
        del profile["trajectory"][key]
    del profile["disturbance"]
    del profile["simulation_options"]
    profile["drive"]["safe_v"] = None
    original = copy.deepcopy(profile)
    migrated = normalize_profile(profile)
    assert profile == original
    assert migrated["drive"]["safe_v"] is None
    assert migrated["disturbance"]["enabled"] is False
    assert migrated["simulation_options"]["measurement_noise_nm"] == 0
    assert "Required: drive.safe_v" in validate_profile(migrated, True)
    del profile["calibration"]["nm_per_cycle"]
    assert "Missing calibration.nm_per_cycle" in validate_profile(profile)


def test_old_profiles_are_saved_with_new_options(tmp_path):
    profile = simulation_profile()
    del profile["disturbance"]
    del profile["simulation_options"]
    path = tmp_path / "中文 空格" / "profile.json"
    save_profile(path, profile)
    assert load_profile(path) == normalize_profile(profile)


@pytest.mark.parametrize("section,key,value", [
    ("trajectory", "waveform", "sawtooth"),
    ("trajectory", "phase_deg", True),
    ("trajectory", "edge_time_s", -1),
    ("trajectory", "duty_pct", 100),
    ("trajectory", "symmetry_pct", 0),
    ("trajectory", "noise_seed", 1.5),
    ("trajectory", "noise_seed", 2**32),
    ("disturbance", "enabled", 1),
    ("disturbance", "waveform", "ramp"),
    ("disturbance", "amplitude_v", 5.01),
    ("disturbance", "duration_s", 0),
    ("disturbance", "seed", -1),
    ("simulation_options", "noise_seed", False),
    ("simulation_options", "hysteresis_enabled", "false"),
    ("simulation_options", "hysteresis_beta", -0.1),
    ("simulation_options", "measurement_noise_nm", float("nan")),
    ("simulation_options", "dead_band_v", 5.1),
    ("simulation_options", "hysteresis_a", None),
])
def test_invalid_feature_values_are_rejected(section, key, value):
    profile = simulation_profile()
    profile[section][key] = value
    assert validate_profile(profile)
    with pytest.raises(ConfigurationError):
        save_profile("unused.json", profile)


def test_pulse_width_and_total_transition_duration_do_not_overlap():
    profile = simulation_profile()
    profile["trajectory"].update(waveform="pulse", frequency_hz=20, pulse_width_s=.01, edge_time_s=.01)
    assert validate_profile(profile, True) == []
    profile["trajectory"]["edge_time_s"] = .0101
    assert any("overlap" in error for error in validate_profile(profile, True))
    profile["trajectory"].update(edge_time_s=0, pulse_width_s=.05)
    assert any("one period" in error for error in validate_profile(profile, True))


def test_disabled_disturbance_allows_unused_frequency():
    profile = simulation_profile()
    profile["disturbance"]["frequency_hz"] = 10000
    assert validate_profile(profile, True) == []
    profile["disturbance"]["enabled"] = True
    assert any("Nyquist" in error for error in validate_profile(profile, True))


def test_disturbance_must_activate_during_experiment():
    profile = simulation_profile()
    profile["disturbance"].update(enabled=True, start_s=10)
    assert any("before" in error for error in validate_profile(profile, True))
