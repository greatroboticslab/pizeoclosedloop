"""Device lifecycle and actual waveform, perturbation and plant effects."""
import copy
import random

import pytest

from laserai.config import ConfigurationError, simulation_profile
from laserai.device import DeviceError, MokuDevice, SimulatedDevice, profile_words
from laserai.protocol import CONFIG_WORDS, FIELDS, board_reference


def simulate(profile, mode="open_loop", *, plant_override=None):
    clock = [0.0]
    device = SimulatedDevice(profile, clock=lambda: clock[0], plant_override=plant_override)
    device.connect()
    device.deploy(False)
    device.configure()
    device.start(mode)
    clock[0] = profile["experiment"]["duration_s"]
    return device, device.read_telemetry()["samples"]


@pytest.mark.parametrize("waveform,code", [("dc", 0), ("sine", 1), ("square", 2),
                                          ("ramp", 3), ("pulse", 4), ("noise", 5)])
def test_all_waveform_profiles_compile(waveform, code):
    profile = simulation_profile()
    profile["trajectory"].update(waveform=waveform, phase_deg=450, noise_seed=123)
    profile["model"]["description"] = "extra model metadata"
    words = profile_words(profile)
    assert len(words) == CONFIG_WORDS
    assert words[FIELDS["waveform"][0]] == code
    assert words[FIELDS["phase_initial"][0]] == 2**30
    assert words[FIELDS["trajectory_seed"][0]] == 123


def test_phase_duty_pulse_and_edge_encoding():
    profile = simulation_profile()
    profile["trajectory"].update(waveform="pulse", pulse_width_s=.01, edge_time_s=.002)
    words = profile_words(profile)
    assert words[36] / 2**24 == pytest.approx(.2, abs=1e-7)
    assert words[38] / 2**24 == pytest.approx(.02, abs=1e-7)
    profile["trajectory"].update(waveform="square", duty_pct=35)
    assert profile_words(profile)[36] / 2**24 == pytest.approx(.35, abs=1e-7)
    profile["trajectory"].update(waveform="sine", frequency_hz=100, edge_time_s=1)
    words = profile_words(profile)
    assert words[36] == 2**23 and words[38] == 0
    assert board_reference(words, 0)[0] == 1000


def test_noise_is_reproducible_and_common_to_runs():
    profile = simulation_profile()
    profile["experiment"].update(duration_s=.05, settle_s=0)
    _, clean = simulate(profile)
    profile["simulation_options"].update(measurement_noise_nm=3, noise_seed=731)
    _, noisy = simulate(profile)
    _, repeated = simulate(profile)
    assert noisy == repeated
    rng = random.Random(731)
    assert [x["measurement_nm"] - y["measurement_nm"] for x, y in zip(noisy, clean)] == pytest.approx(
        [rng.gauss(0, 3) for _ in noisy])
    _, pid = simulate(profile, "pid")
    _, preview = simulate(profile, "preview")
    assert pid[0]["measurement_nm"] == preview[0]["measurement_nm"] == noisy[0]["measurement_nm"]


def test_disabled_nonlinearity_is_neutral_and_enabled_options_change_output():
    profile = simulation_profile()
    profile["experiment"].update(duration_s=.1, settle_s=0)
    profile["trajectory"].update(waveform="dc", amplitude_nm=0, offset_nm=1000)
    _, baseline = simulate(profile)
    profile["simulation_options"]["hysteresis_nm_per_v"] = 50
    _, disabled = simulate(profile)
    assert disabled == baseline
    profile["simulation_options"]["hysteresis_enabled"] = True
    _, hysteresis = simulate(profile)
    assert hysteresis[-1]["measurement_nm"] < baseline[-1]["measurement_nm"]
    profile["simulation_options"].update(hysteresis_enabled=False, dead_band_v=.2)
    _, deadband = simulate(profile)
    assert deadband[-1]["measurement_nm"] < baseline[-1]["measurement_nm"]


def test_disturbance_window_and_shared_protection_affect_output():
    profile = simulation_profile()
    profile["experiment"].update(duration_s=.1, settle_s=0)
    profile["trajectory"].update(waveform="dc", amplitude_nm=0, offset_nm=1000)
    profile["drive"]["slew_v_per_s"] = 100
    _, clean = simulate(profile)
    profile["disturbance"].update(enabled=True, amplitude_v=-5, start_s=.02, duration_s=.02)
    _, perturbed = simulate(profile)
    assert [sample["disturbance_v"] for sample in perturbed] == [0]*20 + [-5]*20 + [0]*60
    assert perturbed[35]["measurement_nm"] < clean[35]["measurement_nm"]
    volts = [0] + [sample["drive_v"] for sample in perturbed]
    assert min(volts) >= 0 and max(volts) <= 5
    assert max(abs(b - a) for a, b in zip(volts, volts[1:])) <= .10002


def test_profile_update_invalidates_commit_and_refreshes_effective_plant():
    profile = simulation_profile()
    device = SimulatedDevice(profile, plant_override={"tau_s": .01})
    device.connect()
    device.deploy(False)
    device.configure()
    updated = copy.deepcopy(profile)
    updated["model"].update(gain_nm_per_v=500, tau_s=.03)
    updated["simulation_options"]["dead_band_v"] = .4
    device.update_profile(updated)
    updated["model"]["gain_nm_per_v"] = 1
    assert not device.configured
    assert device.info["effective_plant"]["gain_nm_per_v"] == 500
    assert device.info["effective_plant"]["tau_s"] == .01
    assert device.info["simulation_options"]["dead_band_v"] == .4
    with pytest.raises(DeviceError, match="Configured"):
        device.start()
    device.configure()
    device.start()
    with pytest.raises(DeviceError, match="Stop"):
        device.update_profile(profile)
    with pytest.raises(DeviceError, match="Stop"):
        device.deploy(False)


@pytest.mark.parametrize("section,key,value", [("device", "address", "other"),
    ("inputs", "carrier_hz", 2e6), ("fpga", "bitstream_path", "/tmp/new.tar")])
@pytest.mark.parametrize("physical", [False, True])
def test_connected_route_changes_require_disconnect(section, key, value, physical):
    profile = simulation_profile()
    profile["simulation"] = not physical
    if physical:
        device = MokuDevice(profile)
        device.session = object()
    else:
        device = SimulatedDevice(profile)
        device.connect()
    profile[section][key] = value
    with pytest.raises(DeviceError, match="Disconnect"):
        device.update_profile(profile)


def test_physical_open_loop_cannot_bypass_limits_with_disturbance():
    profile = simulation_profile()
    profile["simulation"] = False
    profile["trajectory"].update(amplitude_nm=10, offset_nm=0)
    profile["disturbance"]["enabled"] = True
    device = MokuDevice(profile)
    device.configured = True
    device.custom = object()
    with pytest.raises(DeviceError, match="disturbance"):
        device.start("open_loop")
    assert not device.running


@pytest.mark.parametrize("override", [{"tau_s": 0}, {"gain_nm_per_v": -1}, {"plant_delay_s": 1}])
def test_invalid_effective_simulated_plant_is_blocked(override):
    device = SimulatedDevice(simulation_profile(), plant_override=override)
    device.connect()
    device.deploy(False)
    with pytest.raises(ConfigurationError, match="simulated plant"):
        device.configure()
    assert not device.configured
