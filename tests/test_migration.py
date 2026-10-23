import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from laserai.config import (ConfigurationError, default_profile, load_profile, save_profile,
                            simulation_profile, validate_profile)
from laserai.device import DeviceError, MokuDevice, SimulatedDevice, profile_words, verify_bitstream
from laserai.experiments import ExperimentManager, calculate_metrics, run_ab
from laserai.protocol import ABI_MAGIC, CLOCK_HZ, CONFIG_WORDS, Control


def physical_profile(tmp_path):
    p = simulation_profile()
    p["simulation"] = False
    p["device"]["address"] = "192.0.2.1"
    p["validation"] = dict.fromkeys(p["validation"], True)
    bitstream = tmp_path / "compiled package.tar"
    bitstream.write_bytes(b"TEST FIXTURE, NOT A BITSTREAM")
    manifest = {"hardware": "mokugo", "slots": 3, "slot": 2, "clock_hz": CLOCK_HZ,
                "abi_magic": ABI_MAGIC, "mokuos_version": "4.3.0", "build_id": "test-only",
                "sha256": hashlib.sha256(bitstream.read_bytes()).hexdigest()}
    target = tmp_path / "target.json"
    target.write_text(json.dumps(manifest), encoding="utf-8")
    p["fpga"].update(bitstream_path=str(bitstream), manifest_path=str(target))
    return p


class FakeCustom:
    def __init__(self):
        self.words = [0] * CONFIG_WORDS
        self.controls = [0] * 16
        self.status = [ABI_MAGIC] + [0] * 15
        self.status[12] = 1
        self.ack = True

    def get_status(self):
        self.status[5] = self.words[self.controls[8]]
        return {"status": self.status[:]}

    def set_control(self, index, value):
        self.controls[index] = value
        if not self.ack:
            return
        if index == Control.WRITE_SEQUENCE:
            self.words[self.controls[1]] = self.controls[2]
            self.status[10] = value
        elif index == Control.COMMIT_ID:
            self.status[1] = value
            self.status[12] |= 2
        elif index == Control.COMMAND_SEQUENCE:
            self.status[11] = value
            command = self.controls[6]
            if command == 3:
                self.status[12] |= 4
            else:
                self.status[2] = command


def configured_fake(tmp_path):
    profile = physical_profile(tmp_path)
    device = MokuDevice(profile, sdk=SimpleNamespace(), transaction_timeout=.015)
    device.session = SimpleNamespace(relinquish_ownership=lambda: None)
    device.info = {"mokuos_version": "4.3.0"}
    device.custom = FakeCustom()
    return device


def test_unknown_template_is_loadable_but_output_is_blocked(tmp_path):
    profile = default_profile()
    path = tmp_path / "中文 路径" / "rig.json"
    save_profile(path, profile)
    assert load_profile(path) == profile
    assert validate_profile(profile) == []
    assert "Required: drive.capacitance_f" in validate_profile(profile, for_output=True)
    with pytest.raises(ConfigurationError):
        profile_words(profile)


@pytest.mark.parametrize("contents", ["{broken", "[]", '{"schema_version": 2}', '{"x": NaN}'])
def test_corrupt_config_never_falls_back(tmp_path, contents):
    path = tmp_path / "hardware.json"
    path.write_text(contents, encoding="utf-8")
    with pytest.raises(ConfigurationError):
        load_profile(path)


def test_templates_match_code():
    root = Path(__file__).resolve().parents[1]
    assert load_profile(root / "configs/moku-go.template.json") == default_profile()
    assert load_profile(root / "configs/simulation.json") == simulation_profile()


def test_current_limit_and_non_finite_are_blocked():
    p = simulation_profile()
    p["drive"]["capacitance_f"] = 1e-3
    assert any("current" in e for e in validate_profile(p, True))
    p["controller"]["kp"] = float("inf")
    assert any("finite" in e for e in validate_profile(p, True))


def test_real_session_rejects_simulation_profile():
    with pytest.raises(DeviceError, match="simulation"):
        MokuDevice(simulation_profile())


def test_manifest_checks_all_targets_and_hash(tmp_path):
    p = physical_profile(tmp_path)
    assert verify_bitstream(p, {"mokuos_version": "4.3.0"})["build_id"] == "test-only"
    with pytest.raises(DeviceError, match="target mismatch"):
        verify_bitstream(p, {"mokuos_version": "4.2.0"})
    Path(p["fpga"]["bitstream_path"]).write_bytes(b"changed")
    with pytest.raises(DeviceError, match="SHA256"):
        verify_bitstream(p, {"mokuos_version": "4.3.0"})


def test_transaction_ack_readback_and_zero_gate(tmp_path):
    d = configured_fake(tmp_path)
    assert d.configure()["commit_id"] == 1
    assert d.configured
    d.start("pid")
    assert d.running
    with pytest.raises(DeviceError, match="Stop"):
        d.configure()
    d.stop()
    assert d.configure()["commit_id"] == 2
    d.close()
    assert not d.configured


def test_transaction_timeout_locks_output(tmp_path):
    d = configured_fake(tmp_path)
    d.custom.ack = False
    with pytest.raises(DeviceError, match="timed out"):
        d.configure()
    assert not d.configured
    with pytest.raises(DeviceError, match="acknowledged"):
        d.start()


def test_snapshot_metrics_are_not_accepted(tmp_path):
    profile = simulation_profile()
    samples = [{"time_s": i / 10, "measurement_nm": 0, "reference_nm": 0,
                "drive_v": 1, "coherent": False, "faults": 0} for i in range(100)]
    metrics = calculate_metrics(samples, profile)
    assert metrics["rmse_nm"] == 0
    assert not metrics["eligible_for_acceptance"]
    assert not metrics["hardware_target_verified"]


def test_full_tick_data_does_not_depend_on_display_polling(tmp_path):
    p = simulation_profile()
    clock = [0.0]
    d = SimulatedDevice(p, clock=lambda: clock[0])
    d.connect()
    d.deploy(False)
    d.configure()
    d.start("preview")
    clock[0] = 10.0
    telemetry = d.read_telemetry()
    assert len(telemetry["samples"]) == 10000
    assert not d.running
    manager = ExperimentManager(tmp_path / "中文 实验")
    path = manager.begin(p, d.info, "preview")
    manager.append(telemetry)
    assert d.read_telemetry()["samples"] == []
    result = manager.finish("completed")
    assert result["metrics"]["samples"] == 8000
    assert result["metrics"]["continuous"]
    assert not result["metrics"]["hardware_target_verified"]
    assert json.loads((path / "snapshot.json").read_text())["device"]["bitstream"]


def test_ab_repeats_alternate_order_and_save_results(tmp_path):
    p = simulation_profile()
    p["experiment"].update(duration_s=.1, settle_s=.02, repeats=2)
    clock = [0.0]
    def advance(dt):
        clock[0] += dt
    result = run_ab(p, lambda p: SimulatedDevice(p, clock=lambda: clock[0]),
                    tmp_path, sleep=advance)
    assert [r["mode"] for r in result["runs"]] == ["pid", "preview", "preview", "pid"]
    assert Path(result["summary_path"]).is_file()
    assert len(list(tmp_path.glob("*/snapshot.json"))) == 4
    assert not result["hardware_target_verified"]


def test_missing_zero_confirmation_blocks_start(tmp_path):
    d = configured_fake(tmp_path)
    original = d.custom.set_control
    def reject_zero(index, value):
        original(index, value)
        d.custom.status[12] &= ~4
    d.custom.set_control = reject_zero
    with pytest.raises(DeviceError, match="zero rejected"):
        d.configure()
    assert not d.configured


def test_rejected_commit_id_is_not_reused(tmp_path):
    d = configured_fake(tmp_path)
    d.custom.status[14] = 12
    assert d.configure()["commit_id"] == 13


def test_normal_duration_event_does_not_invalidate_metrics():
    p = simulation_profile()
    p["experiment"].update(duration_s=.1, settle_s=.02)
    samples = [{"time_s": i / 1000, "measurement_nm": 1000, "reference_nm": 1000,
                "drive_v": 2, "coherent": True, "faults": 16 if i == 99 else 0}
               for i in range(100)]
    assert calculate_metrics(samples, p)["eligible_for_acceptance"]


def test_stop_waits_for_safe_ramp_before_redeploy(tmp_path):
    d = configured_fake(tmp_path)
    d.running = True
    original = d.custom.get_status
    ramp_samples = [3, 3, 0]
    def status():
        if d.custom.controls[Control.COMMAND_SEQUENCE]:
            d.custom.status[2] = ramp_samples.pop(0) if ramp_samples else 0
        return original()
    d.custom.get_status = status
    d.stop()
    assert not ramp_samples
    assert not d.running


@pytest.mark.parametrize("reject_second_input", [False, True])
def test_measurement_deployment_requires_independent_phase_inputs(tmp_path, reject_second_input):
    calls = []
    class Native:
        def __getattr__(self, name):
            def call(*args, **kwargs):
                calls.append((name, args, kwargs))
                if name == "enable_input" and args[0] == 2 and reject_second_input:
                    raise DeviceError("Second phase input unavailable")
            return call
    native = Native()
    sdk = SimpleNamespace(Phasemeter="phasemeter", WaveformGenerator="zero", Datalogger="logger")
    class Session:
        def set_instrument(self, slot, instrument):
            return native
        def __getattr__(self, name):
            return getattr(native, name)
    sdk.MultiInstrument = lambda *args, **kwargs: Session()
    d = MokuDevice(physical_profile(tmp_path), sdk=sdk)
    d.session = Session()
    d.info = {"mokuos_version": "4.3.0"}
    if reject_second_input:
        with pytest.raises(DeviceError, match="Second phase"):
            d.deploy(True)
        assert not any(name == "set_connections" for name, args, kw in calls)
        assert not d.configured
    else:
        assert d.deploy(True)["deployment"] == "measurement"
    assert ("enable_single_input", (), {"enable": False}) in calls


def test_shutdown_ramp_is_excluded_from_steady_metrics():
    p = simulation_profile()
    p["experiment"].update(duration_s=.1, settle_s=.02)
    samples = [{"time_s": i / 1000, "measurement_nm": 1000 if i < 100 else 0,
                "reference_nm": 1000, "drive_v": 2, "coherent": True, "faults": 0}
               for i in range(150)]
    metrics = calculate_metrics(samples, p)
    assert metrics["samples"] == 80
    assert metrics["rmse_nm"] == 0
