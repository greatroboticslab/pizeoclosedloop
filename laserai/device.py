"""Serialized device sessions. Host calls never implement a real-time hardware loop."""

from __future__ import annotations

from collections import deque
import copy
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import re
import random
import shutil
import subprocess
import time

from .config import ConfigurationError, normalize_profile, validate_profile
from .protocol import (ABI_MAGIC, CLOCK_HZ, Command, Control, FixedController,
                       board_reference, board_disturbance, compile_config, decode_status, prepare_coefficients, staging_writes)


class DeviceError(RuntimeError):
    pass


def discover_cli(explicit: str | None = None) -> dict:
    candidate = explicit or os.environ.get("MOKU_CLI_PATH") or shutil.which("mokucli")
    source = "explicit" if explicit else "environment" if os.environ.get("MOKU_CLI_PATH") else "PATH / SDK"
    result = {"path": None, "source": source, "ok": False}
    if not candidate:
        result["error"] = "Install MokuCLI or set MOKU_CLI_PATH"
        return result
    path = Path(candidate).expanduser()
    result["path"] = str(path)
    if not path.is_file():
        result["error"] = "MokuCLI executable does not exist"
        return result
    try:
        process = subprocess.run([str(path), "--version"], capture_output=True,
                                 text=True, timeout=5, check=True)
        result.update(ok=True, version=(process.stdout or process.stderr).strip())
    except (OSError, subprocess.SubprocessError) as exc:
        result["error"] = str(exc)
    return result


def profile_words(profile: dict) -> list[int]:
    profile = normalize_profile(profile)
    errors = validate_profile(profile, for_output=True)
    if errors:
        raise ConfigurationError("; ".join(errors))
    inp, d, m, c, t, exp = (profile[k] for k in
                            ("inputs", "drive", "model", "controller", "trajectory", "experiment"))
    dt = 1 / c["rate_hz"]
    model_values = {key: m[key] for key in ("gain_nm_per_v", "tau_s", "plant_delay_s", "sensor_delay_s")}
    values = prepare_coefficients(**model_values, dt_s=dt, kp=c["kp"], ki=c["ki"], kd=c["kd"],
                                  d_filter_n=c["d_filter_n"], w0=c["observer_w0"],
                                  v_max=max(abs(d["minimum_v"]), abs(d["maximum_v"])))
    values.update(
        duration_ticks=round(exp["duration_s"] / dt), heartbeat_timeout_ticks=0,
        phase_period_counts=inp["phase_period_counts"],
        nm_per_phase_count=profile["calibration"]["nm_per_cycle"] / inp["phase_period_counts"],
        direction=profile["calibration"]["direction"], v_min=d["minimum_v"],
        v_max=d["maximum_v"], v_safe=d["safe_v"], slew_per_tick=d["slew_v_per_s"] * dt,
        max_abs_error=d["max_abs_error_nm"], signal_threshold=inp["signal_threshold_counts"],
        signal_timeout_ticks=max(1, round(inp["signal_timeout_s"] / dt)),
        dac_counts_per_volt=d["counts_per_volt"],
        waveform={"dc": 0, "sine": 1, "square": 2, "ramp": 3, "pulse": 4, "noise": 5}[t["waveform"]],
        amplitude=t["amplitude_nm"], offset=t["offset_nm"],
        phase_step=round(t["frequency_hz"] * dt * 2**32),
        phase_initial=round((t["phase_deg"] % 360) / 360 * 2**32) & 0xFFFFFFFF,
        duty=(t["pulse_width_s"] * t["frequency_hz"] if t["waveform"] == "pulse"
              else t["duty_pct"] / 100 if t["waveform"] == "square" else .5),
        symmetry=t["symmetry_pct"] / 100,
        edge_fraction=(t["edge_time_s"] * t["frequency_hz"] / 2
                       if t["waveform"] in ("square", "pulse") else 0),
        trajectory_seed=t["noise_seed"],
    )
    disturbance = profile["disturbance"]
    values.update(
        disturbance_mode={"step": 1, "sine": 2, "square": 3, "noise": 4}[disturbance["waveform"]]
                         if disturbance["enabled"] else 0,
        disturbance_amplitude=disturbance["amplitude_v"],
        disturbance_phase_step=(round(disturbance["frequency_hz"] * dt * 2**32)
                                if disturbance["enabled"] else 0),
        disturbance_phase_initial=round((disturbance["phase_deg"] % 360) / 360 * 2**32) & 0xFFFFFFFF,
        disturbance_start_ticks=round(disturbance["start_s"] / dt),
        disturbance_duration_ticks=max(1, round(disturbance["duration_s"] / dt)),
        disturbance_seed=disturbance["seed"],
    )
    return compile_config(values)


def verify_bitstream(profile: dict, device_info: dict) -> dict:
    bitstream = Path(profile["fpga"]["bitstream_path"] or "").expanduser()
    manifest_file = Path(profile["fpga"]["manifest_path"] or "").expanduser()
    if not bitstream.is_file() or not manifest_file.is_file():
        raise DeviceError("Compiled bitstream package and target manifest are required")
    try:
        manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise DeviceError(f"Invalid bitstream manifest: {exc}") from exc
    required = {"hardware": "mokugo", "slots": 3, "slot": 2,
                "clock_hz": CLOCK_HZ, "abi_magic": ABI_MAGIC,
                "mokuos_version": device_info.get("mokuos_version")}
    if not isinstance(manifest, dict):
        raise DeviceError("Bitstream manifest must be an object")
    for key, expected in required.items():
        if expected is None or manifest.get(key) != expected:
            raise DeviceError(f"Bitstream target mismatch: {key}, expected {expected!r}")
    digest = hashlib.sha256(bitstream.read_bytes()).hexdigest()
    if manifest.get("sha256") != digest:
        raise DeviceError("Bitstream SHA256 mismatch")
    if not isinstance(manifest.get("build_id"), str) or not manifest["build_id"]:
        raise DeviceError("Bitstream manifest needs build_id")
    return manifest


def _status_words(payload) -> list[int]:
    if isinstance(payload, dict):
        if "status" in payload:
            return _status_words(payload["status"])
        for pattern in ("{}", "Status{}", "status{}"):
            if all(pattern.format(i) in payload for i in range(16)):
                return [int(payload[pattern.format(i)]) for i in range(16)]
    if isinstance(payload, (list, tuple)) and len(payload) == 16:
        return [int(x) for x in payload]
    raise DeviceError("Unknown SDK status-register format; no output enabled")


class MokuDevice:
    def __init__(self, profile: dict, cli_path: str | None = None, *, sdk=None,
                 transaction_timeout: float = 3.0):
        profile = normalize_profile(profile)
        errors = validate_profile(profile)
        if errors:
            raise ConfigurationError("; ".join(errors))
        if profile["simulation"]:
            raise DeviceError("A simulation profile cannot connect to physical hardware")
        self.profile = copy.deepcopy(profile)
        self.cli_path = cli_path
        self._sdk = sdk
        self._injected_sdk = sdk is not None
        self.timeout = transaction_timeout
        self.session = self.pm = self.custom = self.logger = None
        self.info = {}
        self.manifest = None
        self.configured = False
        self.running = False
        self._sequence = self._commit = self._command_sequence = 0
        self._started_tick = 0
        self._last_tick = None
        self._tick_total = 0

    def update_profile(self, profile: dict):
        if self.running:
            raise DeviceError("Stop before updating the profile")
        updated = normalize_profile(profile)
        errors = validate_profile(updated)
        if errors:
            raise ConfigurationError("; ".join(errors))
        if updated["simulation"]:
            raise DeviceError("A hardware session cannot switch to simulation")
        if self.session is not None and any(updated[key] != self.profile[key]
                                            for key in ("device", "inputs", "fpga")):
            raise DeviceError("Disconnect before changing device, inputs or bitstream")
        self.profile = updated
        self.configured = False

    def _load_sdk(self):
        if self._sdk is not None:
            return self._sdk
        cli = discover_cli(self.cli_path)
        if cli["ok"]:
            os.environ["MOKU_CLI_PATH"] = cli["path"]
        elif self.cli_path or os.environ.get("MOKU_CLI_PATH"):
            raise DeviceError(cli.get("error", "Invalid MokuCLI"))
        import moku
        from moku import instruments
        if cli["ok"]:
            moku.MOKU_CLI_PATH = cli["path"]
        self._sdk = type("SDK", (), {"Moku": moku.Moku, "MultiInstrument": instruments.MultiInstrument,
                                     "Phasemeter": instruments.Phasemeter,
                                     "CustomInstrument": instruments.CustomInstrument,
                                     "Datalogger": instruments.Datalogger,
                                     "WaveformGenerator": instruments.WaveformGenerator})
        return self._sdk

    def connect(self) -> dict:
        if self.session is not None:
            raise DeviceError("Already connected")
        address = self.profile["device"]["address"]
        if not isinstance(address, str) or not address.strip():
            raise DeviceError("Set device.address for Ethernet connection")
        sdk = self._load_sdk()
        self.session = sdk.Moku(address, force_connect=False, connect_timeout=5, read_timeout=5)
        try:
            description = self.session.describe()
            hardware = str(description.get("hardware", "")).replace(":", "").replace(" ", "").lower()
            if hardware != "mokugo":
                raise DeviceError(f"Expected Moku:Go, found {hardware!r}")
            version = str(description.get("mokuOS", ""))
            self.info = {"simulation": False, "hardware": hardware, "mokuos_version": version,
                         "serial": self.session.serial_number(), "description": description,
                         "sdk_version": importlib.metadata.version("moku"),
                         "bitstream": None, "telemetry_kind": "status_snapshot"}
            return copy.deepcopy(self.info)
        except Exception:
            self.close()
            raise

    def _connected(self):
        if self.session is None:
            raise DeviceError("Device is disconnected")

    def diagnose(self) -> dict:
        self._connected()
        return {"device": self.info, "cli": discover_cli(self.cli_path),
                "live_description": self.session.describe(),
                "output_blockers": validate_profile(self.profile, for_output=True),
                "required_capabilities": {"slots": 3, "phasemeter": True, "custom_status_registers": 16,
                                          "minimum_mokuos": "4.1.1", "logger_channels": 2},
                "capability_check": "Instrument availability is confirmed by deployment, not inferred from model name"}

    def deploy(self, measurement_only: bool = True) -> dict:
        self._connected()
        if self.running:
            raise DeviceError("Stop before deploying")
        if self._sdk is None:
            raise DeviceError("SDK unavailable")
        inp = self.profile["inputs"]
        for key in ("carrier_hz", "phase_scale_v_per_cycle"):
            value = inp[key]
            if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
                raise ConfigurationError(f"Measurement deployment requires inputs.{key}")
        if not 1000 <= inp["carrier_hz"] <= 30e6:
            raise ConfigurationError("Moku:Go carrier must be 1 kHz..30 MHz")
        if not measurement_only:
            # Validate target and all parameters before any deployment or output routing.
            profile_words(self.profile)
            version = tuple(int(x) for x in re.findall(r"\d+", self.info["mokuos_version"])[:3])
            if version < (4, 1, 1):
                raise DeviceError("Custom status wrapper requires MokuOS >=4.1.1")
            self.manifest = verify_bitstream(self.profile, self.info)
        if not self._injected_sdk and not discover_cli(self.cli_path)["ok"]:
            raise DeviceError("MokuCLI is required for instrument deployment")
        self.configured = False
        self.pm = self.custom = self.logger = None
        self.session.relinquish_ownership()
        self.session = None
        try:
            self.session = self._sdk.MultiInstrument(self.profile["device"]["address"], platform_id=3,
                                                    force_connect=False, force_deploy=True,
                                                    connect_timeout=5, read_timeout=5)
            self.pm = self.session.set_instrument(1, self._sdk.Phasemeter)
            if measurement_only:
                zero = self.session.set_instrument(2, self._sdk.WaveformGenerator)
                for channel in (1, 2):
                    zero.generate_waveform(channel, "DC", dc_level=0)
            else:
                self.custom = self.session.set_instrument(2, self._sdk.CustomInstrument,
                                    bitstream=str(Path(self.profile["fpga"]["bitstream_path"]).expanduser()))
                self._status()  # Reject wrong ABI before connecting a DAC.
            self.logger = self.session.set_instrument(3, self._sdk.Datalogger)
            self.pm.enable_single_input(enable=False)
            for channel in (1, 2):
                self.session.set_frontend(channel, impedance=inp["impedance"], coupling=inp["coupling"],
                                          attenuation="0dB")
                self.session.set_output(channel, output_gain="0dB")
                self.pm.enable_input(channel)
                self.pm.set_pm_loop(channel, auto_acquire=False, frequency=inp["carrier_hz"],
                                    bandwidth=inp["pll_bandwidth"])
                self.pm.generate_output(channel, signal="Phase", scaling=inp["phase_scale_v_per_cycle"])
            self.pm.set_phase_wrap("2pi")
            connections = [
                {"source": "Input1", "destination": "Slot1InA"},
                {"source": "Input2", "destination": "Slot1InB"},
                {"source": "Slot2OutA", "destination": "Output1"},
            ]
            if measurement_only:
                connections.extend([
                    {"source": "Slot2OutB", "destination": "Output2"},
                    {"source": "Slot1OutA", "destination": "Slot3InA"},
                    {"source": "Slot1OutB", "destination": "Slot3InB"},
                ])
            else:
                # OutputB is internal telemetry. Output2 has no external cable
                # in this single-axis rig and is never routed to that stream.
                connections.extend([
                    {"source": "Slot1OutA", "destination": "Slot2InA"},
                    {"source": "Slot1OutB", "destination": "Slot2InB"},
                    {"source": "Slot2OutA", "destination": "Slot3InA"},
                    {"source": "Slot2OutB", "destination": "Slot3InB"},
                ])
            self.session.set_connections(connections)
            self.session.sync()
            self.logger.enable_input(1)
            self.logger.enable_input(2)
            self.logger.set_samplerate(1000)
            self.info["bitstream"] = self.manifest if not measurement_only else None
            self.info["deployment"] = "measurement" if measurement_only else "controller"
            return copy.deepcopy(self.info)
        except Exception:
            self.configured = False
            self.running = False
            raise

    def validate_measurement(self) -> dict:
        if self.pm is None:
            raise DeviceError("Deploy measurement instrument first")
        readings = [self.pm.get_data() for _ in range(8)]
        return {"readings": readings, "validation_accepted": False,
                "note": "Static native readings do not establish wrap, latency, polarity or loss-of-lock behavior"}

    def _status(self):
        if self.custom is None:
            raise DeviceError("Custom controller is not deployed")
        words = _status_words(self.custom.get_status())
        return decode_status(words)

    def _wait(self, predicate, label):
        deadline = time.monotonic() + self.timeout
        while True:
            status = self._status()
            if predicate(status):
                return status
            if time.monotonic() >= deadline:
                raise DeviceError(f"FPGA {label} acknowledgement timed out")
            time.sleep(.005)

    def _command(self, command):
        current = self._status()
        self._command_sequence = (current.command_sequence + 1) & 0xFFFFFFFF or 1
        self.custom.set_control(Control.COMMAND, int(command))
        self.custom.set_control(Control.COMMAND_SEQUENCE, self._command_sequence)
        return self._wait(lambda s: s.command_sequence == self._command_sequence, "command")

    def configure(self) -> dict:
        self._connected()
        if self.running:
            raise DeviceError("Stop before reconfiguring")
        self.configured = False
        words = profile_words(self.profile)
        verify_bitstream(self.profile, self.info)
        self._command(Command.STOP)
        self._wait(lambda s: s.state == 0, "safe stop")
        current = self._status()
        self._sequence = current.write_sequence + 1
        self._commit = (max(current.commit_seen, current.commit_id) + 1) & 0xFFFFFFFF or 1
        try:
            for index, value in staging_writes(words, commit_id=self._commit,
                                                first_sequence=self._sequence):
                self.custom.set_control(int(index), value)
                if index == Control.WRITE_SEQUENCE:
                    self._wait(lambda s: s.write_sequence == value, "staging write")
            status = self._wait(lambda s: s.commit_id == self._commit, "commit")
            if status.faults:
                raise DeviceError(f"FPGA rejected configuration: {status.faults}")
            for address, expected in enumerate(words):
                self.custom.set_control(Control.STATUS_PAGE, address)
                self._wait(lambda s: (s.page_data & 0xFFFFFFFF) == (expected & 0xFFFFFFFF),
                           f"readback {address}")
            status = self._command(Command.ZERO)
            if not status.zeroed or status.faults:
                raise DeviceError("FPGA zero rejected; check input signal presence and retry")
            self.configured = True
            return {"commit_id": self._commit, "words": words, "bitstream": self.manifest}
        except Exception:
            self.configured = False
            try:
                self._command(Command.STOP)
            except Exception:
                pass
            raise

    def start(self, mode: str = "pid"):
        if not self.configured or self.custom is None:
            raise DeviceError("A verified, acknowledged configuration is required")
        commands = {"pid": Command.START_PID, "preview": Command.START_PREVIEW,
                    "open_loop": Command.START_OPEN_LOOP}
        if mode not in commands:
            raise DeviceError(f"Unknown mode: {mode}")
        if mode == "open_loop" and (self.profile["trajectory"]["amplitude_nm"] > 50
                                    or abs(self.profile["trajectory"]["offset_nm"]) > 50):
            raise DeviceError("Initial open-loop validation is limited to 50 nm amplitude and offset")
        if mode == "open_loop" and self.profile["disturbance"]["enabled"]:
            raise DeviceError("Disable drive disturbance for initial physical open-loop validation")
        try:
            status = self._command(commands[mode])
            if status.faults or status.state not in (1, 2, 4):
                raise DeviceError(f"FPGA refused start: state={status.state}, faults={status.faults}")
            self._started_tick = status.tick
            self._last_tick = status.tick
            self._tick_total = 0
            self.running = True
        except Exception:
            self.configured = False
            raise

    def stop(self):
        if self.custom is not None:
            try:
                self._command(Command.STOP)
                self._wait(lambda s: s.state == 0, "safe stop")
            except Exception:
                self.configured = False
                raise
        self.running = False

    def read_telemetry(self) -> dict:
        self._connected()
        if self.custom is None:
            return {"state": "measurement" if self.pm else "connected", "coherent": False,
                    "time_s": 0.0, "faults": 0}
        try:
            status = self._status()
        except Exception:
            self.running = self.configured = False
            raise
        if self._last_tick is not None:
            self._tick_total += (status.tick - self._last_tick) & 0xFFFFFFFF
        self._last_tick = status.tick
        self.running = status.state in (1, 2, 3, 4)
        return {"time_s": self._tick_total / self.profile["controller"]["rate_hz"],
                "tick": status.tick, "measurement_nm": status.measurement_nm,
                "reference_nm": status.reference_nm, "drive_v": status.voltage_v,
                "residual_nm": status.residual_nm, "state": status.state,
                "faults": int(status.faults), "commit_id": status.commit_id,
                "coherent": False, "sample_kind": "status_snapshot", "running": self.running}

    def start_board_recording(self) -> dict:
        if self.logger is None:
            raise DeviceError("Datalogger is not deployed")
        return self.logger.start_logging(duration=self.profile["experiment"]["duration_s"],
                                          file_name_prefix="laserai", comments=json.dumps(self.info))

    def close(self):
        try:
            self.stop()
        finally:
            if self.session is not None:
                try:
                    self.session.relinquish_ownership()
                finally:
                    self.session = None
            self.pm = self.custom = self.logger = None
            self.configured = self.running = False


class SimulatedDevice:
    """Offline fixed-point rig; computes missed ticks independently of UI refresh."""

    def __init__(self, profile: dict, *, clock=time.monotonic, plant_override: dict | None = None):
        profile = normalize_profile(profile)
        errors = validate_profile(profile)
        if errors:
            raise ConfigurationError("; ".join(errors))
        if not profile.get("simulation"):
            raise DeviceError("Choose an explicitly labelled simulation profile")
        self.profile = copy.deepcopy(profile)
        self.clock = clock
        self._plant_override = copy.deepcopy(plant_override or {})
        self.plant_model = {**profile["model"], **self._plant_override}
        self.connected = self.deployed = self.configured = self.running = False
        self.info = {"simulation": True, "hardware": "simulation", "mokuos_version": None,
                     "bitstream": {"build_id": "python-fixed-reference-v2"},
                     "telemetry_kind": "simulated_full_rate"}
        self._refresh_info()
        self._tick = 0
        self._voltage = 0.0
        self._y = 0.0
        self._faults = 0
        self._latest = {"time_s": 0.0, "measurement_nm": 0.0, "reference_nm": 0.0,
                        "drive_v": 0.0, "residual_nm": 0.0, "state": "connected",
                        "faults": 0, "coherent": True, "sample_kind": "simulation"}

    def _refresh_info(self):
        self.info["effective_plant"] = copy.deepcopy(self.plant_model)
        self.info["simulation_options"] = copy.deepcopy(self.profile["simulation_options"])
        self.info["disturbance"] = copy.deepcopy(self.profile["disturbance"])

    def update_profile(self, profile: dict):
        if self.running:
            raise DeviceError("Stop before updating the profile")
        updated = normalize_profile(profile)
        errors = validate_profile(updated)
        if errors:
            raise ConfigurationError("; ".join(errors))
        if not updated["simulation"]:
            raise DeviceError("A simulated session cannot switch to physical hardware")
        if self.connected and any(updated[key] != self.profile[key]
                                   for key in ("device", "inputs", "fpga")):
            raise DeviceError("Disconnect before changing device, inputs or bitstream")
        self.profile = updated
        self.plant_model = {**updated["model"], **self._plant_override}
        self.configured = False
        self._refresh_info()

    def connect(self) -> dict:
        self.connected = True
        return copy.deepcopy(self.info)

    def diagnose(self) -> dict:
        return {"device": self.info, "output_blockers": validate_profile(self.profile, for_output=True),
                "note": "Offline simulation; does not validate any physical hardware"}

    def deploy(self, measurement_only: bool = True) -> dict:
        if not self.connected:
            raise DeviceError("Disconnected")
        if self.running:
            raise DeviceError("Stop before deploying")
        self.deployed = True
        self.configured = False
        return {**self.info, "measurement_only": measurement_only}

    def validate_measurement(self) -> dict:
        return {"simulation": True, "validation_accepted": False}

    def configure(self) -> dict:
        if not self.connected or not self.deployed or self.running:
            raise DeviceError("Connect and deploy, then stop before configuring")
        effective = copy.deepcopy(self.profile)
        effective["model"] = self.plant_model
        errors = validate_profile(effective, for_output=True)
        if errors:
            raise ConfigurationError("Invalid simulated plant: " + "; ".join(errors))
        self.words = profile_words(self.profile)
        self.configured = True
        return {"commit_id": 1, "words": self.words}

    def start(self, mode: str = "pid"):
        if not self.configured or self.running or mode not in ("pid", "preview", "open_loop"):
            raise DeviceError("Configured idle device and a valid mode required")
        self._controller = FixedController(self.words)
        self._mode = mode
        self._tick = 0
        self._faults = 0
        self._start_time = self.clock()
        self._y = self._voltage = 0.0
        self._hysteresis = self._previous_effective_v = 0.0
        self._rng = random.Random(self.profile["simulation_options"]["noise_seed"])
        rate = self.profile["controller"]["rate_hz"]
        self._delay = deque([0.0] * (round(self.plant_model["plant_delay_s"] * rate) + 1))
        self._sensor_delay = deque([0.0] * round(self.plant_model["sensor_delay_s"] * rate))
        self.running = True

    def _reference(self, tick):
        return board_reference(self.words, tick)[0]

    def read_telemetry(self) -> dict:
        if not self.connected:
            raise DeviceError("Disconnected")
        samples = []
        if self.running:
            p = self.profile
            rate = p["controller"]["rate_hz"]
            target = min(int((self.clock() - self._start_time) * rate), self.words[1])
            a = math.exp(-1 / (rate * self.plant_model["tau_s"]))
            b = self.plant_model["gain_nm_per_v"] * (1 - a)
            simulation = p["simulation_options"]
            while self._tick < target:
                drive = self._delay.popleft()
                effective = math.copysign(max(0.0, abs(drive) - simulation["dead_band_v"]), drive)
                self._y = a * self._y + b * effective
                if simulation["hysteresis_enabled"]:
                    change = effective - self._previous_effective_v
                    self._hysteresis += (simulation["hysteresis_a"] * change
                        - simulation["hysteresis_beta"] * abs(change) * self._hysteresis
                        - simulation["hysteresis_gamma"] * change * abs(self._hysteresis))
                self._previous_effective_v = effective
                true_position = self._y - self._hysteresis * simulation["hysteresis_nm_per_v"]
                measurement = true_position
                if self._sensor_delay:
                    measurement = self._sensor_delay.popleft()
                    self._sensor_delay.append(true_position)
                measurement += self._rng.gauss(0.0, simulation["measurement_noise_nm"])
                reference = self._reference(self._tick)
                future = (self._reference(self._tick + self.words[19]),
                          self._reference(self._tick + self.words[19] + 1))
                disturbance_v = 0.0
                try:
                    disturbance_v = board_disturbance(self.words, self._tick)
                    voltage = self._controller.update(self._mode, reference, measurement, future,
                                                      disturbance_v=disturbance_v)
                except (OverflowError, ValueError):
                    self._faults |= 32
                    voltage = p["drive"]["safe_v"]
                    self.running = False
                if abs(reference - measurement) > p["drive"]["max_abs_error_nm"]:
                    self._faults |= 8
                    voltage = p["drive"]["safe_v"]
                    self.running = False
                self._voltage = voltage
                self._delay.append(voltage)
                self._tick += 1
                self._latest = {"time_s": (self._tick - 1) / rate, "tick": self._tick,
                                "measurement_nm": measurement, "reference_nm": reference,
                                "drive_v": voltage, "residual_nm": self._controller.res1 / 65536,
                                "disturbance_v": disturbance_v,
                                "state": self._mode if self.running else "fault",
                                "faults": self._faults, "coherent": True, "sample_kind": "simulation"}
                samples.append(dict(self._latest))
                if not self.running:
                    break
            if self._tick >= self.words[1]:
                self.running = False
                self._latest["state"] = "finished"
                self._voltage = p["drive"]["safe_v"]
        return {**self._latest, "samples": samples, "running": self.running}

    def stop(self):
        self.running = False
        self._voltage = self.profile["drive"]["safe_v"] or 0.0
        self._latest.update(state="stopped", drive_v=self._voltage)

    def close(self):
        self.stop()
        self.connected = self.deployed = self.configured = False
