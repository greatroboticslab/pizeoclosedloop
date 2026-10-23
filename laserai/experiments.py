"""Durable experiment snapshots and metrics; no device or UI imports."""

from __future__ import annotations

import copy
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid

import numpy as np

from .config import _atomic_json, settings_dir

COLUMNS = ("time_s", "tick", "measurement_nm", "reference_nm", "drive_v", "residual_nm", "disturbance_v",
           "state", "faults", "coherent", "sample_kind")


def calculate_metrics(samples: list[dict], profile: dict) -> dict:
    stable = [s for s in samples if profile["experiment"]["settle_s"] <= float(s["time_s"])
              < profile["experiment"]["duration_s"]]
    result = {"samples": len(stable), "simulation": bool(profile["simulation"]),
              "eligible_for_acceptance": False, "hardware_target_verified": False}
    if len(stable) < 4:
        return {**result, "reason": "Insufficient steady-state samples"}
    t, y, r, u = (np.asarray([float(s[k]) for s in stable]) for k in
                   ("time_s", "measurement_nm", "reference_nm", "drive_v"))
    if not all(np.all(np.isfinite(v)) for v in (t, y, r, u)):
        return {**result, "reason": "Non-finite samples"}
    dt = 1 / profile["controller"]["rate_hz"]
    coherent = all(s.get("coherent") is True or s.get("coherent") == "True" for s in stable)
    continuous = bool(np.all(np.abs(np.diff(t) - dt) < dt * .01))
    complete = bool(t[-1] >= profile["experiment"]["duration_s"] - dt * 1.5)
    # Bit 4 is the autonomous duration event, not a protection failure.
    faulted = any(int(s.get("faults", 0)) & ~16 for s in stable)
    error = r - y
    d = profile["drive"]
    result.update(rmse_nm=float(np.sqrt(np.mean(error**2))),
                  mean_error_nm=float(np.mean(error)), peak_error_nm=float(np.max(np.abs(error))),
                  saturation_ratio=float(np.mean((u <= d["minimum_v"] + 1e-4) |
                                                  (u >= d["maximum_v"] - 1e-4))),
                  continuous=continuous, coherent=coherent, complete=complete, faulted=faulted)
    if profile["trajectory"]["waveform"] == "sine":
        omega = 2 * math.pi * profile["trajectory"]["frequency_hz"]
        basis = np.column_stack((np.sin(omega * t), np.cos(omega * t), np.ones_like(t)))
        rc, yc = np.linalg.lstsq(basis, r, rcond=None)[0], np.linalg.lstsq(basis, y, rcond=None)[0]
        ra, ya = float(np.hypot(*rc[:2])), float(np.hypot(*yc[:2]))
        lag = math.degrees(math.atan2(rc[1], rc[0]) - math.atan2(yc[1], yc[0]))
        result.update(reference_amplitude_nm=ra, measured_amplitude_nm=ya,
                      amplitude_ratio=ya / ra if ra > 1e-9 else None,
                      phase_lag_deg=(lag + 180) % 360 - 180)
    eligible = coherent and continuous and complete and not faulted
    result["eligible_for_acceptance"] = eligible
    result["reason"] = ("Full-rate simulation; physical hardware unverified" if eligible and profile["simulation"] else
                        "Full-rate calibrated recording" if eligible else
                        "Incomplete, faulted or decimated data; display metrics only")
    result["simulation_target_met"] = bool(profile["simulation"] and eligible
        and profile["trajectory"]["frequency_hz"] >= 20
        and profile["trajectory"]["amplitude_nm"] >= 500
        and result["rmse_nm"] <= profile["experiment"]["target_rmse_nm"])
    result["hardware_target_verified"] = bool(not profile["simulation"] and eligible
        and profile["trajectory"]["frequency_hz"] >= 20
        and profile["trajectory"]["amplitude_nm"] >= 500
        and result["rmse_nm"] <= profile["experiment"]["target_rmse_nm"])
    return result


class ExperimentManager:
    def __init__(self, root: str | Path | None = None):
        self.root = Path(root).expanduser() if root else settings_dir() / "experiments"
        self.active = False
        self._error = None

    def begin(self, profile: dict, device_info: dict, mode: str) -> Path:
        if self.active:
            raise RuntimeError("An experiment is already running")
        self.profile = copy.deepcopy(profile)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        self.path = self.root / f"{stamp}-{mode}-{uuid.uuid4().hex[:8]}"
        self.path.mkdir(parents=True, exist_ok=False)
        self.snapshot = {"schema_version": 1, "created_utc": stamp, "mode": mode,
                         "profile": self.profile, "device": copy.deepcopy(device_info)}
        _atomic_json(self.path / "snapshot.json", self.snapshot)
        self._queue = queue.Queue(maxsize=256)
        self._error = None
        self._thread = threading.Thread(target=self._write, name="experiment-writer", daemon=False)
        self.active = True
        self._thread.start()
        return self.path

    def _write(self):
        try:
            with (self.path / "telemetry.csv").open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=COLUMNS, extrasaction="ignore")
                writer.writeheader()
                while True:
                    batch = self._queue.get()
                    if batch is None:
                        break
                    writer.writerows(batch)
                    stream.flush()
        except Exception as exc:
            self._error = exc

    def append(self, telemetry: dict):
        if not self.active:
            raise RuntimeError("No experiment is running")
        if self._error:
            raise RuntimeError(f"Experiment writer failed: {self._error}") from self._error
        batch = telemetry.get("samples")
        if batch is None:
            batch = [telemetry] if "measurement_nm" in telemetry else []
        if batch:
            # Copy before crossing threads; never silently lose acquisition data.
            try:
                self._queue.put_nowait(copy.deepcopy(batch))
            except queue.Full as exc:
                raise RuntimeError("Experiment writer backlog exceeded; experiment aborted") from exc

    def finish(self, reason: str = "stopped") -> dict:
        if not self.active:
            return {}
        self.active = False
        while self._thread.is_alive():
            try:
                self._queue.put(None, timeout=.1)
                break
            except queue.Full:
                continue
        self._thread.join()
        if self._error:
            _atomic_json(self.path / "result.json", {"reason": "writer_failed", "error": str(self._error)})
            raise RuntimeError(f"Experiment writer failed: {self._error}") from self._error
        with (self.path / "telemetry.csv").open(encoding="utf-8", newline="") as stream:
            samples = list(csv.DictReader(stream))
        metrics = calculate_metrics(samples, self.profile)
        if reason not in ("completed", "finished"):
            metrics.update(eligible_for_acceptance=False, hardware_target_verified=False,
                           simulation_target_met=False, reason=f"Experiment ended: {reason}")
        result = {"path": str(self.path), "reason": reason, "metrics": metrics}
        _atomic_json(self.path / "result.json", result)
        return result


def run_ab(profile: dict, device_factory, root: str | Path | None = None, *,
           sleep=time.sleep, poll_s=.02, cancelled=lambda: False) -> dict:
    """Run equal-condition repeats; alternate ordering to expose drift bias."""
    manager = ExperimentManager(root)
    results = []
    device = device_factory(copy.deepcopy(profile))
    try:
        info = device.connect()
        info.update(device.deploy(measurement_only=False))
        for repeat in range(profile["experiment"]["repeats"]):
            for mode in (("pid", "preview") if repeat % 2 == 0 else ("preview", "pid")):
                if cancelled():
                    raise RuntimeError("A/B experiment cancelled")
                device.configure()
                manager.begin(profile, info, mode)
                try:
                    device.start(mode)
                    while True:
                        sleep(poll_s)
                        telemetry = device.read_telemetry()
                        manager.append(telemetry)
                        if int(telemetry.get("faults", 0)) & ~16:
                            raise RuntimeError(f"Device fault: {telemetry['faults']}")
                        if cancelled():
                            raise RuntimeError("A/B experiment cancelled")
                        if not device.running:
                            break
                    result = manager.finish("completed")
                    results.append({"repeat": repeat + 1, "mode": mode, **result})
                except Exception:
                    manager.finish("failed")
                    raise
                finally:
                    device.stop()
        aggregate = {}
        for mode in ("pid", "preview"):
            selected = [r["metrics"] for r in results if r["mode"] == mode]
            aggregate[mode] = {}
            for key in ("rmse_nm", "amplitude_ratio", "phase_lag_deg", "saturation_ratio"):
                values = [m[key] for m in selected if m.get(key) is not None]
                if values:
                    aggregate[mode][key] = {"mean": float(np.mean(values)), "std": float(np.std(values)),
                                            "min": min(values), "max": max(values)}
        summary = {"simulation": profile["simulation"], "runs": results, "aggregate": aggregate,
                   "hardware_target_verified": False}
        target = manager.root / f"ab-{uuid.uuid4().hex[:8]}.json"
        _atomic_json(target, summary)
        return {"summary_path": str(target), **summary}
    finally:
        if manager.active:
            manager.finish("aborted")
        device.close()
