"""Exercise visible parameters through the real simulator and saved experiment."""

import csv
import json
import os
from pathlib import Path
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6 import QtCore, QtWidgets

from laserai.gui import MainWindow


def test_visible_idle_edits_reach_simulator_and_recording(monkeypatch, tmp_path):
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path))
    window = MainWindow(simulation=True)

    def wait(predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            application.processEvents()
            if predicate():
                return
            time.sleep(.005)
        raise AssertionError(window.diagnostics.toPlainText())

    try:
        window.show()
        window.connect_button.click()
        wait(lambda: window.connected and not window.busy)
        window.measurement_only.setChecked(False)
        window.deploy_button.click()
        wait(lambda: window.controller_deployed and not window.busy)
        window.configure_button.click()
        wait(lambda: window.configured and not window.busy)

        updates = {
            ("trajectory", "amplitude_nm"): 100.0,
            ("trajectory", "offset_nm"): 800.0,
            ("trajectory", "phase_deg"): 90.0,
            ("model", "gain_nm_per_v"): 500.0,
            ("disturbance", "enabled"): True,
            ("disturbance", "amplitude_v"): .1,
            ("disturbance", "start_s"): .05,
            ("disturbance", "duration_s"): .04,
            ("experiment", "duration_s"): .18,
            ("experiment", "settle_s"): .04,
        }
        for key, value in updates.items():
            window.parameters.fields[key].set_value(value)
        window.parameters.changed.emit()
        assert not window.configured
        assert not window.start_button.isEnabled()
        assert window.configure_button.isEnabled()
        window.configure_button.click()
        wait(lambda: window.configured and not window.busy)
        window.mode.setCurrentIndex(window.mode.findData("pid"))
        window.start_button.click()
        wait(lambda: window.running)
        assert not window.parameters.fields[("trajectory", "amplitude_nm")].isEnabled()
        wait(lambda: "series_finished" in window.diagnostics.toPlainText() and not window.running)

        snapshots = list((tmp_path / ".laserai" / "experiments").glob("*/snapshot.json"))
        assert len(snapshots) == 1
        snapshot = json.loads(snapshots[0].read_text(encoding="utf-8"))
        for (section, key), value in updates.items():
            assert snapshot["profile"][section][key] == value
        assert snapshot["device"]["effective_plant"]["gain_nm_per_v"] == 500.0
        with snapshots[0].with_name("telemetry.csv").open(encoding="utf-8", newline="") as stream:
            samples = list(csv.DictReader(stream))
        assert len(samples) == 180
        assert snapshot["mode"] == "pid"
        assert float(samples[0]["reference_nm"]) == 900.0
        for index in (20, 60, 100):
            sample = samples[index]
            injected = float(sample["disturbance_v"])
            assert abs(injected - (.1 if index == 60 else 0)) < .00002
        integral, previous_v = 0.0, 0.0
        for sample in samples:
            error = float(sample["reference_nm"]) - float(sample["measurement_nm"])
            integral = max(-5, min(5, integral + error * .2 * .001))
            requested = max(0, min(5, .002 * error + integral + float(sample["disturbance_v"])))
            expected_v = previous_v + max(-1, min(1, requested - previous_v))
            assert abs(float(sample["drive_v"]) - expected_v) < .002
            previous_v = expected_v
        result = json.loads(snapshots[0].with_name("result.json").read_text(encoding="utf-8"))
        assert result["reason"] == "completed"
        assert result["metrics"]["continuous"]
        assert window.parameters.fields[("trajectory", "amplitude_nm")].isEnabled()
    finally:
        window.close()
        wait(lambda: window.closed)
        assert window.worker_thread.wait(1000)
        window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
        application.processEvents()
