"""Headless Qt integration checks, including thread ownership and fault gates."""

import os
import json
from pathlib import Path
import threading
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PySide6")
from PySide6 import QtCore, QtWidgets

from laserai import gui


@pytest.fixture
def application():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def wait_for(application, predicate, timeout=3):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        application.processEvents()
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("Qt operation timed out")


def dispose(application, widget):
    widget.close()
    wait_for(application, lambda: widget.closed)
    assert widget.worker_thread.wait(1000)
    widget.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    application.processEvents()


class FakeDevice:
    instances = []

    def __init__(self, profile):
        self.profile = profile
        self.calls = []
        self.fail_read = False
        self.counter = 0
        self.finish_after_reads = None
        self.modes = []
        self.completion_faults = 0
        self.ramp_first = False
        self.instances.append(self)

    def call(self, name):
        self.calls.append((name, threading.get_ident()))

    def connect(self):
        self.call("connect")
        return {"serial": "SIMULATION"}

    def deploy(self, measurement_only=True):
        self.call("deploy")
        return {"measurement_only": measurement_only}

    def configure(self):
        self.call("configure")
        return {"ack": 1}

    def update_profile(self, profile):
        self.call("update_profile")
        self.profile = profile

    def start(self, mode="pid"):
        self.call("start")
        self.modes.append(mode)
        self.counter = 0

    def stop(self):
        self.call("stop")

    def close(self):
        self.call("close")

    def read_telemetry(self):
        self.call("read")
        if self.fail_read:
            raise RuntimeError("Simulated disconnect")
        self.counter += 1
        finished = self.finish_after_reads is not None and self.counter >= self.finish_after_reads
        state = "finished" if finished else "ready"
        if self.ramp_first:
            state = 0 if finished else 3
        return {"time_s": self.counter / 10, "measurement_nm": 10,
                "reference_nm": 12, "residual_nm": 2, "drive_v": 0.1,
                "state": state, "faults": self.completion_faults,
                "running": not finished, "coherent": True}


class FakeExperiments:
    def __init__(self, root=None):
        self.samples = []

    def begin(self, profile, info, mode):
        return Path.home() / "\u5b9e\u9a8c \u7ed3\u679c" / mode

    def append(self, sample):
        self.samples.append(sample)

    def finish(self, reason="stopped"):
        return {"reason": reason}


@pytest.fixture
def window(application, monkeypatch, tmp_path):
    monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path))
    monkeypatch.setattr(gui, "SimulatedDevice", FakeDevice)
    monkeypatch.setattr(gui, "ExperimentManager", FakeExperiments)
    widget = gui.MainWindow(simulation=True)
    widget.show()
    yield widget
    dispose(application, widget)


def prepare(application, window):
    window.connect_button.click()
    wait_for(application, lambda: window.connected)
    assert not window.start_button.isEnabled()
    window.measurement_only.setChecked(False)
    window.deploy_button.click()
    wait_for(application, lambda: window.controller_deployed and not window.busy)
    window.configure_button.click()
    wait_for(application, lambda: window.configured)


def test_gui_device_io_uses_worker_and_shutdown_closes_device(application, window):
    main_thread = threading.get_ident()
    prepare(application, window)
    window.start_button.click()
    wait_for(application, lambda: window.running and len(window.times) > 0)
    assert "nm" in window.values.text()
    assert "\u5b9e\u9a8c \u7ed3\u679c" in window.experiment_path_label.text()
    window.stop_button.click()
    wait_for(application, lambda: not window.running and not window.busy)
    device = FakeDevice.instances[-1]
    window.close()
    wait_for(application, lambda: window.closed)
    assert any(name == "close" for name, _ in device.calls)
    assert all(thread != main_thread for _, thread in device.calls)


def test_telemetry_failure_stops_device_and_locks_start(application, window):
    prepare(application, window)
    FakeDevice.instances[-1].fail_read = True
    wait_for(application, lambda: window.output_fault)
    assert not window.start_button.isEnabled()
    assert not window.configured
    assert "Simulated disconnect" in window.diagnostics.toPlainText()
    assert any(name == "stop" for name, _ in FakeDevice.instances[-1].calls)


@pytest.mark.parametrize("mode", ["pid", "preview"])
def test_only_requested_controllers_are_available_and_finish_once(application, window, mode):
    assert [window.mode.itemData(i) for i in range(window.mode.count())] == ["pid", "preview"]
    prepare(application, window)
    device = FakeDevice.instances[-1]
    device.finish_after_reads = 1
    device.completion_faults = 16
    window.mode.setCurrentIndex(window.mode.findData(mode))
    window.start_button.click()
    wait_for(application, lambda: "series_finished" in window.diagnostics.toPlainText()
             and not window.running and not window.busy)
    assert device.modes == [mode]
    assert sum(name == "configure" for name, _ in device.calls) == 2
    assert window.start_button.isEnabled()
    assert "series_finished" in window.diagnostics.toPlainText()
    assert not window.output_fault


def test_duration_ramp_waits_for_idle_and_error_is_reference_minus_measurement(application, window):
    prepare(application, window)
    device = FakeDevice.instances[-1]
    device.finish_after_reads = 2
    device.completion_faults = 16
    device.ramp_first = True
    window.start_button.click()
    wait_for(application, lambda: window.run_state.text() == "\u5b89\u5168\u56de\u843d")
    assert window.running
    assert not window.output_fault
    wait_for(application, lambda: not window.running and "series_finished" in window.diagnostics.toPlainText())
    window._telemetry({"time_s": 1, "measurement_nm": 10, "reference_nm": 12,
                       "residual_nm": 999, "drive_v": 0.1, "faults": 0})
    assert "\u8bef\u5dee: 2.00 nm" in window.values.text()


def test_new_experiment_restores_plot_range_after_manual_zoom(application, window):
    window.plot.setYRange(0, 10, padding=0)
    window.plot.setXRange(100, 110, padding=0)
    window._completed("start", {"experiment_path": "test"})
    for k in range(30):
        window._telemetry({"time_s": k / 1000, "measurement_nm": 600 + 5*k,
                           "reference_nm": 900 - 3*k, "drive_v": 1, "faults": 0})
    application.processEvents()
    x_range, y_range = window.plot.viewRange()
    assert y_range[0] <= 600 and y_range[1] >= 900
    assert x_range[0] <= 0 and x_range[1] >= .029
    assert len(window.measurement_curve.getData()[0]) == 30


def test_fit_plot_recovers_hidden_curves_and_continues_following_data(application, window):
    for k in range(20):
        window._telemetry({"time_s": k/10, "measurement_nm": 650+k,
                           "reference_nm": 900, "drive_v": 1, "faults": 0})
    window.auto_plot_range.setChecked(False)
    window.plot.setYRange(0, 10, padding=0)
    window.fit_plot_button.click()
    application.processEvents()
    assert window.auto_plot_range.isChecked()
    assert all(window.plot.getViewBox().autoRangeEnabled())
    assert window.plot.viewRange()[1][1] >= 900
    window._telemetry({"time_s": 2, "measurement_nm": 1600,
                       "reference_nm": 1700, "drive_v": 1, "faults": 0})
    application.processEvents()
    assert window.plot.viewRange()[1][1] >= 1700


def test_manual_plot_inspection_is_preserved_until_auto_range_is_restored(application, window):
    window.plot.setYRange(0, 10, padding=0)
    window.plot.getViewBox().sigRangeChangedManually.emit([False, True])
    assert not window.auto_plot_range.isChecked()
    window._telemetry({"time_s": 1, "measurement_nm": 650,
                       "reference_nm": 900, "drive_v": 1, "faults": 0})
    application.processEvents()
    assert window.plot.viewRange()[1] == [0, 10]
    window.auto_plot_range.setChecked(True)
    application.processEvents()
    assert window.plot.viewRange()[1][1] >= 900


def test_corrupt_profile_blocks_connection_without_fallback(application, monkeypatch, tmp_path):
    monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path))
    path = tmp_path / "\u914d\u7f6e \u6d4b\u8bd5.json"
    path.write_text('{"schema_version":', encoding="utf-8")
    widget = gui.MainWindow(profile_path=path)
    try:
        assert widget.profile_error
        assert not widget.connect_button.isEnabled()
        assert widget.editor.toPlainText() == path.read_text(encoding="utf-8")
        assert widget.tabs.currentIndex() == 2
    finally:
        dispose(application, widget)


def test_idle_parameter_edits_reach_device_and_invalidate_ack(application, window):
    prepare(application, window)
    assert not window.parameters.fields[("inputs", "carrier_hz")].isEnabled()
    field = window.parameters.fields[("controller", "kp")]
    assert field.isEnabled()
    field.set_value(0.004)
    window.parameters.changed.emit()
    assert not window.configured
    assert not window.start_button.isEnabled()
    assert json.loads(window.editor.toPlainText())["controller"]["kp"] == 0.004
    window.configure_button.click()
    wait_for(application, lambda: window.configured)
    assert FakeDevice.instances[-1].profile["controller"]["kp"] == 0.004
    window.start_button.click()
    wait_for(application, lambda: window.running)
    assert not field.isEnabled()
    assert window.editor.isReadOnly()


def test_invalid_forms_can_be_repaired_without_driving_old_values(application, window):
    prepare(application, window)
    wave = window.parameters.fields[("trajectory", "waveform")]
    wave.input.setCurrentIndex(wave.input.findData("square"))
    duty = window.parameters.fields[("trajectory", "duty_pct")]
    duty.set_value(0)
    window.parameters.changed.emit()
    assert window.profile_error
    assert duty.isEnabled()
    assert not window.configure_button.isEnabled()
    duty.set_value(50)
    window.parameters.changed.emit()
    assert not window.profile_error
    kp = window.parameters.fields[("controller", "kp")]
    kp.input.setText("-")
    window.parameters.changed.emit()
    assert window.form_error
    window._configure()
    assert not window.configured
    assert kp.input.text() == "-"
    kp.set_value(0.003)
    window.parameters.changed.emit()
    assert not window.form_error
    assert window.configure_button.isEnabled()


def test_malformed_advanced_json_is_preserved_and_blocks_output(application, window):
    prepare(application, window)
    text = '{"trajectory": '
    window.editor.setPlainText(text)
    assert window.editor.toPlainText() == text
    assert window.profile_error
    assert not window.configure_button.isEnabled()
    assert not window.start_button.isEnabled()
    assert not window.parameters.fields[("controller", "kp")].isEnabled()
