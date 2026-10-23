"""Qt supervisor; all device I/O is serialized on one worker thread."""

from __future__ import annotations

import copy
import json
from collections import deque
from pathlib import Path
import sys

from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from .config import (default_profile, load_local_settings, load_profile,
                     normalize_profile, save_local_settings, save_profile, simulation_profile, validate_profile)
from .device import MokuDevice, SimulatedDevice
from .experiments import ExperimentManager
from .protocol import Fault
from .parameters import LOCKED_SECTIONS, ParameterPanel


class DeviceWorker(QtCore.QObject):
    completed = QtCore.Signal(str, object)
    failed = QtCore.Signal(str, str)
    telemetry = QtCore.Signal(object)
    shutdown_complete = QtCore.Signal()

    def __init__(self):
        super().__init__()
        self.device = None
        self.info = {}
        self.experiments = None
        self.active_experiment = False
        self.timer = None
        self.sequence = deque()
        self.experiment_profile = None
        self.series_results = []

    @QtCore.Slot()
    def initialize(self):
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.poll)

    def _finish(self, reason):
        if self.active_experiment:
            self.active_experiment = False
            return self.experiments.finish(reason=reason)
        return {}

    def _close_device(self):
        self.sequence.clear()
        if self.timer:
            self.timer.stop()
        if self.device is not None:
            try:
                self.device.stop()
            finally:
                try:
                    self._finish("disconnected")
                finally:
                    self.device.close()
                    self.device = None

    def _begin(self, profile, mode):
        self._update_profile(profile)
        self.device.configure()
        path = self.experiments.begin(profile, self.info, mode)
        self.active_experiment = True
        try:
            self.device.start(mode=mode)
        except Exception:
            self._finish("start_failed")
            raise
        return {"experiment_path": str(path), "mode": mode}

    def _update_profile(self, profile):
        self.device.update_profile(profile)
        self.info = copy.deepcopy(getattr(self.device, "info", self.info))

    @QtCore.Slot(str, object)
    def execute(self, command, payload):
        try:
            if command == "connect":
                self._close_device()
                profile = payload["profile"]
                self.device = (SimulatedDevice(profile) if payload["simulation"] else
                               MokuDevice(profile, cli_path=payload.get("cli_path")))
                self.info = self.device.connect()
                self.experiments = ExperimentManager(root=payload.get("experiment_root"))
                result = self.info
            elif command == "disconnect":
                self._close_device()
                result = {"state": "disconnected"}
            elif command == "shutdown":
                self._close_device()
                self.shutdown_complete.emit()
                return
            else:
                if self.device is None:
                    raise RuntimeError("\u8bf7\u5148\u8fde\u63a5\u8bbe\u5907")
                if command == "diagnose":
                    result = self.device.diagnose()
                elif command == "measurement":
                    result = self.device.validate_measurement()
                elif command == "deploy":
                    self._update_profile(payload["profile"])
                    result = self.device.deploy(measurement_only=payload["measurement_only"])
                    self.info = copy.deepcopy(getattr(self.device, "info", self.info))
                    self.timer.start()
                elif command == "configure":
                    self._update_profile(payload["profile"])
                    result = self.device.configure()
                    self.timer.start()
                elif command == "update_profile":
                    self._update_profile(payload["profile"])
                    result = {"configured": False}
                elif command == "start":
                    if self.active_experiment:
                        raise RuntimeError("\u5b9e\u9a8c\u5df2\u5728\u8fd0\u884c")
                    if payload["mode"] not in ("pid", "preview"):
                        raise ValueError("请选择 PID 或 Preview + ADRC")
                    self.experiment_profile = payload["profile"]
                    self.series_results = []
                    self.sequence = deque([payload["mode"]])
                    result = self._begin(self.experiment_profile, self.sequence.popleft())
                elif command == "stop":
                    self.sequence.clear()
                    self.device.stop()
                    result = self._finish("stopped")
                else:
                    raise ValueError(f"Unknown command: {command}")
            self.completed.emit(command, result)
        except Exception as exc:
            if command == "shutdown":
                self.failed.emit(command, str(exc))
                self.shutdown_complete.emit()
                return
            if self.timer:
                self.timer.stop()
            self.sequence.clear()
            if self.device:
                try:
                    self.device.stop()
                except Exception:
                    pass
            try:
                self._finish(f"{command}_failed")
            except Exception:
                pass
            self.failed.emit(command, str(exc))

    @QtCore.Slot()
    def poll(self):
        if self.device is None:
            return
        try:
            sample = self.device.read_telemetry()
            if sample:
                if self.active_experiment:
                    self.experiments.append(sample)
                self.telemetry.emit(sample)
                finished = (sample.get("running") is False or sample.get("state") in (0, "finished", "fault"))
                if self.active_experiment and finished:
                    self.device.stop()
                    safety_faults = int(sample.get("faults") or 0) & ~int(Fault.DURATION)
                    faulted = bool(safety_faults) or sample.get("state") == "fault"
                    result = self._finish("fault" if faulted else "completed")
                    self.series_results.append(result)
                    self.completed.emit("finished", result)
                    if faulted:
                        self.sequence.clear()
                    elif self.sequence:
                        result = self._begin(self.experiment_profile, self.sequence.popleft())
                        self.completed.emit("start", result)
                    else:
                        self.completed.emit("series_finished", self.series_results)
        except Exception as exc:
            self.timer.stop()
            self.sequence.clear()
            try:
                self.device.stop()
            except Exception:
                pass
            try:
                self._finish("telemetry_failed")
            except Exception:
                pass
            self.failed.emit("telemetry", str(exc))


class MainWindow(QtWidgets.QMainWindow):
    request = QtCore.Signal(str, object)

    def __init__(self, profile_path=None, simulation=False):
        super().__init__()
        self.setWindowTitle("LaserAI | Moku:Go")
        self.resize(1120, 780)
        self.setMinimumSize(760, 600)
        self.explicit_profile = profile_path is not None
        self.profile_path = Path(profile_path or Path.home() / ".laserai" / "hardware.json")
        self.profile = default_profile()
        self.connected = self.running = self.busy = self.closing = self.closed = False
        self.output_fault = False
        self.configured = False
        self.controller_deployed = False
        self.pending_measurement_only = True
        self.profile_error = None
        self.form_error = None
        self.json_invalid = False
        self.connected_profile = None
        self._syncing_profile = False
        self.times, self.measurements, self.references = (deque(maxlen=2000) for _ in range(3))
        local_error = None
        try:
            self.local = load_local_settings()
        except Exception as exc:
            self.local = {}
            local_error = str(exc)
        self._build_ui()
        self._load_initial_profile(simulation)
        if local_error:
            self.diagnostics.appendPlainText(f"local settings: {local_error}")
        geometry = self.local.get("window_geometry")
        if geometry:
            self.restoreGeometry(QtCore.QByteArray.fromBase64(geometry.encode("ascii")))
        self.worker_thread = QtCore.QThread(self)
        self.worker = DeviceWorker()
        self.worker.moveToThread(self.worker_thread)
        self.worker_thread.started.connect(self.worker.initialize)
        self.request.connect(self.worker.execute)
        self.worker.completed.connect(self._completed)
        self.worker.failed.connect(self._failed)
        self.worker.telemetry.connect(self._telemetry)
        self.worker.shutdown_complete.connect(self.worker_thread.quit)
        self.worker_thread.finished.connect(self.worker.deleteLater)
        self.worker_thread.finished.connect(self._shutdown_finished)
        self.worker_thread.start()
        self._update_controls()

    def _button(self, text, callback, icon=None):
        button = QtWidgets.QPushButton(text)
        if icon:
            button.setIcon(self.style().standardIcon(icon))
        button.clicked.connect(callback)
        return button

    def _build_ui(self):
        body = QtWidgets.QWidget()
        self.setCentralWidget(body)
        layout = QtWidgets.QVBoxLayout(body)
        layout.setContentsMargins(14, 12, 14, 12)
        connection = QtWidgets.QHBoxLayout()
        self.simulation = QtWidgets.QCheckBox("\u4eff\u771f")
        self.simulation.toggled.connect(self._simulation_toggled)
        connection.addWidget(self.simulation)
        connection.addWidget(QtWidgets.QLabel("Moku IP"))
        self.address = QtWidgets.QLineEdit()
        self.address.setPlaceholderText("192.168.1.100")
        self.address.textEdited.connect(self._address_changed)
        connection.addWidget(self.address, 1)
        self.connect_button = self._button("\u8fde\u63a5", self._connect)
        self.disconnect_button = self._button("\u65ad\u5f00", lambda: self._submit("disconnect"))
        connection.addWidget(self.connect_button)
        connection.addWidget(self.disconnect_button)
        self.connection_state = QtWidgets.QLabel("\u672a\u8fde\u63a5")
        self.connection_state.setMinimumWidth(130)
        connection.addWidget(self.connection_state)
        layout.addLayout(connection)
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, 1)
        monitor = QtWidgets.QWidget()
        monitor_layout = QtWidgets.QVBoxLayout(monitor)
        tools = QtWidgets.QHBoxLayout()
        self.diagnose_button = self._button("\u8bbe\u5907\u8bca\u65ad", lambda: self._submit("diagnose"))
        self.measurement_button = self._button("\u6d4b\u91cf\u9a8c\u8bc1", lambda: self._submit("measurement"))
        self.measurement_only = QtWidgets.QCheckBox("\u4ec5\u6d4b\u91cf")
        self.measurement_only.setChecked(True)
        self.deploy_button = self._button("\u90e8\u7f72", self._deploy)
        self.configure_button = self._button("\u5199\u5165\u53c2\u6570", self._configure)
        for widget in (self.diagnose_button, self.measurement_button, self.measurement_only,
                       self.deploy_button, self.configure_button):
            tools.addWidget(widget)
        tools.addStretch()
        monitor_layout.addLayout(tools)
        run_tools = QtWidgets.QHBoxLayout()
        self.mode = QtWidgets.QComboBox()
        self.mode.addItem("PID", "pid")
        self.mode.addItem("Preview + ADRC", "preview")
        run_tools.addWidget(self.mode)
        self.start_button = self._button("\u5f00\u59cb\u5b9e\u9a8c", self._start,
                                         QtWidgets.QStyle.StandardPixmap.SP_MediaPlay)
        self.stop_button = self._button("\u505c\u6b62", lambda: self._submit("stop"),
                                        QtWidgets.QStyle.StandardPixmap.SP_MediaStop)
        run_tools.addWidget(self.start_button)
        run_tools.addWidget(self.stop_button)
        self.run_state = QtWidgets.QLabel("\u5c31\u7eea")
        run_tools.addWidget(self.run_state, 1)
        monitor_layout.addLayout(run_tools)
        pg.setConfigOptions(antialias=True)
        self.plot = pg.PlotWidget(background="w")
        self.plot.setLabel("left", "\u4f4d\u79fb", units="nm")
        self.plot.setLabel("bottom", "\u65f6\u95f4", units="s")
        self.plot.getAxis("left").enableAutoSIPrefix(False)
        self.plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.plot.showGrid(x=True, y=True, alpha=0.18)
        self.plot.addLegend()
        self.reference_curve = self.plot.plot(pen=pg.mkPen("#b35d23", width=2), name="\u76ee\u6807")
        self.measurement_curve = self.plot.plot(pen=pg.mkPen("#187566", width=2), name="\u6d4b\u91cf")
        plot_tools = QtWidgets.QHBoxLayout()
        plot_tools.addStretch()
        self.auto_plot_range = QtWidgets.QCheckBox("自动量程")
        self.auto_plot_range.setChecked(True)
        self.auto_plot_range.toggled.connect(self._set_plot_auto_range)
        plot_tools.addWidget(self.auto_plot_range)
        self.fit_plot_button = QtWidgets.QToolButton()
        self.fit_plot_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.StandardPixmap.SP_TitleBarMaxButton))
        self.fit_plot_button.setToolTip("适应数据并恢复自动量程")
        self.fit_plot_button.setAccessibleName("适应数据")
        self.fit_plot_button.clicked.connect(self._fit_plot)
        plot_tools.addWidget(self.fit_plot_button)
        monitor_layout.addLayout(plot_tools)
        self.plot.getViewBox().sigRangeChangedManually.connect(self._plot_range_changed_manually)
        monitor_layout.addWidget(self.plot, 1)
        self.values = QtWidgets.QLabel("\u4f4d\u79fb: -- nm    \u8bef\u5dee: -- nm    \u8f93\u51fa: -- V")
        monitor_layout.addWidget(self.values)
        self.experiment_path_label = QtWidgets.QLabel("\u5b9e\u9a8c: --")
        self.experiment_path_label.setWordWrap(True)
        self.experiment_path_label.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        monitor_layout.addWidget(self.experiment_path_label)
        self.tabs.addTab(monitor, "\u76d1\u63a7")
        self.parameters = ParameterPanel()
        self.parameters.changed.connect(self._parameters_changed)
        self.parameters.error.connect(lambda message: self._failed("config", message))
        self.tabs.addTab(self.parameters, "参数")
        config_tab = QtWidgets.QWidget()
        config_layout = QtWidgets.QVBoxLayout(config_tab)
        file_tools = QtWidgets.QHBoxLayout()
        self.import_button = self._button("\u5bfc\u5165", self._import_profile, QtWidgets.QStyle.StandardPixmap.SP_DialogOpenButton)
        self.export_button = self._button("\u5bfc\u51fa", self._export_profile, QtWidgets.QStyle.StandardPixmap.SP_DialogSaveButton)
        self.save_button = self._button("\u4fdd\u5b58", lambda: self._save_to(self.profile_path))
        self.demo_button = self._button("\u52a0\u8f7d\u4eff\u771f\u53c2\u6570", self._load_simulation)
        self.validate_button = self._button("\u68c0\u67e5\u914d\u7f6e", self._check_profile)
        for button in (self.import_button, self.export_button, self.save_button,
                       self.demo_button, self.validate_button):
            file_tools.addWidget(button)
        file_tools.addStretch()
        config_layout.addLayout(file_tools)
        self.profile_path_label = QtWidgets.QLabel(str(self.profile_path))
        self.profile_path_label.setWordWrap(True)
        config_layout.addWidget(self.profile_path_label)
        self.editor = QtWidgets.QPlainTextEdit()
        self.editor.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont))
        self.editor.textChanged.connect(self._editor_changed)
        config_layout.addWidget(self.editor, 1)
        cli_row = QtWidgets.QHBoxLayout()
        cli_row.addWidget(QtWidgets.QLabel("MokuCLI"))
        self.cli_path = QtWidgets.QLineEdit(str(self.local.get("cli_path") or ""))
        self.cli_path.setPlaceholderText("MOKU_CLI_PATH / SDK")
        cli_row.addWidget(self.cli_path, 1)
        config_layout.addLayout(cli_row)
        recording_row = QtWidgets.QHBoxLayout()
        recording_row.addWidget(QtWidgets.QLabel("记录目录"))
        self.recording_path = QtWidgets.QLineEdit(str(self.local.get("experiment_root") or ""))
        self.recording_path.setPlaceholderText(str(Path.home() / ".laserai" / "experiments"))
        recording_row.addWidget(self.recording_path, 1)
        self.choose_recording = self._button("", self._choose_recording_directory,
                                            QtWidgets.QStyle.StandardPixmap.SP_DirOpenIcon)
        self.choose_recording.setToolTip("选择记录目录")
        recording_row.addWidget(self.choose_recording)
        config_layout.addLayout(recording_row)
        self.config_state = QtWidgets.QLabel()
        self.config_state.setWordWrap(True)
        config_layout.addWidget(self.config_state)
        self.tabs.addTab(config_tab, "高级 JSON")
        self.diagnostics = QtWidgets.QPlainTextEdit()
        self.diagnostics.setReadOnly(True)
        self.diagnostics.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont))
        self.tabs.addTab(self.diagnostics, "\u8bca\u65ad\u4e0e\u65e5\u5fd7")
        self.statusBar().showMessage("\u672a\u8fde\u63a5")

    def _load_initial_profile(self, simulation):
        if simulation and not self.explicit_profile:
            self.profile = simulation_profile()
        elif self.explicit_profile or self.profile_path.exists():
            try:
                self.profile = load_profile(self.profile_path)
            except Exception as exc:
                self.profile_error = str(exc)
                try:
                    text = self.profile_path.read_text(encoding="utf-8", errors="replace")
                except OSError:
                    text = ""
                self.editor.setPlainText(text)
                self.config_state.setText(f"\u914d\u7f6e\u635f\u574f: {exc}")
                self.json_invalid = True
                self.tabs.setCurrentWidget(self.editor.parentWidget())
                return
        if simulation:
            self.profile["simulation"] = True
        self.editor.setPlainText(json.dumps(self.profile, indent=2, ensure_ascii=False))
        self.address.setText(str(self.profile.get("device", {}).get("address") or ""))
        with QtCore.QSignalBlocker(self.simulation):
            self.simulation.setChecked(bool(self.profile.get("simulation")))
        self.measurement_only.setChecked(not bool(self.profile.get("simulation")))
        self._check_profile()

    def _editor_changed(self):
        if not self._syncing_profile:
            self.form_error = None
            self.configured = False
            self._check_profile()

    def _parameters_changed(self):
        if self._syncing_profile:
            return
        self.configured = False
        self.form_error = None
        try:
            profile = self.parameters.profile()
            if not self.connected:
                self.address.setText(str(profile.get("device", {}).get("address") or ""))
                with QtCore.QSignalBlocker(self.simulation):
                    self.simulation.setChecked(bool(profile.get("simulation")))
            with QtCore.QSignalBlocker(self.editor):
                self.editor.setPlainText(json.dumps(profile, indent=2, ensure_ascii=False))
            self._check_profile(sync_parameters=False)
        except (ValueError, TypeError) as exc:
            self.form_error = str(exc)
            self.profile_error = str(exc)
            self.config_state.setText(str(exc))
            self._update_controls()

    def _address_changed(self, text):
        field = self.parameters.fields.get(("device", "address"))
        if field and not self.connected:
            field.set_value(text or None)
            self._parameters_changed()

    def _simulation_toggled(self, checked):
        if self.connected or self._syncing_profile:
            return
        if checked and not self.profile.get("simulation"):
            self._load_simulation()
        else:
            profile = self.parameters.profile()
            profile["simulation"] = checked
            self.editor.setPlainText(json.dumps(profile, indent=2, ensure_ascii=False))
        self.measurement_only.setChecked(not checked)
        self._update_controls()

    def _choose_recording_directory(self):
        directory = QtWidgets.QFileDialog.getExistingDirectory(self, "记录目录", self.recording_path.text())
        if directory:
            self.recording_path.setText(directory)

    def _read_editor(self):
        parsed = json.loads(self.editor.toPlainText())
        if not isinstance(parsed, dict):
            raise ValueError("\u914d\u7f6e\u6839\u8282\u70b9\u5fc5\u987b\u662f JSON \u5bf9\u8c61")
        return normalize_profile(parsed)

    def _check_profile(self, sync_parameters=True):
        if self.form_error:
            self.profile_error = self.form_error
            self.config_state.setText(self.form_error)
            self._update_controls()
            return False
        try:
            profile = self._read_editor()
            errors = validate_profile(profile, for_output=False)
            if self.connected and self.connected_profile:
                locked = [section for section in LOCKED_SECTIONS if section != "__root__"]
                if any(profile.get(section) != self.connected_profile.get(section) for section in locked) or \
                        profile.get("simulation") != self.connected_profile.get("simulation"):
                    errors.append("设备、输入、标定或部署设置已变化，请断开后重新连接")
            self.profile_error = "; ".join(errors) if errors else None
            if not errors:
                self.profile = profile
            self.json_invalid = bool(errors) if sync_parameters else False
            if not errors and sync_parameters:
                self._syncing_profile = True
                try:
                    self.parameters.set_profile(profile)
                finally:
                    self._syncing_profile = False
                with QtCore.QSignalBlocker(self.simulation):
                    self.simulation.setChecked(bool(profile.get("simulation")))
                if not self.connected:
                    self.address.setText(str(profile.get("device", {}).get("address") or ""))
            output_errors = validate_profile(profile, for_output=True)
            summary = "; ".join(output_errors[:3])
            if len(output_errors) > 3:
                summary += f" (+{len(output_errors) - 3})"
            message = "\u8f93\u51fa\u672a\u5c31\u7eea: " + summary if output_errors else "\u914d\u7f6e\u68c0\u67e5\u901a\u8fc7"
            self.config_state.setText(self.profile_error or message)
            self.config_state.setToolTip("\n".join(errors + output_errors))
        except Exception as exc:
            self.profile_error = str(exc)
            self.json_invalid = True
            self.config_state.setText(f"\u914d\u7f6e\u9519\u8bef: {exc}")
        self._update_controls()
        return self.profile_error is None

    def _load_simulation(self):
        if not self.connected:
            self.editor.setPlainText(json.dumps(simulation_profile(), indent=2, ensure_ascii=False))
            with QtCore.QSignalBlocker(self.simulation):
                self.simulation.setChecked(True)
            self.measurement_only.setChecked(False)
            self._check_profile()

    def _import_profile(self):
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(self, "\u5bfc\u5165\u786c\u4ef6\u914d\u7f6e", "", "JSON (*.json)")
        if filename:
            try:
                profile = load_profile(Path(filename))
                self.profile_path = Path(filename)
                self.profile_path_label.setText(str(self.profile_path))
                self.editor.setPlainText(json.dumps(profile, indent=2, ensure_ascii=False))
                self.address.setText(str(profile.get("device", {}).get("address") or ""))
                self._check_profile()
            except Exception as exc:
                self._failed("import", str(exc))

    def _save_to(self, path):
        if self._check_profile():
            try:
                save_profile(path, self.profile)
                self.profile_path = Path(path)
                self.profile_path_label.setText(str(path))
                self.statusBar().showMessage(f"\u5df2\u4fdd\u5b58: {path}")
            except Exception as exc:
                self._failed("save", str(exc))

    def _export_profile(self):
        filename, _ = QtWidgets.QFileDialog.getSaveFileName(self, "\u5bfc\u51fa\u786c\u4ef6\u914d\u7f6e", str(self.profile_path), "JSON (*.json)")
        if filename:
            self._save_to(Path(filename))

    def _submit(self, command, payload=None):
        if not self.closing:
            self.busy = True
            self._update_controls()
            self.statusBar().showMessage(f"{command}...")
            self.request.emit(command, payload or {})

    def _connect(self):
        address = self.address.text().strip()
        if self._check_profile():
            profile = copy.deepcopy(self.profile)
            profile.setdefault("device", {})["address"] = address or None
            if self.simulation.isChecked():
                profile["simulation"] = True
            self.editor.setPlainText(json.dumps(profile, indent=2, ensure_ascii=False))
            self.profile = profile
            self.profile_error = None
            self.connected_profile = copy.deepcopy(profile)
            self._submit("connect", {"profile": profile, "simulation": self.simulation.isChecked(),
                                     "cli_path": self.cli_path.text().strip() or None,
                                     "experiment_root": self.recording_path.text().strip() or None})

    def _deploy(self):
        self.pending_measurement_only = self.measurement_only.isChecked()
        self.configured = False
        if self._check_profile():
            self._submit("deploy", {"measurement_only": self.measurement_only.isChecked(), "profile": self.profile})

    def _output_ready(self):
        if not self._check_profile(sync_parameters=False):
            return False
        if self.profile_error:
            self._failed("config", self.profile_error)
            return False
        errors = validate_profile(self.profile, for_output=True)
        if errors:
            self._failed("config", "; ".join(errors))
            return False
        return True

    def _configure(self):
        if self._output_ready():
            self._submit("configure", {"profile": self.profile})

    def _start(self):
        if self.configured and not self.output_fault and self._output_ready():
            self._submit("start", {"mode": self.mode.currentData(), "profile": self.profile})

    @QtCore.Slot(str, object)
    def _completed(self, command, result):
        self.busy = False
        if command == "connect":
            self.connected = True
            self.output_fault = False
            self.configured = self.controller_deployed = False
            self.connection_state.setText("\u4eff\u771f\u5df2\u8fde\u63a5" if self.simulation.isChecked() else "Moku \u5df2\u8fde\u63a5")
        elif command == "disconnect":
            self.connected = self.running = False
            self.configured = self.controller_deployed = False
            self.connection_state.setText("\u672a\u8fde\u63a5")
        elif command == "deploy":
            self.controller_deployed = not self.pending_measurement_only
        elif command == "configure":
            self.output_fault = False
            self.configured = True
        elif command == "start":
            self.running = True
            self.run_state.setText("\u5b9e\u9a8c\u8fd0\u884c\u4e2d")
            self.times.clear()
            self.measurements.clear()
            self.references.clear()
            self.measurement_curve.clear()
            self.reference_curve.clear()
            self._fit_plot()
            self.experiment_path_label.setText(f"\u5b9e\u9a8c: {result.get('experiment_path', '--')}")
        elif command in {"stop", "finished"}:
            self.running = False
            self.run_state.setText("\u5df2\u5b8c\u6210" if command == "finished" else "\u5df2\u505c\u6b62")
        self.diagnostics.appendPlainText(f"{command}\n{json.dumps(result, ensure_ascii=False, indent=2, default=str)}")
        self.statusBar().showMessage(f"{command}: OK")
        self._update_controls()

    @QtCore.Slot(str, str)
    def _failed(self, command, message):
        self.busy = self.running = False
        if command not in {"import", "save", "config"}:
            self.output_fault = True
            self.configured = False
        self.run_state.setText("\u9519\u8bef")
        self.diagnostics.appendPlainText(f"{command}: {message}")
        self.statusBar().showMessage(f"{command}: {message}")
        self.tabs.setCurrentWidget(self.diagnostics)
        self._update_controls()

    def _set_plot_auto_range(self, enabled):
        view = self.plot.getViewBox()
        view.enableAutoRange(x=enabled, y=enabled)
        if enabled:
            view.updateAutoRange()

    def _fit_plot(self):
        with QtCore.QSignalBlocker(self.auto_plot_range):
            self.auto_plot_range.setChecked(True)
        self._set_plot_auto_range(True)

    def _plot_range_changed_manually(self, axes):
        self.auto_plot_range.setChecked(False)

    @QtCore.Slot(object)
    def _telemetry(self, sample):
        def number(key):
            value = sample.get(key)
            return float(value) if value is not None else float("nan")
        samples = sample.get("samples", [sample])
        step = max(1, len(samples) // 200)
        for point in samples[::step]:
            if point.get("measurement_nm") is not None:
                self.times.append(float(point.get("time_s", float("nan"))))
                self.measurements.append(float(point["measurement_nm"]))
                self.references.append(float(point.get("reference_nm", float("nan"))))
        if samples:
            self.measurement_curve.setData(list(self.times), list(self.measurements), connect="finite")
            self.reference_curve.setData(list(self.times), list(self.references), connect="finite")
            if self.auto_plot_range.isChecked():
                self._set_plot_auto_range(True)
        tracking_error = number("reference_nm") - number("measurement_nm")
        self.values.setText(f"\u4f4d\u79fb: {number('measurement_nm'):.2f} nm    "
                            f"\u8bef\u5dee: {tracking_error:.2f} nm    \u8f93\u51fa: {number('drive_v'):.4f} V")
        safety_faults = int(sample.get("faults") or 0) & ~int(Fault.DURATION)
        if safety_faults:
            self.run_state.setText(f"\u4fdd\u62a4: {safety_faults}")
            self.output_fault = True
            self._update_controls()
        elif sample.get("state"):
            states = {0: "\u5c31\u7eea", 1: "PID", 2: "Preview + ADRC", 3: "\u5b89\u5168\u56de\u843d", 4: "\u5f00\u73af",
                      "pid": "PID", "preview": "Preview + ADRC", "open_loop": "\u5f00\u73af"}
            self.run_state.setText(states.get(sample["state"], str(sample["state"])))

    def _update_controls(self):
        if not hasattr(self, "start_button"):
            return
        idle = not self.busy and not self.closing
        disconnected = not self.connected and idle
        self.connect_button.setEnabled(disconnected and not self.profile_error)
        self.disconnect_button.setEnabled(self.connected and idle)
        self.simulation.setEnabled(disconnected)
        self.address.setEnabled(disconnected and not self.simulation.isChecked())
        self.editor.setReadOnly(not idle or self.running)
        self.parameters.set_editable(idle and not self.running and not self.json_invalid, self.connected)
        self.import_button.setEnabled(disconnected)
        self.demo_button.setEnabled(disconnected)
        self.cli_path.setEnabled(disconnected)
        self.recording_path.setEnabled(disconnected)
        self.choose_recording.setEnabled(disconnected)
        for button in (self.diagnose_button, self.measurement_button, self.deploy_button):
            button.setEnabled(self.connected and idle and not self.running)
        self.configure_button.setEnabled(self.connected and idle and not self.running and self.controller_deployed and not self.profile_error)
        self.mode.setEnabled(idle and not self.running)
        self.measurement_only.setEnabled(idle and not self.running)
        self.start_button.setEnabled(self.connected and idle and not self.running and
                                     not self.profile_error and not self.output_fault and self.configured)
        self.stop_button.setEnabled(self.connected and not self.closing)

    def closeEvent(self, event):
        if self.closed:
            event.accept()
            return
        event.ignore()
        if not self.closing:
            self.closing = True
            self.local["window_geometry"] = bytes(self.saveGeometry().toBase64()).decode("ascii")
            self.local["cli_path"] = self.cli_path.text().strip() or None
            self.local["experiment_root"] = self.recording_path.text().strip() or None
            try:
                save_local_settings(self.local)
            except Exception as exc:
                self.diagnostics.appendPlainText(f"local settings: {exc}")
            self.statusBar().showMessage("\u6b63\u5728\u505c\u6b62\u5e76\u65ad\u5f00\u8fde\u63a5...")
            self._update_controls()
            self.request.emit("shutdown", {})

    @QtCore.Slot()
    def _shutdown_finished(self):
        self.closed = True
        self.close()


def run(profile_path=None, simulation=False):
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    application.setApplicationName("LaserAI")
    application.setOrganizationName("LaserAI")
    window = MainWindow(profile_path, simulation)
    window.show()
    return application.exec()
