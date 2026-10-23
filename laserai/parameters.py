"""Typed, nullable rig parameter forms shared by the Qt supervisor."""

from __future__ import annotations

import copy
import math

from PySide6 import QtCore, QtGui, QtWidgets


GROUPS = (
    ("轨迹", ("trajectory",)),
    ("控制与模型", ("controller", "model")),
    ("输出保护", ("drive",)),
    ("测量与标定", ("inputs", "calibration", "validation")),
    ("扰动与仿真", ("disturbance", "simulation_options")),
    ("实验与记录", ("experiment",)),
    ("设备与部署", ("device", "fpga", "__root__")),
)
SECTION_NAMES = {
    "trajectory": "位移轨迹 · Output 1", "controller": "PID / Preview + ADRC",
    "model": "对象模型", "drive": "驱动与输出限制", "inputs": "REF / MEAS 输入",
    "calibration": "位移换算", "validation": "实机验收记录", "disturbance": "输出扰动",
    "simulation_options": "仿真对象", "experiment": "实验", "device": "设备",
    "fpga": "FPGA 编译产物", "__root__": "配置",
}
LABELS = {
    "waveform": "波形", "frequency_hz": "频率 (Hz)", "amplitude_nm": "峰值幅度 (nm)",
    "offset_nm": "位移偏置 (nm)", "phase_deg": "初相位 (°)", "duty_pct": "占空比 (%)",
    "symmetry_pct": "上升段比例 (%)", "pulse_width_s": "脉宽 (s)", "edge_time_s": "边沿时间 (s)",
    "noise_seed": "噪声随机种子", "rate_hz": "控制更新率 (Hz)", "kp": "Kp (V/nm)",
    "ki": "Ki (V/nm/s)", "kd": "Kd (V·s/nm)", "d_filter_n": "微分滤波 N",
    "observer_w0": "观测器带宽 w0 (rad/s)", "gain_nm_per_v": "对象增益 (nm/V)",
    "tau_s": "时间常数 (s)", "plant_delay_s": "执行延迟 (s)", "sensor_delay_s": "测量延迟 (s)",
    "piezo_model": "压电型号", "capacitance_f": "压电电容 (F)", "rated_voltage_v": "额定电压 (V)",
    "minimum_v": "输出下限 (V)", "maximum_v": "输出上限 (V)", "safe_v": "安全电压 (V)",
    "slew_v_per_s": "最大变化率 (V/s)", "max_abs_error_nm": "误差保护阈值 (nm)",
    "max_current_a": "电流上限 (A)", "counts_per_volt": "输出换算 (counts/V)",
    "standard": "输入电平标准", "receiver": "接收与电平适配", "carrier_hz": "载波频率 (Hz)",
    "level_vpp": "输入峰峰值 (Vpp)", "impedance": "输入阻抗", "coupling": "耦合方式",
    "range": "输入量程", "pll_bandwidth": "PLL 带宽", "phase_scale_v_per_cycle": "相位输出 (V/cycle)",
    "phase_period_counts": "相位周期 (counts)", "signal_threshold_counts": "信号阈值 (counts)",
    "signal_timeout_s": "失锁超时 (s)", "nm_per_cycle": "每周期位移 (nm/cycle)", "direction": "位移方向 (+1 / −1)",
    "signal_conditioning": "电平适配已验证", "phase_scale": "相位比例已验证", "phase_wrap": "相位回绕已验证",
    "latency": "测量延迟已验证", "loss_of_lock": "失锁保护已验证", "polarity": "反馈极性已验证",
    "direct_drive": "直接驱动已验证", "duration_s": "持续时间 (s)", "settle_s": "稳态等待 (s)",
    "independent_phase_channels": "双路独立测相已验证",
    "repeats": "A/B 重复次数", "target_rmse_nm": "RMSE 目标 (nm)", "address": "设备 IP / 地址",
    "hardware": "设备型号", "slots": "仪器槽数", "bitstream_path": "Bitstream 路径",
    "manifest_path": "编译清单路径", "schema_version": "配置版本", "simulation": "仿真配置",
    "enabled": "启用扰动", "amplitude_v": "扰动峰值 (V)", "start_s": "开始时间 (s)",
    "seed": "随机种子", "measurement_noise_nm": "测量噪声 RMS (nm)",
    "hysteresis_enabled": "启用迟滞", "hysteresis_a": "迟滞 A", "hysteresis_beta": "迟滞 β",
    "hysteresis_gamma": "迟滞 γ", "hysteresis_nm_per_v": "迟滞幅度 (nm/V)", "dead_band_v": "死区 (V)",
}
TEXT_KEYS = {"standard", "receiver", "piezo_model", "address", "hardware", "bitstream_path", "manifest_path"}
INTEGER_KEYS = {"schema_version", "slots", "repeats", "noise_seed", "seed", "phase_period_counts", "signal_threshold_counts"}
ENUMS = {
    ("trajectory", "waveform"): (("正弦", "sine"), ("直流", "dc"), ("方波", "square"),
                                  ("斜坡", "ramp"), ("脉冲", "pulse"), ("噪声", "noise")),
    ("disturbance", "waveform"): (("阶跃", "step"), ("正弦", "sine"), ("方波", "square"), ("噪声", "noise")),
    ("inputs", "coupling"): (("AC", "AC"), ("DC", "DC")),
    ("inputs", "impedance"): (("1 MΩ", "1MOhm"), ("50 Ω", "50Ohm")),
    ("inputs", "range"): (("10 Vpp", "10Vpp"), ("50 Vpp", "50Vpp")),
    ("inputs", "pll_bandwidth"): tuple((v, v) for v in ("1Hz", "10Hz", "100Hz", "1kHz", "10kHz", "100kHz")),
}
LOCKED_SECTIONS = {"device", "inputs", "calibration", "validation", "fpga", "__root__"}


class ParameterField(QtWidgets.QWidget):
    changed = QtCore.Signal()

    def __init__(self, section, key, value):
        super().__init__()
        self.key = key
        self.boolean = isinstance(value, bool)
        self.numeric = key not in TEXT_KEYS and not self.boolean and (isinstance(value, (int, float)) or value is None)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        if self.boolean:
            self.input = QtWidgets.QCheckBox()
            self.input.toggled.connect(self.changed)
        elif (section, key) in ENUMS:
            self.input = QtWidgets.QComboBox()
            for label, option in ENUMS[(section, key)]:
                self.input.addItem(label, option)
            self.input.currentIndexChanged.connect(self.changed)
        else:
            self.input = QtWidgets.QLineEdit()
            if self.numeric:
                validator = QtGui.QDoubleValidator(self.input)
                validator.setNotation(QtGui.QDoubleValidator.Notation.ScientificNotation)
                validator.setLocale(QtCore.QLocale.c())
                self.input.setValidator(validator)
                self.input.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
            self.input.textEdited.connect(self.changed)
        layout.addWidget(self.input, 1)
        self.unknown = None
        if not self.boolean and not isinstance(self.input, QtWidgets.QComboBox):
            self.unknown = QtWidgets.QCheckBox("未知")
            self.unknown.toggled.connect(lambda checked: self.input.setEnabled(not checked))
            self.unknown.toggled.connect(self.changed)
            layout.addWidget(self.unknown)
        self.set_value(value)

    def set_value(self, value):
        blockers = [QtCore.QSignalBlocker(self.input)]
        if self.unknown:
            blockers.append(QtCore.QSignalBlocker(self.unknown))
            self.unknown.setChecked(value is None)
            self.input.setEnabled(value is not None)
        if isinstance(self.input, QtWidgets.QCheckBox):
            self.input.setChecked(bool(value))
        elif isinstance(self.input, QtWidgets.QComboBox):
            index = self.input.findData(value)
            if index < 0:
                self.input.addItem(str(value), value)
                index = self.input.count() - 1
            self.input.setCurrentIndex(index)
        else:
            self.input.setText("" if value is None else str(value))

    def value(self):
        if self.unknown and self.unknown.isChecked():
            return None
        if isinstance(self.input, QtWidgets.QCheckBox):
            return self.input.isChecked()
        if isinstance(self.input, QtWidgets.QComboBox):
            return self.input.currentData()
        text = self.input.text().strip()
        if not self.numeric:
            return text
        try:
            value = float(text)
        except ValueError as exc:
            raise ValueError(f"{LABELS.get(self.key, self.key)}: 数值无效") from exc
        if not math.isfinite(value) or self.key in INTEGER_KEYS and not value.is_integer():
            raise ValueError(f"{LABELS.get(self.key, self.key)}: 数值无效")
        return int(value) if self.key in INTEGER_KEYS else value


class ParameterPanel(QtWidgets.QWidget):
    changed = QtCore.Signal()
    error = QtCore.Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.fields = {}
        self._profile = {}
        self.editable = True
        self.connected = False
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.navigation = QtWidgets.QListWidget()
        self.navigation.setFixedWidth(126)
        self.navigation.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.pages = QtWidgets.QStackedWidget()
        self.navigation.currentRowChanged.connect(self.pages.setCurrentIndex)
        layout.addWidget(self.navigation)
        layout.addWidget(self.pages, 1)

    def set_profile(self, profile):
        self._profile = copy.deepcopy(profile)
        values = {(section, key): value for section, obj in profile.items() if isinstance(obj, dict)
                  for key, value in obj.items()}
        values.update({("__root__", key): value for key, value in profile.items() if not isinstance(value, dict)})
        if values.keys() == self.fields.keys():
            for path, value in values.items():
                self.fields[path].set_value(value)
            self.set_editable(self.editable, self.connected)
            return
        selected = max(0, self.navigation.currentRow())
        self.navigation.clear()
        while self.pages.count():
            page = self.pages.widget(0)
            self.pages.removeWidget(page)
            page.deleteLater()
        self.fields = {}
        groups = list(GROUPS)
        known = {section for _, sections in groups for section in sections}
        extra = tuple(section for section in profile if section not in known and isinstance(profile[section], dict))
        if extra:
            groups.append(("其他", extra))
        for title, sections in groups:
            page = QtWidgets.QWidget()
            page_layout = QtWidgets.QVBoxLayout(page)
            for section in sections:
                entries = [(key, value) for (sec, key), value in values.items() if sec == section]
                if not entries:
                    continue
                heading = QtWidgets.QLabel(SECTION_NAMES.get(section, section))
                font = heading.font()
                font.setBold(True)
                heading.setFont(font)
                page_layout.addWidget(heading)
                form = QtWidgets.QFormLayout()
                form.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
                form.setRowWrapPolicy(QtWidgets.QFormLayout.RowWrapPolicy.WrapLongRows)
                for key, value in entries:
                    field = ParameterField(section, key, value)
                    field.setToolTip(f"{section}.{key}")
                    field.changed.connect(self._field_changed)
                    self.fields[(section, key)] = field
                    form.addRow(LABELS.get(key, key), field)
                page_layout.addLayout(form)
                if section == "controller":
                    self.tune_button = QtWidgets.QPushButton("IMC 整定")
                    self.tune_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.StandardPixmap.SP_DialogApplyButton))
                    self.tune_button.clicked.connect(self.tune)
                    page_layout.addWidget(self.tune_button, 0, QtCore.Qt.AlignmentFlag.AlignLeft)
            page_layout.addStretch()
            scroll = QtWidgets.QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
            scroll.setWidget(page)
            self.pages.addWidget(scroll)
            self.navigation.addItem(title)
        self.navigation.setCurrentRow(min(selected, self.navigation.count() - 1))
        self.set_editable(self.editable, self.connected)

    def profile(self):
        result = copy.deepcopy(self._profile)
        for (section, key), field in self.fields.items():
            if section == "__root__":
                result[key] = field.value()
            else:
                result[section][key] = field.value()
        return result

    def set_editable(self, editable, connected=False):
        self.editable, self.connected = editable, connected
        for (section, _), field in self.fields.items():
            field.setEnabled(editable and not (connected and section in LOCKED_SECTIONS))
        if hasattr(self, "tune_button"):
            self.tune_button.setEnabled(editable)
        self._update_relevance()

    def _field_changed(self):
        self._update_relevance()
        self.changed.emit()

    def _update_relevance(self):
        def option(section, key, fallback=None):
            field = self.fields.get((section, key))
            return field.value() if field else fallback

        waveform = option("trajectory", "waveform", "sine")
        relevant = {
            "frequency_hz": waveform not in {"dc", "noise"},
            "phase_deg": waveform not in {"dc", "noise"},
            "duty_pct": waveform == "square", "symmetry_pct": waveform == "ramp",
            "pulse_width_s": waveform == "pulse", "edge_time_s": waveform in {"pulse", "square"},
            "noise_seed": waveform == "noise",
        }
        for key, active in relevant.items():
            if ("trajectory", key) in self.fields:
                self.fields[("trajectory", key)].setEnabled(self.editable and active)
        if ("trajectory", "amplitude_nm") in self.fields:
            self.fields[("trajectory", "amplitude_nm")].setToolTip(
                "直流电平 = 位移偏置 + 幅度" if waveform == "dc" else "位移轨迹峰值幅度 (nm)")
        enabled = option("disturbance", "enabled", False)
        disturbance_wave = option("disturbance", "waveform", "step")
        for (section, key), field in self.fields.items():
            if section == "disturbance" and key != "enabled":
                active = enabled
                if key in {"frequency_hz", "phase_deg"}:
                    active = active and disturbance_wave in {"sine", "square"}
                if key == "seed":
                    active = active and disturbance_wave == "noise"
                field.setEnabled(self.editable and active)
            elif section == "simulation_options":
                active = bool(option("__root__", "simulation", False))
                if key.startswith("hysteresis_") and key != "hysteresis_enabled":
                    active = active and option("simulation_options", "hysteresis_enabled", False)
                field.setEnabled(self.editable and active)

    def tune(self):
        try:
            from control import PlantModel, imc_tune

            p = self.profile()
            m, d, c = (p[key] for key in ("model", "drive", "controller"))
            model = PlantModel(K_nm_per_V=m["gain_nm_per_v"], tau_s=m["tau_s"],
                               theta_plant_s=m["plant_delay_s"], theta_sensor_s=m["sensor_delay_s"],
                               v_max=d["maximum_v"])
            gains = imc_tune(model, 1 / c["rate_hz"], fs_sample_hz=c["rate_hz"])
            for key in ("kp", "ki", "kd", "d_filter_n"):
                self.fields[("controller", key)].set_value(gains[key])
            self.fields[("controller", "observer_w0")].set_value(gains["w0"])
            self.changed.emit()
        except (TypeError, ValueError, ZeroDivisionError) as exc:
            self.error.emit(f"IMC: {exc}")
