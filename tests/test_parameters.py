"""Parameter forms retain calibration precision and all portable fields."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PySide6")
from PySide6 import QtCore, QtWidgets

from laserai.config import default_profile, simulation_profile
from laserai.parameters import ParameterPanel


@pytest.fixture
def panel():
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = ParameterPanel()
    yield widget
    widget.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    application.processEvents()


def test_all_profile_fields_round_trip_including_unknown_calibration(panel):
    profile = default_profile()
    panel.set_profile(profile)
    assert panel.profile() == profile
    expected = {(section, key) for section, value in profile.items() if isinstance(value, dict) for key in value}
    expected.update(("__root__", key) for key, value in profile.items() if not isinstance(value, dict))
    assert panel.fields.keys() == expected
    capacitance = panel.fields[("drive", "capacitance_f")]
    assert capacitance.unknown.isChecked()
    capacitance.set_value(1.23456789e-7)
    assert panel.profile()["drive"]["capacitance_f"] == pytest.approx(1.23456789e-7)
    capacitance.unknown.setChecked(True)
    assert panel.profile()["drive"]["capacitance_f"] is None


def test_waveform_relevance_preserves_values_and_all_six_waveforms(panel):
    panel.set_profile(simulation_profile())
    wave = panel.fields[("trajectory", "waveform")]
    assert {wave.input.itemData(i) for i in range(wave.input.count())} == {"dc", "sine", "square", "ramp", "pulse", "noise"}
    wave.input.setCurrentIndex(wave.input.findData("square"))
    assert panel.fields[("trajectory", "duty_pct")].isEnabled()
    assert panel.fields[("trajectory", "edge_time_s")].isEnabled()
    assert not panel.fields[("trajectory", "pulse_width_s")].isEnabled()
    wave.input.setCurrentIndex(wave.input.findData("noise"))
    assert panel.fields[("trajectory", "noise_seed")].isEnabled()
    assert not panel.fields[("trajectory", "frequency_hz")].isEnabled()
    assert panel.profile()["trajectory"]["frequency_hz"] == 20


def test_imc_tunes_visible_gains_and_connected_lock_is_scoped(panel):
    panel.set_profile(simulation_profile())
    panel.fields[("controller", "kp")].set_value(99)
    panel.tune()
    tuned = panel.profile()["controller"]
    assert 0 < tuned["kp"] < 1
    assert tuned["ki"] > 0
    assert tuned["observer_w0"] > 0
    panel.set_editable(True, connected=True)
    assert not panel.fields[("inputs", "carrier_hz")].isEnabled()
    assert panel.fields[("controller", "kp")].isEnabled()
    panel.set_editable(False, connected=True)
    assert not panel.fields[("controller", "kp")].isEnabled()
