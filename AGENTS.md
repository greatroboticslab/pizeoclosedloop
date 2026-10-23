# AGENTS.md

Python 3.13 cross-platform Moku:Go host (PySide6/pyqtgraph). `app.py` launches the
new application; `legacy_app.py` preserves the old Windows Tk/VB/MQTT workflow.

## Commands

- Run without hardware: `python app.py --simulation`
- Startup diagnostics: `python app.py --diagnose`
- Host and reference tests: `python -m pytest tests test_control.py -v`
- Headless Qt tests: set `QT_QPA_PLATFORM=offscreen`
- Build natively on each OS: `python -m PyInstaller moku_host.spec`
- Legacy integration tests: `python -m pytest test_closed_loop.py -v` (requires Tk/display; auto-skips without it)

No lint/typecheck/format config exists; do not invent tooling.

## Architecture

- The new runtime never starts VB, Mosquitto, Windows batch files or the bundled
  Windows CLI. Moku performs real-time acquisition, conversion, reference
  generation, control and protection; Python performs configuration and display.
- `laserai/` owns the new host. Device operations belong behind its device
  interface, not in widgets. Communication, streaming and saving run off the Qt
  main thread. Widgets update only on the main thread.
- `fpga/` owns controller sources and the register contract in
  `laserai/protocol.py`. A source file is not a deployable bitstream. Require a
  compatible compiled target and verified hardware calibration before output.
- Moku:Go three-slot design: Phasemeter, Custom Instrument and Datalogger. Only
  custom OutputA/B are routable; Datalogger has two input channels. Do not claim
  four synchronous telemetry channels or infer PLL lock from host refresh rate.
- `control.py` is the headless numerical reference. Keep Tk, Qt, MQTT and Moku
  imports out of it. Its simulation defaults are not hardware calibration.
- Hardware and local settings live under `~/.laserai/`, using `pathlib`.
  Hardware profiles must fail on corruption; never substitute defaults that can
  energize an output. Missing calibration permits diagnostics and simulation,
  but blocks physical output. Unknown physical values are JSON `null`.
- Portable rig profiles and machine-specific settings are separate. Explicit
  local CLI path takes priority over `MOKU_CLI_PATH`, then SDK discovery.
- Every experiment snapshots configuration, firmware/device identity and the
  bitstream identity. Never claim hardware-qualified performance from simulation.

## Legacy Contracts

- In `legacy_app.py`, VB `uMD_GUI.exe` publishes `refCount,D,phaseRaw` over MQTT
  (`vb_to_py`); `display.py` converts it to nm. Preserve this legacy path.
- Two old PID features share Output 1: FPGA waveform smoothing in
  `moku_waveform.py`, and software feedback in `closed_loop.py`. They are mutually
  exclusive and do not belong in the new runtime.
- Legacy widgets taking `bootstyle` must be `tb.*`, not plain `ttk.*`.
- `process_raw.py` uses Agg; `closed_loop.py` embeds `FigureCanvasTkAgg` without
  pyplot. Legacy packaging needs `matplotlib.backends.backend_tkagg`.
- Legacy `D:<count> N:<serial>` logs remain compatible with `process_raw.py`.
- `app_settings.py` saves legacy JSON in the user's home directory; its tolerant
  loading behavior is unsuitable for the new hardware configuration.

## Validation

Tests require no Moku, MQTT or broker. Numerical Preview comparisons use the
MATLAB reference's exact-model/no-noise acceptance case. Hardware acceptance is
staged: measure phase range/wrap/latency/loss first, then low-amplitude open loop,
PID and Preview+ADRC. Target: 20 Hz, at least 500 nm peak, steady RMSE <=50 nm.
Report simulation, HDL simulation, synthesis and physical tests separately.
