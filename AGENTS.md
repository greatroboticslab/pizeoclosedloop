# AGENTS.md

Python 3.13 desktop app (ttkbootstrap/Tk) orchestrating a laser interferometry workflow: Moku:Go waveform generation, a bundled VB app (`umd_gui/uMD_GUI.exe`) that publishes interferometer values over MQTT (topic `vb_to_py`), a bundled Mosquitto broker, recording, and offline processing. Windows is the primary target.

## Commands

- Run: `python app.py`
- Tests (headless, no hardware): `python3 -m pytest test_control.py -v`
- Integration tests (need a display/Tk; auto-skip without one): `python3 -m pytest test_closed_loop.py -v`
- `test_control.py` also runs standalone: `python3 test_control.py`
- Build Windows exe: use the spec file, e.g. `python -m PyInstaller LaserLab_v4.spec` (or the one-liner in README; PowerShell backtick continuation)

No lint/typecheck/format config exists; don't invent tooling.

## Architecture facts an agent would miss

- **Python does not do the low-level interferometer decoding.** The VB `uMD_GUI.exe` is part of the live data path: it publishes `refCount,D,phaseRaw` payloads that `display.py` parses and converts to nm. Keep this pipeline intact.
- **Two different "PID" features exist and are mutually exclusive** (both drive Moku Output 1):
  - Moku tab "PID Smoothing": PID in the Moku FPGA, sees only the waveform generator output.
  - `closed_loop.py` "Closed-Loop Control": software PID / Preview+ADRC in Python, closing the loop via MQTT → `control.py` → `moku_waveform.py:set_dc_voltage()`.
- `control.py` is deliberately free of Tk/MQTT/Moku imports so it stays headless-testable. Keep GUI code out of it.
- `moku_waveform.py` deploys Moku Multi-Instrument Mode (slot 1 WG, slot 2 PID); it is the single source of truth for waveform config — the Record tab mirrors it, does not own it.
- Closed-loop seam: `display.py:DisplayFrame.read_latest()` (measurement in) and `moku_waveform.py:set_dc_voltage()` (actuation out).
- `app_settings.py` stores user settings as JSON under the home directory (not next to code) because PyInstaller's `_MEIPASS` is read-only and temporary. Don't write settings into the app directory.

## Conventions

- Tk widgets that take a `bootstyle` option must be `tb.*` (ttkbootstrap), not plain `ttk.*` (required for ttkbootstrap 2.x compatibility). Plain ttk is fine for labels/entries/frames.
- UI updates only on the main thread; worker threads read shared "latest" values rather than touching widgets.
- No hardcoded absolute paths; use the PyInstaller-safe `BASE_DIR` helpers from `app.py`.
- `process_raw.py` pins the matplotlib global backend to Agg; `closed_loop.py` embeds figures via `FigureCanvasTkAgg` without pyplot. Preserve this split — PyInstaller needs `matplotlib.backends.backend_tkagg` listed as a hiddenimport for this reason.
- Recorded log format (`D:<count> N:<serial>`) is a contract with `process_raw.py`; keep them compatible.

## Testing notes

- `control.py` was ported from a MATLAB project (`/Users/nccer/Documents/meter`); test acceptance criteria mirror `meter/test_simulate.m` — with an exact model and no noise, Preview must track better than PID.
- Tests never require Moku hardware, MQTT, or the broker.
