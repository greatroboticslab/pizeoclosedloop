# LaserAI

Cross-platform Moku:Go host application for a single-axis laser interferometer.
The new application uses Python 3.13, PySide6 and pyqtgraph on macOS, Linux and
Windows. Acquisition, displacement conversion, trajectories, feedback and output
protection belong on Moku; the host configures experiments, displays telemetry
and saves results.

This branch is the migration implementation. FPGA source and simulation are not
a compiled, hardware-qualified controller. The hardware target is a 20 Hz sine
with at least 500 nm peak displacement and steady-state RMSE at most 50 nm;
meeting it requires the staged measurements described in
[the migration guide](docs/moku-go-migration.md).
Local official-Moku demo observations and packaged-host checks are recorded in
[the environment report](docs/moku-demo-environment.md), including an unresolved
Phasemeter channel limitation in the demo's three-slot layout.

## Install And Run

Create a Python 3.13 virtual environment, activate it, then install the runtime:

```sh
python -m venv .venv
python -m pip install -r requirements.txt
python app.py --simulation
```

Activate with `source .venv/bin/activate` on macOS/Linux or
`.venv\Scripts\Activate.ps1` in Windows PowerShell before installing.
The simulator exercises the host without hardware and does not establish real
tracking performance. To select a portable profile:

```sh
python app.py --profile configs/moku-go.template.json
python app.py --simulation --profile configs/simulation.json
python app.py --diagnose
python app.py --help
```

`--diagnose` connects using the selected profile and prints diagnostics as JSON
without opening the interface; use `--simulation --diagnose` without a device.
Unknown rig parameters in the hardware template are `null`. Complete and verify
them before enabling hardware output. Malformed hardware profiles are errors;
they must never silently become a default drive configuration.

## Moku Setup

Install [MokuCLI](https://apis.liquidinstruments.com/cli/getting-started/install.html)
separately on each computer. Configure its path in local settings, set
`MOKU_CLI_PATH`, or use SDK discovery, in that priority order. The app does not
bundle the old Windows CLI or an instrument cache.

```sh
mokucli --help
mokucli list
mokucli instrument download <your-device-MokuOS-version>
```

Use the actual device firmware version when downloading instruments. The
application reads device identity and firmware and checks a custom bitstream's
manifest against its target before accepting it. Cloud compilation and the
Phasemeter/Custom Instrument licenses are external requirements.

Ethernet is the common acceptance path on all three operating systems. USB is
optional and uses IPv6. Windows USB requires the Moku driver installed with the
Moku Desktop application; see the
[official connection guide](https://apis.liquidinstruments.com/api/getting-started/starting-python.html).

REF/~REF and MEAS/~MEAS are differential pairs. Use suitable differential
receivers and electrical level conditioning before the analog inputs. They are
not four directly interchangeable Moku input channels.

## Configuration And Experiments

Runtime settings live under `~/.laserai/` on every platform:

| File | Purpose |
| --- | --- |
| `hardware.json` | Portable rig calibration, model, controller and protection settings |
| `local.json` | Machine-specific CLI path, cache and interface preferences |

The repository templates are starting points, not measured calibration.
Portable profiles may be imported/exported between computers; machine-local
paths must be configured separately. Each saved experiment includes its
configuration and device/bitstream identity. See
[configuration and acceptance details](docs/moku-go-migration.md).

The **参数** tab contains editable trajectory, PID/Preview, plant model,
protection, measurement/calibration, disturbance and experiment forms; advanced
JSON edits the same profile. Drive disturbances have their own amplitude in V
and timed activation. Measurement noise, dead band and hysteresis belong to
simulation options. Editing connected idle parameters requires writing them
again before starting; parameters are locked during a run.

## Tests And Packaging

```sh
python -m pip install -r requirements-dev.txt
python -m pytest tests test_control.py -v
python -m PyInstaller --clean --noconfirm moku_host.spec
```

Set `QT_QPA_PLATFORM=offscreen` when testing without a desktop. CI runs host and
numerical tests with Python 3.13 on macOS, Linux and Windows; hardware acceptance
is separate. Build on the operating system being shipped: macOS produces
`dist/LaserAI.app`; Windows produces `dist/LaserAI/LaserAI.exe`; Linux produces
`dist/LaserAI/LaserAI`. Signing/notarization and installer creation are separate
release steps. The spec does not package VB, MQTT, old CLI binaries or caches.

## Legacy Application

The previous Tk application is preserved as `legacy_app.py`, with its original
VB/MQTT data path. Install `requirements-legacy.txt` to
run it. Its modules remain available for old datasets and comparison, but the
new `app.py` does not launch the VB application or Mosquitto. `control.py`
remains the headless numerical reference; the simulator is not a Python-based
hardware feedback loop. Legacy logs use `D:<count> N:<serial>` and remain
compatible with `process_raw.py`.

## License

MIT
