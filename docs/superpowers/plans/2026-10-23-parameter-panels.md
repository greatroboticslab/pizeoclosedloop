# Parameter Panels Implementation Plan

> **For agentic workers:** Use executing-plans to implement this plan task by task.

**Goal:** Restore the legacy application's editable control parameters in the Qt host, including working waveforms, disturbance injection and simulation options.

**Architecture:** A typed Qt parameter panel edits the same portable profile as advanced JSON. Idle configuration updates go through the existing serialized device worker; all physical control and injected drive disturbances remain in FPGA logic. Simulation-only noise/hysteresis are isolated from real device deployment.

**Tech Stack:** Python 3.13, PySide6, pyqtgraph, SystemVerilog, pytest and Icarus.

**Spec:** User request to inspect legacy closed_loop.py/moku_waveform.py and restore their parameter controls.

## Global Constraints

- Preserve local changes and the codex/moku-go-migration branch.
- Keep a single axis, common trajectory/clock/limits, and fail closed on invalid hardware configuration.
- Keep unknown rig parameters nullable, config import/export round trips, and existing version-1 profiles loadable through explicit default additions.
- Rebuild the macOS app and open the updated parameter interface after verification.

## Tasks

- [x] Extend profile fields for trajectory phase/duty/symmetry/pulse edges, simulation dead band, drive disturbance, and simulation noise/hysteresis. Validate conditional settings and document defaults added to older profiles.
- [x] Extend the versioned FPGA ABI for waveform geometry and bounded disturbance injection; update the offline fixed-point reference and actual HDL tests together. Old bitstreams must be rejected by the new ABI.
- [x] Add Qt forms for trajectory, model, PID/Preview gains, protections, disturbances and experiment/recording options. Retain advanced JSON and expose IMC tuning using the existing numerical helper.
- [x] Serialize idle profile updates on the device worker, invalidate prior parameter commits, preserve deployed instruments where safe, and snapshot the applied profile for every experiment.
- [x] Verify field round trips, null handling, invalid input, connected edits, idle/run locking, waveform/perturbation numerical behavior and output protection. Run the host/reference/HDL tests, rebuild the native app, inspect screenshots, and launch it.

Verified on macOS: full suite 196 passed; final GUI/parameter/workflow check 12
passed after the last UI adjustment. Native build and packaged simulation
diagnostics exited zero. Evidence: `reports/parameter-panels-2026-10-23/report.md`.
Physical Moku synthesis, timing closure and performance remain unverified.

## Interfaces

- `laserai.config.normalize_profile(profile)` adds only new non-energizing fields for older v1 profile files; it does not supply missing legacy calibration values.
- `ParameterPanel.set_profile(profile)`, `ParameterPanel.profile()`, `ParameterPanel.changed` provide the Qt/profile boundary.
- Worker command `update_profile` validates/copies an idle profile into the existing device session, marks the controller unconfigured and returns the applied profile.
- FPGA configuration keeps addresses 0..35 and adds waveform geometry, deterministic noise seed and timed drive disturbance words. `profile_words`, `board_reference`, `FixedController` and RTL share the resulting ABI.

## Verification

Run `.venv/bin/python -m pytest tests test_control.py -v`, including Icarus tests. Run `.venv/bin/python -m PyInstaller --noconfirm moku_host.spec`, exercise the packaged simulator and inspect the parameter forms at normal and compact window sizes. Hardware synthesis, real acquisition and closed-loop performance remain unverified without a physical device.
