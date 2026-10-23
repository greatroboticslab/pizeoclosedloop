# Moku:Go Migration And Acceptance

The new host uses one Python 3.13/PySide6/pyqtgraph implementation on macOS,
Linux and Windows. Its device interface owns connection, deployment, parameter
transactions, start/stop and telemetry. Widgets never perform a control update.
The legacy Tk/VB/MQTT application has a separate launcher.

## Board Architecture

The proposed three-slot configuration is:

```text
REF/~REF  -> receiver/level adapter -> Input 1 -> Phasemeter (slot 1)
MEAS/~MEAS -> receiver/level adapter -> Input 2 -> Phasemeter (slot 1)
Phasemeter phase outputs -> Custom Instrument (slot 2)
Custom Output A -> protected drive -> physical Output 1
Custom Output A/B -> Datalogger (slot 3), subject to validated routing
Host <- configuration acknowledgments, status and recorded telemetry
```

This routing is conditional on two independently usable Phasemeter channels in
the selected Go MiM target. The installed official demo exposes only Input A
in its applied three-slot Phasemeter window; generic API channel lists do not
resolve that discrepancy. See [the demo evidence](moku-demo-environment.md).
Validate actual device capabilities before routing REF and MEAS this way.

The receivers perform electrical adaptation, not displacement computation.
Establish the actual differential standard, common-mode range, termination,
carrier, amplitude and receiver delay before connecting the inputs. A native
Phasemeter phase output is not automatically a calibrated displacement stream.
Verify phase scale, sign, wrapping, shared reference, latency and loss of lock.

Moku:Go's custom logic clock is 31.25 MHz. A divider of 31250 gives the initial
1 kHz control update. In three-slot mode each custom slot has a budget of
12000 LUTs, 24000 flip-flops, 40 BRAMs and 60 DSPs. Check the actual synthesis
report and timing before deployment. See the official
[slot resource table](https://apis.liquidinstruments.com/mc/slots.html).

The custom wrapper uses signed 16-bit ports; this does not make the physical
ADC/DAC 16-bit. Go has 12-bit converters. In three-slot mode only OutputA and
OutputB are routable; OutputC/D are unconnected. InputC/D are tied to ADC1/2.
The nominal inter-instrument/DAC scale is 6550.4 counts/V, subject to the
configured front-end and output gain. See
[the wrapper I/O documentation](https://apis.liquidinstruments.com/mc/io.html).
The Datalogger has two channels. A proposed telemetry multiplexer must be
framed and tested for missed samples and clock alignment before its output can
be used for four-variable tracking metrics. Host status snapshots must not be
presented as four simultaneous full-rate logged signals.

## Portable Profile Fields

`configs/moku-go.template.json` is intentionally incomplete. `null` means
unknown, not zero or an assumed default. `configs/simulation.json` is synthetic
and must not be treated as rig calibration. JSON profiles use `schema_version`
1 and preserve the distinction between simulation and hardware.

| Section / fields | Meaning and units |
| --- | --- |
| `device.address`, `hardware`, `slots` | SDK network address, `mokugo`, three-slot target |
| `inputs.standard`, `receiver` | Actual differential standard and electrical receiver/adapter identification |
| `inputs.carrier_hz`, `level_vpp` | Measured carrier frequency in Hz and conditioned analog amplitude in Vpp |
| `inputs.impedance`, `coupling`, `range` | Moku input termination, coupling and full-scale range accepted by the SDK |
| `inputs.pll_bandwidth` | Phasemeter tracking bandwidth accepted by the SDK |
| `inputs.phase_scale_v_per_cycle` | Programmed Phasemeter output volts per phase cycle |
| `inputs.phase_period_counts` | Measured integer count period at the custom input after all routing/scaling |
| `inputs.signal_threshold_counts`, `signal_timeout_s` | Carrier activity gate and allowed absence interval; this alone is not proof of PLL lock |
| `calibration.nm_per_cycle`, `direction` | Calibrated nm per phase cycle and polarity `1` or `-1` |
| `drive.piezo_model`, `capacitance_f`, `rated_voltage_v` | Identified piezo, measured/rated capacitance in F, allowed voltage magnitude |
| `drive.minimum_v`, `maximum_v`, `safe_v` | Allowed output interval and configured stopped/fault voltage, in V |
| `drive.slew_v_per_s`, `max_current_a` | Maximum output slope and allowed source/sink current |
| `drive.max_abs_error_nm`, `counts_per_volt` | Tracking-error trip threshold and calibrated DAC count scale |
| `model.gain_nm_per_v`, `tau_s` | Identified first-order gain and time constant |
| `model.plant_delay_s`, `sensor_delay_s` | Identified plant and sensing delays in seconds |
| `controller.rate_hz` | Initial supported update rate: 1000 Hz |
| `controller.kp`, `ki`, `kd` | PID coefficients: V/nm, V/(nm*s), V*s/nm |
| `controller.d_filter_n`, `observer_w0` | Derivative filter rate and observer bandwidth, in inverse seconds |
| `trajectory.waveform`, `frequency_hz` | `dc`, `sine`, `square`, `ramp`, `pulse` or `noise`; reference frequency in Hz |
| `trajectory.amplitude_nm`, `offset_nm` | Peak amplitude, not peak-to-peak, and displacement offset in nm |
| `trajectory.phase_deg`, `duty_pct`, `symmetry_pct` | Initial phase in degrees, square duty and ramp rising-time fraction in percent |
| `trajectory.pulse_width_s`, `edge_time_s`, `noise_seed` | Pulse high interval and transition time in seconds; reproducible trajectory noise seed |
| `disturbance.enabled`, `waveform` | Enable additive drive disturbance; `step`, `sine`, `square` or `noise` |
| `disturbance.amplitude_v`, `frequency_hz`, `phase_deg` | Drive disturbance peak voltage, frequency and initial phase |
| `disturbance.start_s`, `duration_s`, `seed` | Active interval measured from run start; deterministic noise seed |
| `simulation_options.measurement_noise_nm`, `noise_seed` | Gaussian measurement noise standard deviation and random seed, simulation only |
| `simulation_options.dead_band_v` | Simulated actuator dead band, in V |
| `simulation_options.hysteresis_enabled`, `hysteresis_a`, `hysteresis_beta`, `hysteresis_gamma`, `hysteresis_nm_per_v` | Simulated Bouc-Wen hysteresis switch, shape coefficients and displacement scale |
| `experiment.duration_s`, `settle_s`, `repeats` | Run duration, excluded initial settling interval and repetition count |
| `experiment.target_rmse_nm` | Acceptance threshold for steady-state RMSE |
| `validation.*` | Recorded results of the physical acceptance gates below; do not mark true from simulation |
| `fpga.bitstream_path`, `manifest_path` | Local compiled bitstream and compatible identity/target manifest |

Store the active hardware profile at `~/.laserai/hardware.json`. Local settings
are `~/.laserai/local.json`; `cli_path`, cache paths and interface preferences
belong there rather than in an exported rig profile. Explicit CLI path wins
over `MOKU_CLI_PATH`, followed by SDK discovery. The profile loader rejects
malformed JSON, NaN/infinity, incorrect structure and invalid field types.
Output validation additionally requires complete physical parameters and the
hardware acceptance flags. An error must leave output disabled.

## Editing Parameters

The Qt host provides named parameter forms alongside advanced JSON. Both edit
the same profile. Restore the trajectory, object model, PID gains, derivative
filter, Preview observer, voltage/slew limits, acquisition/calibration and
experiment settings here rather than editing source code. IMC tuning uses the
legacy numerical helper and writes its calculated gains into the profile.
The single-axis architecture drives Output 1; the old waveform-smoothing PID
is not an additional controller in this signal path.

The reference amplitude is displacement peak in nm. It is not the legacy
waveform-generator voltage Vpp. Control remains at 1 kHz, so short pulse edges
are limited by that update period. A DC reference retains the legacy convention
`offset_nm + amplitude_nm`. Ramp is an asymmetric triangle: it rises over the
configured symmetry fraction of a cycle, then falls during the rest.
Square spans `offset +/- amplitude`; pulse spans `offset` to
`offset + amplitude`. Pulse duty is `pulse_width_s * frequency_hz`.
`edge_time_s` is the complete linear transition duration, centered on each
square/pulse edge; transitions cannot overlap. Non-square/pulse waveforms ignore
the stored duty and edge settings. Trajectory and disturbance noise use separate
deterministic unsigned 32-bit seeds.

Drive disturbance is injected after the selected control law and before the
common voltage and slew protections. It is active only during its configured
time interval. PID and Preview A/B runs use the same disturbance profile and
seed. Noise, dead band and hysteresis settings under `simulation_options` affect
only the offline plant; they do not configure physical piezo characteristics.
Physical disturbances and simulated plant imperfections are different tests.
Measurement noise is Gaussian with the configured standard deviation in nm;
each run resets its generator to the same seed, including PID/Preview pairs.
The simulated dead band maps voltage to `sign(v) * max(abs(v)-dead_band_v, 0)`.
Hysteresis follows the legacy Bouc-Wen convention: for effective voltage change
`dv`, update `z += a*dv - beta*abs(dv)*z - gamma*dv*abs(z)`, then subtract
`hysteresis_nm_per_v*z` from the first-order plant displacement. These optional
imperfections are synthetic scenarios and start disabled. Snapshots identify
the effective plant, including any deliberate model mismatch.
Physical initial open-loop validation rejects enabled drive disturbance, so
perturbations cannot bypass the initial 50 nm reference restriction.

Editing an idle connected profile invalidates the previous parameter commit;
write and acknowledge the updated parameters before starting another run.
Deployment-related changes require reconnecting. Running experiments lock
parameter editing, and every run stores the profile actually applied.
Loading an older schema-1 profile adds only new disabled/neutral options;
missing original drive or calibration fields still fail validation.

Go's output range is nominally +/-5 V into high impedance, with 200 ohm output
impedance and a maximum 25 mA source/sink current. Direct piezo operation needs
an independent check of voltage rating, polarity, capacitive stability and
current. A necessary capacitive-load bound is `I = C * dV/dt`; for a sine,
`I_peak = 2*pi*f*C*V_peak`. This bound does not establish loop stability or
settling. Do not copy simulation drive settings onto the physical piezo. See
the official [current limit](https://knowledge.liquidinstruments.com/how-much-current-can-moku%3Alab-source-and-sink)
and [output impedance](https://knowledge.liquidinstruments.com/moku-general/output-impedance-of-moku)
references.

## Firmware And Parameter Transactions

The installed Moku 4.3.0a demo exposes two Phasemeter inputs standalone but only
Input A in a three-slot instrument window. This does not prove the real device
has the same limit, and the generic channel API does not establish the opposite.
See [demo evidence](moku-demo-environment.md). Before physical output, require
`validation.independent_phase_channels=true` only after verifying two independent
REF/MEAS phase streams in this exact slot arrangement. The backend explicitly
disables single-input fanout and refuses deployment if channel 2 is rejected.
If dual phase streams are unavailable, this topology must be redesigned before
real use; the numerical controller tests do not resolve that capability.

Compile for the exact device hardware, MokuOS and slot arrangement. The
manifest identifies the bitstream and hash and must agree with the connected
target. The modern custom wrapper/status interface requires a compatible
MokuOS version; see the [wrapper contract](https://apis.liquidinstruments.com/mc/wrapper.html)
and [status registers](https://apis.liquidinstruments.com/mc/statusregs.html).
Copying a source file or changing a manifest does not compile or qualify it.

`laserai/protocol.py` defines the versioned register ABI. A host writes a full
staging bank through an indirect address/data/sequence mailbox and waits for
each write acknowledgment. It then sends a fresh commit ID. FPGA validation
and bank switching occur at a control-cycle boundary; the acknowledged commit
ID must match before start. Partial writes, wrong ABI, timeout, arithmetic
overflow and unmatched acknowledgment must fail. Both algorithms share the
measurement, trajectory, clock and output protection.

Host-side coefficient calculation is configuration work. The per-sample PID,
preview feedforward, disturbance observer and protection execute on the board.
The numerical `control.py` implementation and the fixed-point reference are
offline comparisons. Check quantization range and overflow handling as well as
average numerical error; close nominal outputs do not prove fault handling.

Every run should preserve the complete profile, actual committed parameters,
device serial and firmware, SDK version, bitstream identity and run outcome.
Capture a fault or disconnect as a failed/aborted run, not a successful run with
zero error. Keep original logs when changing the offline analysis.

## Hardware Acceptance Order

1. **Electrical and measurement only.** Leave the drive disconnected. Confirm
   receiver levels and input range, carrier frequency, phase scaling, sign and
   phase wrap in both directions. Exercise carrier loss and a deliberately
   detuned PLL. Measure total phase-to-displacement latency with a known
   modulation. Set `signal_conditioning`, `phase_scale`, `phase_wrap`, `latency`
   and `loss_of_lock` only after saving the corresponding evidence.
2. **Direct-drive limits.** Measure piezo capacitance and check voltage/current,
   source impedance and stability. Verify the stopped voltage and the response
   to reset, missing carrier and communication loss without an attached piezo
   first. Record `direct_drive` only after the physical load is qualified.
3. **Low-amplitude open loop.** Use a bounded board-side test, increase output
   slowly and verify displacement polarity and hard limits. Identify gain,
   time constant and delays. Record `polarity` from this measurement. The
   migration scaffold must not bypass its output gate to obtain this result;
   use a separately qualified bench procedure before enabling its controller.
4. **PID baseline.** Start well below the final amplitude/frequency, tune against
   the identified model and check voltage, slew, tracking-error, arithmetic and
   duration protections. Confirm the FPGA continues at its configured tick
   rate when host rendering is slow or paused; verify the configured autonomous
   duration/communication-loss policy and safe stop.
5. **Preview+ADRC and A/B runs.** Use the same reference, amplitude, offset,
   sample rate, filtering and output limits. Alternate or randomize PID and
   Preview order, reset initial conditions, retain settling exclusions and run
   at least three repeats. Log loss, saturation and faults in addition to error.

There is no claim that Preview always beats PID. Compare tracking at increasing
frequency (including 20 Hz), response to a controlled disturbance and tolerance
to realistic model errors. A phase lead on an ideal exact-model simulation is
not proof of better physical robustness.

For each valid steady interval report `RMSE = sqrt(mean((r-y)^2))`, fitted sine
amplitude ratio `A_y/A_r`, phase lag, output saturation fraction and repetition
spread. Fit the same timestamps and frequency to both signals; use wrapped
phase consistently. Report the 500 nm target as peak amplitude. Exclude only
the configured settling interval, never selectively remove bad cycles. A
20 Hz acceptance result needs measured peak reference >=500 nm, steady RMSE
<=50 nm, verified telemetry timing and no protection faults. Retain the raw data
so a lower RMSE cannot hide saturation or skipped samples.

## Cross-Platform Release Checks

The CI matrix runs Python 3.13 tests and a native PyInstaller build separately
on macOS, Windows and Linux. It is an executable release recipe, not evidence
that those hosted jobs or physical tests have run in the local session.

Use the same Ethernet-connected device for each OS. Test connection,
measurement-only deployment, configuration import/export, missing CLI, wrong
firmware/manifest, write/commit timeout, cable removal and exit during a read.
Use profile and result paths containing Chinese characters and spaces. Test
high-DPI scaling and resizing with real displays in addition to offscreen Qt
tests. Ensure no background worker survives shutdown and that a failed run is
marked as such. USB is an extra check; on Windows install the vendor driver,
and on all platforms retain IPv6 support. Consult the official
[CLI installation](https://apis.liquidinstruments.com/cli/getting-started/install.html)
and [Python connection guide](https://apis.liquidinstruments.com/api/getting-started/starting-python.html).

Build with `python -m PyInstaller --clean --noconfirm moku_host.spec` on each
target OS. The package includes profile templates and SDK Python data, and
excludes legacy VB/broker/CLI binaries. MokuCLI, instrument downloads, device
licenses and a compatible compiled custom bitstream remain external. macOS
signing/notarization, Windows signing and Linux distribution compatibility
require release-specific checks beyond a local developer build.
