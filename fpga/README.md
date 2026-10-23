# Moku:Go fixed-point control source

This directory contains synthesizable SystemVerilog source and executable HDL
tests, **not a compiled, hardware-qualified bitstream**. Moku Compile synthesis,
slot resource fit, timing closure, phase scaling/latency, external signal
conditioning and physical drive limits still require validation on the device.

Target: Moku:Go, three-slot Multi-Instrument Mode, custom instrument in slot 2,
31.25 MHz clock, MokuOS >=4.1.1. Compile `CustomInstrument.sv`,
`laser_control.sv` and `sine_lut.svh` together. Do not include `testbench.sv` in
the deployment project. The clock rate is a build-time assumption; a different
slot layout needs a different build and manifest.

Official interfaces:
- https://apis.liquidinstruments.com/mc/examples/template.html
- https://apis.liquidinstruments.com/mc/wrapper.html
- https://apis.liquidinstruments.com/mc/slots.html
- https://apis.liquidinstruments.com/mc/io.html

## Signal path and control

- InputA: REF phase; InputB: MEAS phase. Both are signed 16-bit routed samples.
- InputC/D: raw ADC REF/MEAS respectively, used for per-tick signal-presence gates.
- OutputA: actuator drive. Conversion uses the explicitly calibrated DAC scale.
- OutputB: selected monitor or coherent telemetry frames (Control[9]).

Both phase streams are individually unwrapped every FPGA clock. Their difference
is accumulated in nm using `phase_period_counts`, `nm_per_phase_count` and
`direction`. The period **must** match the Phasemeter routing scale, including any
configured phase wrapping. Adjacent valid samples must move by less than half
a phase period. A zero command defines displacement zero; starting requires a
committed configuration, observed input signals, a valid zero and no fault.

Raw amplitude gating does not certify Phasemeter PLL lock. A valid optical signal
can still produce an invalid phase estimate. Bench validation of lock loss,
phase discontinuities, acquisition, phase noise and latency is a deployment
prerequisite; there is no fabricated lock status in this implementation.

DC, sine, square, asymmetric triangle/ramp, pulse and seeded noise references
are synthesized on the board. Periodic waves use a 32-bit phase DDS; sine uses
a 256-entry LUT. The reference and both look-ahead samples use the same waveform
function, including deterministic noise. Include LUT quantization in performance assessment. Control defaults
to 1 kHz (divider 31250); all coefficients are prepared for that exact rate.

PID preserves the existing filtered derivative-on-measurement and bounded
integral contribution. Preview preserves exact FOPDT inversion, the delayed
reference model, the model-minus-measurement residual observer and **+res1/K**
trim. `plant_delay_ticks` selects the preview horizon; `sensor_delay_ticks`
delays the reference model. Open-loop mode converts the board reference via 1/K.
Host-side helpers in `laserai/protocol.py` are used for offline verification,
not to issue live voltage commands.

## Register ABI v2

Controls and statuses are sixteen 32-bit words. Numeric dimensions (nm, V) use
signed Q16.16. Coefficients use signed Q8.24. Phase DDS words are unsigned.

| Control | Meaning |
| --- | --- |
| 0 | ABI magic `0x4c415302` |
| 1, 2 | Staging address and encoded data |
| 3 | New write sequence: latches the address/data; await Status[10] |
| 4 | New nonzero commit ID; await Status[1] |
| 5, 6 | New command sequence and command value |
| 7 | Optional heartbeat counter |
| 8 | Active configuration readback address |
| 9 | OutputB: 0 measurement, 1 reference, 2 error, 3 residual, 4 frames |

Command values: 0 stop, 1 PID, 2 Preview+ADRC, 3 zero, 4 open-loop.
Commands are acknowledged at a control tick through Status[11]; acknowledgment
does not imply successful start. Check state/faults and zero-valid flags.

| Status | Meaning |
| --- | --- |
| 0, 1 | ABI magic, accepted commit ID |
| 2 | State: 0 disabled, 1 PID, 2 Preview, 3 ramp, 4 open-loop |
| 3 | Fault bits: config=1, heartbeat=2, signal=4, tracking=8, duration=16, arithmetic=32 |
| 4, 5 | Free-running tick counter, selected active configuration word |
| 6..9 | Measurement nm, reference nm, drive V, observer res1 nm (Q16.16) |
| 10, 11 | Observed write sequence, observed command sequence |
| 12 | Flags: bit2 zero valid, bit1 configuration valid, bit0 current signal peaks |
| 13 | Elapsed ticks of the current/last experiment |
| 14 | Last observed commit ID, including a rejected transaction |

Stage all 47 fields before every commit; successful commit clears the staging
presence mask. Write address, then data, then a fresh write sequence and await
acknowledgment for each word. Commit only while disabled. Missing fields,
invalid physical ranges and invalid plant coefficients reject the commit and
retain the active configuration. Configuration readback must match all words.
Configuration faults can be cleared by successful zeroing while disabled.
Changing ABI magic or register values alone never enables output.

The definitive address/format schema is `laserai.protocol.FIELDS`. Addresses
0..20 configure timing, limits, phase conversion, trajectory and delays;
21..24 hold the FOPDT coefficients and inverses; 25..29 hold PID coefficients;
30..35 hold the residual observer matrix and input vector. Additional ABI v2
words are:

| Address | Name | Format |
| --- | --- | --- |
| 36 | duty | Q8.24 fraction in [0, 1] |
| 37 | symmetry | Q8.24 fraction strictly between 0 and 1 |
| 38 | edge_fraction | Q8.24 half-transition fraction of a cycle |
| 39 | disturbance_mode | 0 off, 1 step, 2 sine, 3 square, 4 noise |
| 40 | disturbance_amplitude | Q16.16 signed V, [-5, 5] |
| 41, 42 | disturbance_phase_step, disturbance_phase_initial | unsigned turns/2**32 |
| 43, 44 | disturbance_start_ticks, disturbance_duration_ticks | nonnegative ticks |
| 45, 46 | disturbance_seed, trajectory_seed | unsigned 32-bit |

Waveform codes are 0 DC, 1 sine, 2 square, 3 ramp, 4 pulse, 5 noise. Duty controls
the positive/high interval of square and pulse. Square spans offset +/- amplitude;
pulse spans offset to offset + amplitude. With nonzero edge fraction `e`, linear
transitions are centered on phase zero and duty, each occupying `2e` of a cycle.
The constraint is `0 <= e <= min(duty,1-duty)/2`. A user-facing full edge duration
maps to `e = edge_time_s * frequency_hz / 2`. Duty endpoints require zero edge.
Ramp rises from -amplitude to +amplitude over `symmetry` of a cycle, then falls.

Noise is a stateless hash of tick index plus seed modulo 2**32:
`x ^= x>>16; x *= 0x7feb352d; x ^= x>>15; x *= 0x846ca68b; x ^= x>>16`.
Multiplications wrap at 32 bits; the signed high sixteen bits divided by 32768
give a value in [-1,1). The same index always produces the same reference,
allowing Preview to evaluate exact future noise samples. Noise ignores DDS phase.

Disturbance is active only for `start <= tick < start + duration`; active modes
require positive duration. Its phase/noise index starts at zero at the beginning
of that interval. A square disturbance has 50% duty and abrupt edges. The signed
voltage is added after the controller calculation and before the single shared
output clamp and slew limiter; it never bypasses protection or persists in RAMP.
The Preview reference model uses the previously applied final voltage, including
this known injection. Therefore this waveform is not an unmeasured exogenous
plant disturbance and cannot alone establish ADRC disturbance-rejection advantage.

All fields must be
provided; unknown hardware parameters must not be replaced by demo values.

## Autonomous protections

Boot starts disabled with zero DAC output. A bounded duration is mandatory.
Heartbeat timeout 0 disables the host heartbeat policy, so a normal experiment
continues independently of host display, polling or disconnection and finishes
on the FPGA. A nonzero timeout explicitly enables ramp-down on heartbeat loss.
Signal loss, excessive absolute tracking error, fixed-point overflow, explicit
stop and duration expiry cause a slew-limited ramp to the configured safe
voltage, then disable the controller. The safe voltage remains on the DAC;
"disabled" does not mean a physical high-impedance disconnect.

The initial accepted configuration sets safe voltage while disabled. Choose a
physically safe value and validate output calibration before deployment.

## Framed telemetry

Control[9]=4 emits sixteen held signed-16-bit symbols per control period. Each
symbol lasts `floor(tick_divider/16)` FPGA clocks; the last symbol also consumes
the leftover clocks. At 1 kHz this is approximately 16 kwords/s. Fields are:

`A55A, 0001, tick_hi, tick_lo, y_hi, y_lo, r_hi, r_lo, v_hi, v_lo,
res1_hi, res1_lo, commit_hi, commit_lo, faults[5:0]<<3|state, xor_checksum`

The telemetry format remains version 1 (unchanged field layout) independently
of register ABI v2. The frame is captured one FPGA clock after the control tick, so all fields are
coherent. Values use Q16.16 and high-word-first ordering. The XOR of all sixteen
words must be zero. `decode_telemetry_frames` resynchronizes word-aligned data
and validates the header/version/checksum. Consumers must detect missing tick
IDs and reject incomplete recordings for full-rate A/B metrics.

Actual Datalogger voltage scaling, adequate oversampling, and recovery of held
symbol boundaries **have not been validated on hardware**. The decoder expects
already aligned integer symbols, not analog voltage samples. Status snapshots
remain a display/diagnostic channel and are not eligible as complete 1 kHz
performance recordings. In monitor modes 0..3, OutputB contains integer nm.

## Run verification

```sh
python -m pytest tests/test_protocol.py -v
iverilog -g2012 -I fpga -s CustomInstrument -o /tmp/laserai-wrapper fpga/laser_control.sv fpga/CustomInstrument.sv
```

The pytest suite generates numeric vectors and runs the standalone testbench
through Icarus Verilog (`iverilog` and `vvp`). If the tools are absent the HDL
cases skip explicitly. Coverage includes all six trajectories crossed with all
five disturbance modes and all three algorithms (90 HDL runs), coherent frame
decode, boot, partial/invalid commits, phase wrap, zero, autonomous duration,
signal-loss ramp-down, optional heartbeat, displacement overflow and refusal to
start after overflow. Separate tests compare prepared observer coefficients and
fixed-point output against the existing independent `control.py` numeric laws.
