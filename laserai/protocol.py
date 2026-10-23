"""Versioned host/FPGA contract. No network calls or host-side live control.

All dimensional values are signed Q16.16; coefficients are signed Q8.24.
The unsigned phase accumulator represents one turn by 2**32 counts.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import Mapping, Sequence

ABI_MAGIC = 0x4C415302
CLOCK_HZ = 31_250_000
CONFIG_WORDS = 47


class Control(IntEnum):
    MAGIC = 0
    ADDRESS = 1
    DATA = 2
    WRITE_SEQUENCE = 3
    COMMIT_ID = 4
    COMMAND_SEQUENCE = 5
    COMMAND = 6
    HEARTBEAT = 7
    STATUS_PAGE = 8


class Command(IntEnum):
    STOP = 0
    START_PID = 1
    START_PREVIEW = 2
    ZERO = 3
    START_OPEN_LOOP = 4


class Fault(IntFlag):
    NONE = 0
    CONFIG = 1
    HEARTBEAT = 2
    SIGNAL = 4
    TRACKING = 8
    DURATION = 16
    ARITHMETIC = 32


# address, fractional bits; None means an integer word.
FIELDS = {
    "tick_divider": (0, None), "duration_ticks": (1, None),
    "heartbeat_timeout_ticks": (2, None), "phase_period_counts": (3, None),
    "nm_per_phase_count": (4, 16), "direction": (5, None),
    "v_min": (6, 16), "v_max": (7, 16), "v_safe": (8, 16),
    "slew_per_tick": (9, 16), "max_abs_error": (10, 16),
    "signal_threshold": (11, None), "signal_timeout_ticks": (12, None),
    "dac_counts_per_volt": (13, 16), "waveform": (14, None),
    "amplitude": (15, 16), "offset": (16, 16), "phase_step": (17, None),
    "phase_initial": (18, None), "plant_delay_ticks": (19, None),
    "sensor_delay_ticks": (20, None), "model_a": (21, 24),
    "model_b": (22, 24), "inverse_b": (23, 24), "inverse_K": (24, 24),
    "kp": (25, 24), "ki_dt": (26, 24), "integral_limit": (27, 16),
    "derivative_a": (28, 24), "derivative_b": (29, 24),
    "observer_a11": (30, 24), "observer_a12": (31, 24),
    "observer_a21": (32, 24), "observer_a22": (33, 24),
    "observer_b1": (34, 24), "observer_b2": (35, 24),
    "duty": (36, 24), "symmetry": (37, 24), "edge_fraction": (38, 24),
    "disturbance_mode": (39, None), "disturbance_amplitude": (40, 16),
    "disturbance_phase_step": (41, None), "disturbance_phase_initial": (42, None),
    "disturbance_start_ticks": (43, None), "disturbance_duration_ticks": (44, None),
    "disturbance_seed": (45, None), "trajectory_seed": (46, None),
}


def signed32(word: int) -> int:
    word = int(word) & 0xFFFFFFFF
    return word - (1 << 32) if word & (1 << 31) else word


def encode_fixed(value: float, fractional_bits: int = 16) -> int:
    if not math.isfinite(value):
        raise ValueError("non-finite fixed-point value")
    result = round(value * (1 << fractional_bits))
    if not -(1 << 31) <= result < (1 << 31):
        raise ValueError(f"value {value} exceeds signed Q{32-fractional_bits}.{fractional_bits}")
    return result


def decode_fixed(word: int, fractional_bits: int = 16) -> float:
    return signed32(word) / (1 << fractional_bits)


def prepare_coefficients(*, gain_nm_per_v: float, tau_s: float, dt_s: float,
                         plant_delay_s: float, sensor_delay_s: float,
                         kp: float, ki: float, kd: float,
                         d_filter_n: float, w0: float,
                         v_max: float) -> dict[str, float | int]:
    """Compile the existing control.py laws into board coefficients.

    Observer input is model minus measurement; correction is res1/K.
    The repeated-pole exponential below equals control.py's augmented expm.
    """
    values = (gain_nm_per_v, tau_s, dt_s, plant_delay_s, sensor_delay_s,
              kp, ki, kd, d_filter_n, w0, v_max)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("controller parameters must be finite")
    if min(gain_nm_per_v, tau_s, dt_s, w0, v_max) <= 0:
        raise ValueError("gain, time constant, dt, observer bandwidth and v_max must be positive")
    if min(plant_delay_s, sensor_delay_s, kp, ki, kd, d_filter_n) < 0:
        raise ValueError("delays and PID parameters must be nonnegative")
    a = math.exp(-dt_s / tau_s)
    b = gain_nm_per_v * (1 - a)
    ew = math.exp(-w0 * dt_s)
    wd = w0 * dt_s
    return {
        "tick_divider": round(CLOCK_HZ * dt_s),
        "plant_delay_ticks": round(plant_delay_s / dt_s),
        "sensor_delay_ticks": round(sensor_delay_s / dt_s),
        "model_a": a, "model_b": b, "inverse_b": 1 / b,
        "inverse_K": 1 / gain_nm_per_v, "kp": kp, "ki_dt": ki * dt_s,
        "integral_limit": v_max,
        "derivative_a": 1 / (1 + d_filter_n * dt_s),
        "derivative_b": kd * d_filter_n / (1 + d_filter_n * dt_s),
        "observer_a11": ew * (1 - wd), "observer_a12": ew * dt_s,
        "observer_a21": -ew * w0 * w0 * dt_s, "observer_a22": ew * (1 + wd),
        "observer_b1": 1 - ew * (1 - wd), "observer_b2": ew * w0 * w0 * dt_s,
    }


def compile_config(values: Mapping[str, float | int | None]) -> list[int]:
    """Validate a complete, flat configuration and encode its 47 words."""
    missing = [key for key in FIELDS if values.get(key) is None]
    if missing:
        raise ValueError("missing FPGA parameters: " + ", ".join(missing))
    words = [0] * CONFIG_WORDS
    for key, (address, bits) in FIELDS.items():
        value = values[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{key} must be a finite number")
        if bits is None:
            if int(value) != value:
                raise ValueError(f"{key} must be an integer")
            word = int(value)
            if key in ("phase_step", "phase_initial", "disturbance_phase_step",
                       "disturbance_phase_initial", "disturbance_seed", "trajectory_seed"):
                if not 0 <= word < (1 << 32):
                    raise ValueError(f"{key} exceeds an unsigned word")
            elif not -(1 << 31) <= word < (1 << 31):
                raise ValueError(f"{key} exceeds a signed word")
        else:
            word = encode_fixed(float(value), bits)
        words[address] = word
    v = values
    checks = [
        (100 <= v["tick_divider"] <= CLOCK_HZ, "tick_divider must be 100..31250000"),
        (v["duration_ticks"] > 0, "duration_ticks must be positive"),
        (v["heartbeat_timeout_ticks"] >= 0, "heartbeat timeout must be nonnegative; zero disables it"),
        (2 <= v["phase_period_counts"] <= 65536, "phase period must be 2..65536 counts"),
        (v["nm_per_phase_count"] > 0, "phase calibration must be positive"),
        (v["direction"] in (-1, 1), "direction must be -1 or 1"),
        (-5 <= v["v_min"] <= v["v_safe"] <= v["v_max"] <= 5 and v["v_min"] < v["v_max"], "drive limits/safe value invalid"),
        (v["slew_per_tick"] > 0 and v["max_abs_error"] > 0, "slew and error limits must be positive"),
        (0 < v["signal_threshold"] <= 32767 and v["signal_timeout_ticks"] > 0, "signal gate invalid"),
        (0 < v["dac_counts_per_volt"] <= 32767, "DAC scale invalid"),
        (v["waveform"] in range(6) and v["amplitude"] >= 0, "invalid waveform or negative amplitude"),
        (0 <= v["duty"] <= 1 and 0 < v["symmetry"] < 1, "duty must be in [0,1] and symmetry in (0,1)"),
        (0 <= v["edge_fraction"] <= min(v["duty"], 1-v["duty"])/2, "edge_fraction exceeds available pulse width"),
        (v["disturbance_mode"] in range(5) and abs(v["disturbance_amplitude"]) <= 5, "invalid disturbance mode/amplitude"),
        (v["disturbance_start_ticks"] >= 0 and v["disturbance_duration_ticks"] >= 0, "negative disturbance interval"),
        (v["disturbance_mode"] == 0 or v["disturbance_duration_ticks"] > 0, "active disturbance needs positive duration"),
        (abs(v["offset"]) + v["amplitude"] < 32768, "trajectory exceeds Q16 range"),
        (0 <= v["plant_delay_ticks"] <= 255 and 0 <= v["sensor_delay_ticks"] <= 255, "delay exceeds 255 samples"),
        (0 < v["model_a"] < 1 and min(v["model_b"], v["inverse_b"], v["inverse_K"]) > 0, "invalid plant coefficients"),
        (min(v["kp"], v["ki_dt"], v["derivative_b"]) >= 0 and 0 <= v["derivative_a"] <= 1 and v["integral_limit"] > 0, "invalid PID coefficients"),
        (max(abs(v["v_min"]), abs(v["v_max"])) * v["dac_counts_per_volt"] <= 32767, "drive range exceeds DAC scale"),
    ]
    for passed, message in checks:
        if not passed:
            raise ValueError(message)
    # Positive physical values must survive fixed-point quantization.
    for key in ("nm_per_phase_count", "slew_per_tick", "max_abs_error", "model_b", "inverse_b", "inverse_K"):
        if words[FIELDS[key][0]] <= 0:
            raise ValueError(f"{key} quantizes to zero")
    if not 0 < words[37] < (1 << 24):
        raise ValueError("symmetry quantizes to an unsupported endpoint")
    if words[38] * 2 > min(words[36], (1 << 24) - words[36]):
        raise ValueError("edge_fraction exceeds quantized pulse width")
    return words


def staging_writes(words: Sequence[int], *, commit_id: int,
                   first_sequence: int = 1) -> list[tuple[int, int]]:
    """Ordered writes; every ADDRESS/DATA pair is latched by WRITE_SEQUENCE.

    Caller waits for status[10] after each sequence, then waits for commit ack.
    Commit IDs and sequences must be nonzero and fresh within a session.
    """
    if len(words) != CONFIG_WORDS or not 0 < commit_id < (1 << 32):
        raise ValueError("complete configuration and nonzero 32-bit commit ID required")
    if not 0 < first_sequence <= (1 << 32) - CONFIG_WORDS:
        raise ValueError("write sequence would wrap")
    writes = [(Control.MAGIC, ABI_MAGIC)]
    for address, value in enumerate(words):
        writes.extend(((Control.ADDRESS, address), (Control.DATA, int(value) & 0xFFFFFFFF),
                       (Control.WRITE_SEQUENCE, first_sequence + address)))
    writes.append((Control.COMMIT_ID, commit_id))
    return writes


@dataclass(frozen=True)
class Status:
    commit_id: int
    state: int
    faults: Fault
    tick: int
    page_data: int
    measurement_nm: float
    reference_nm: float
    voltage_v: float
    residual_nm: float
    write_sequence: int
    command_sequence: int
    flags: int = 0
    elapsed_ticks: int = 0
    commit_seen: int = 0

    @property
    def zeroed(self) -> bool:
        return bool(self.flags & 4)

    @property
    def configured(self) -> bool:
        return bool(self.flags & 2)


def decode_status(words: Sequence[int]) -> Status:
    if len(words) < 12 or (int(words[0]) & 0xFFFFFFFF) != ABI_MAGIC:
        raise ValueError("FPGA ABI mismatch or incomplete status")
    return Status(int(words[1]) & 0xFFFFFFFF, int(words[2]), Fault(words[3]),
                  int(words[4]) & 0xFFFFFFFF, signed32(words[5]),
                  *(decode_fixed(v) for v in words[6:10]),
                  int(words[10]) & 0xFFFFFFFF, int(words[11]) & 0xFFFFFFFF,
                  int(words[12]) if len(words) > 12 else 0,
                  int(words[13]) if len(words) > 13 else 0,
                  int(words[14]) if len(words) > 14 else 0)


class FixedController:
    """Bit-arithmetic reference for HDL comparisons, used offline only."""
    def __init__(self, words: Sequence[int]):
        if len(words) != CONFIG_WORDS:
            raise ValueError("47 configuration words required")
        self.c = [signed32(v) for v in words]
        self.integral = self.derivative = self.previous_y = self.model_y = 0
        self.res1 = self.res2 = 0
        self.last_v = self.c[8]
        self.first = True
        self.sensor_delay = [0] * self.c[20]

    @staticmethod
    def mul(x: int, coefficient: int) -> int:
        result = (x * coefficient) >> 24
        if not -(1 << 31) <= result < (1 << 31):
            raise OverflowError("FPGA fixed-point arithmetic overflow")
        return result

    def update(self, algorithm: str, reference_nm: float, measurement_nm: float,
               preview_nm: tuple[float, float] | None = None, *,
               disturbance_v: float = 0.0) -> float:
        c, m = self.c, self.mul
        y, r = encode_fixed(measurement_nm), encode_fixed(reference_nm)
        if algorithm == "open_loop":
            requested = m(r, c[24])
        elif algorithm == "pid":
            self.integral = max(-c[27], min(c[27], self.integral + m(r-y, c[26])))
            self.derivative = (0 if self.first else m(self.derivative, c[28]) + m(self.previous_y-y, c[29]))
            requested = m(r-y, c[25]) + self.integral + self.derivative
        elif algorithm == "preview":
            if preview_nm is None:
                raise ValueError("preview requires two future references")
            rd, rd1 = map(encode_fixed, preview_nm)
            uff = m(rd1-m(rd, c[21]), c[23])
            self.model_y = m(self.model_y, c[21]) + m(self.last_v, c[22])
            aligned = self.model_y
            if self.sensor_delay:
                aligned = self.sensor_delay.pop(0)
                self.sensor_delay.append(self.model_y)
            err = aligned - y
            r1 = m(self.res1, c[30]) + m(self.res2, c[31]) + m(err, c[34])
            r2 = m(self.res1, c[32]) + m(self.res2, c[33]) + m(err, c[35])
            self.res1, self.res2 = r1, r2
            requested = uff + m(r1, c[24])
        else:
            raise ValueError("algorithm must be pid, preview or open_loop")
        self.first = False
        self.previous_y = y
        requested += encode_fixed(disturbance_v)
        requested = max(c[6], min(c[7], requested))
        self.last_v += max(-c[9], min(c[9], requested - self.last_v))
        return self.last_v / 65536


def _noise_q24(index: int, seed: int) -> int:
    value = (index + seed) & 0xFFFFFFFF
    value ^= value >> 16
    value = (value * 0x7FEB352D) & 0xFFFFFFFF
    value ^= value >> 15
    value = (value * 0x846CA68B) & 0xFFFFFFFF
    value ^= value >> 16
    high = value >> 16
    return (high - 65536 if high & 32768 else high) << 9


def _gate_q24(phase: int, duty: int, edge: int) -> int:
    one, p = 1 << 24, phase >> 8
    if duty == 0:
        return 0
    if duty == one:
        return one
    if edge == 0:
        return one if p < duty else 0
    if p < edge:
        return one // 2 + (p * one) // (2 * edge)
    if p >= one - edge:
        return ((p - (one - edge)) * one) // (2 * edge)
    if p < duty - edge:
        return one
    if p < duty + edge:
        return one - ((p - (duty - edge)) * one) // (2 * edge)
    return 0


def _wave_q24(waveform: int, phase: int, index: int, words: Sequence[int]) -> int:
    one, p = 1 << 24, phase >> 8
    if waveform == 0:
        return one
    if waveform == 1:
        return round(math.sin((phase >> 24) * 2 * math.pi / 256) * one)
    if waveform in (2, 4):
        gate = _gate_q24(phase, int(words[36]), int(words[38]))
        return gate * 2 - one if waveform == 2 else gate
    if waveform == 3:
        symmetry = int(words[37])
        return (-one + (2*p*one)//symmetry if p < symmetry else
                one - (2*(p-symmetry)*one)//(one-symmetry))
    return _noise_q24(index, int(words[46]))


def board_reference(words: Sequence[int], tick: int) -> tuple[float, tuple[float, float]]:
    """Bit-exact board trajectory, including lookahead and seeded tick noise."""
    def at(index: int) -> float:
        phase = (int(words[18]) + int(words[17]) * index) & 0xFFFFFFFF
        wave = _wave_q24(int(words[14]), phase, index, words)
        return (signed32(words[16]) + ((signed32(words[15]) * wave) >> 24)) / 65536
    return at(tick), (at(tick + words[19]), at(tick + words[19] + 1))


def board_disturbance(words: Sequence[int], tick: int) -> float:
    """Board voltage disturbance, added before the common clamp and slew limiter."""
    mode, start, duration = (int(words[i]) for i in (39, 43, 44))
    if mode == 0 or not start <= tick < start + duration:
        return 0.0
    active_tick = tick - start
    phase = (int(words[42]) + int(words[41]) * active_tick) & 0xFFFFFFFF
    if mode == 1:
        wave = 1 << 24
    elif mode == 2:
        wave = round(math.sin((phase >> 24) * 2 * math.pi / 256) * (1 << 24))
    elif mode == 3:
        wave = (1 << 24) if phase < (1 << 31) else -(1 << 24)
    else:
        wave = _noise_q24(active_tick, int(words[45]))
    return ((signed32(words[40]) * wave) >> 24) / 65536


@dataclass(frozen=True)
class TelemetryFrame:
    tick: int
    measurement_nm: float
    reference_nm: float
    voltage_v: float
    residual_nm: float
    commit_id: int
    state: int
    faults: Fault


def decode_telemetry_frames(symbols: Sequence[int]) -> list[TelemetryFrame]:
    """Decode word-aligned OutputB symbols; not uncalibrated logger voltages.

    Recover one symbol per FPGA-held word before calling. Header/checksum permit
    resynchronization after loss. Tick gaps must disqualify full-rate metrics.
    """
    result = []
    words = [int(v) & 0xFFFF for v in symbols]
    index = 0
    while index + 16 <= len(words):
        frame = words[index:index + 16]
        check = 0
        for word in frame:
            check ^= word
        if frame[0] != 0xA55A or frame[1] != 1 or check:
            index += 1
            continue
        def u32(at: int) -> int:
            return (frame[at] << 16) | frame[at + 1]
        result.append(TelemetryFrame(u32(2), *(decode_fixed(u32(at)) for at in (4, 6, 8, 10)),
                                     u32(12), frame[14] & 7, Fault((frame[14] >> 3) & 63)))
        index += 16
    return result
