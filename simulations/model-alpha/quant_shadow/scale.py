"""Declared filtered-FIFO range semantics and ideal 16-bit quantization.

No bus, estimator, health policy, calibration, or physical sample-time mapping.
The scale is the Bosch example's range/32768 convention, evaluated in binary64.
"""
from dataclasses import dataclass
import hashlib
import json
import math

G0 = 9.80665
MIN_COUNT, MAX_COUNT = -32768, 32767
ACC_G = (2, 4, 8, 16)
GYR_DPS = (2000, 1000, 500, 250, 125)


def integer(value, lo, hi, label):
    if type(value) is not int or not lo <= value <= hi:
        raise ValueError(label)
    return value


def finite(value, label):
    if type(value) not in (int, float):
        raise ValueError(label)
    try:
        out = float(value)
    except (OverflowError, ValueError):
        raise ValueError(label) from None
    if not math.isfinite(out):
        raise ValueError(label)
    return out


@dataclass(frozen=True)
class RangeProfile:
    """Raw bytes are declared model inputs, never a board readback claim.

    Selected subset: ACC_RANGE, GYR_RANGE, FIFO_DOWNS. No full device
    configuration validation. Pre-filtered/OIS paths are deliberately rejected.
    """
    epoch: int
    registers: tuple
    source_path: str = 'filtered_fifo'

    def __post_init__(self):
        integer(self.epoch, 1, (1 << 63) - 1, 'epoch')
        if self.source_path != 'filtered_fifo' or type(self.registers) is not tuple:
            raise ValueError('selected filtered FIFO source required')
        if any(type(p) is not tuple or len(p) != 2 for p in self.registers):
            raise ValueError('register pairs')
        for k, v in self.registers:
            integer(k, 0, 255, 'register address')
            integer(v, 0, 255, 'register byte')
        r = dict(self.registers)
        if len(self.registers) != 3 or set(r) != {0x41, 0x43, 0x45}:
            raise ValueError('exact selected register subset required')
        if r[0x41] & ~3 or r[0x43] & ~15 or (r[0x43] & 7) > 4:
            raise ValueError('reserved range bits/code')
        if r[0x45] & 0x88 != 0x88:
            raise ValueError('pre-filtered FIFO source unsupported')
        if self.registers != tuple(sorted(self.registers)):
            raise ValueError('canonical register order')

    @property
    def acc_scale(self):
        return ACC_G[dict(self.registers)[0x41]] * G0 / 32768

    @property
    def gyr_scale(self):
        return math.radians(GYR_DPS[dict(self.registers)[0x43] & 7]) / 32768

    def description(self):
        r = dict(self.registers)
        return dict(epoch=self.epoch, register_bytes={f'0x{k:02x}': v for k, v in self.registers},
                    source_path=self.source_path, sensor_frame='unmapped_sensor_xyz',
                    range_g=ACC_G[r[0x41]], range_dps=GYR_DPS[r[0x43] & 7],
                    ois_range_bit=(r[0x43] >> 3) & 1,
                    accel_downsample_factor=1 << ((r[0x45] >> 4) & 7),
                    gyro_downsample_factor=1 << (r[0x45] & 7),
                    accel_lsb_m_s2=self.acc_scale, gyro_lsb_rad_s=self.gyr_scale,
                    gravity_convention_m_s2=G0, resolution_bits=16,
                    scaling_convention='Bosch_example_range_divided_by_32768_binary64',
                    configuration_physically_qualified=False, cadence_qualified=False)

    @property
    def fingerprint(self):
        b = json.dumps(self.description(), sort_keys=True, allow_nan=False).encode()
        return hashlib.sha256(b).hexdigest()


def profile(epoch, acc_range_byte, gyro_range_byte, fifo_downs_byte, *, source_path='filtered_fifo'):
    return RangeProfile(epoch, ((0x41, acc_range_byte), (0x43, gyro_range_byte), (0x45, fifo_downs_byte)), source_path)


def decode_axis(count, scale):
    integer(count, MIN_COUNT, MAX_COUNT, 'signed16 count')
    s = finite(scale, 'scale')
    if s <= 0:
        raise ValueError('scale')
    result = count * s
    if not math.isfinite(result):
        raise ValueError('SI overflow')
    return result


def quantize_axis(value, scale):
    """Nearest-even then signed16 clip; illustrative ADC, not a chip model.

    Error bound = distance to representable SI interval + half an LSB,
    plus an explicit floating arithmetic allowance. Rail occupancy alone
    cannot distinguish an exact endpoint from an out-of-range input.
    """
    x, s = finite(value, 'ideal SI input'), finite(scale, 'scale')
    if s <= 0 or not math.isfinite(MAX_COUNT * s) or not math.isfinite(MIN_COUNT * s):
        raise ValueError('scale')
    lo, hi = MIN_COUNT * s, MAX_COUNT * s
    # Bounding before division avoids overflow for large finite SI inputs.
    bounded = min(hi, max(lo, x))
    count = max(MIN_COUNT, min(MAX_COUNT, round(bounded / s)))
    reconstructed = decode_axis(count, s)
    distance = max(lo - x, x - hi, 0.0)
    allowance = 8 * max(math.ulp(x), math.ulp(reconstructed), math.ulp(s))
    bound = distance + 0.5 * s + allowance
    error = abs(x - reconstructed)
    if not all(math.isfinite(v) for v in (bound, error)):
        raise ValueError('error bound overflow')
    return dict(count=count, reconstructed_SI=reconstructed, absolute_error_SI=error,
                bound_SI=bound, arithmetic_allowance_SI=allowance,
                input_outside_representable_interval=x < lo or x > hi,
                at_raw_rail=count in (MIN_COUNT, MAX_COUNT),
                physical_saturation_proven=False)


def convert_pair(p, accel_counts, gyro_counts, *, epoch, sequence, read_clock_s,
                 provenance='manufactured'):
    if type(p) is not RangeProfile:
        raise ValueError('range profile required')
    integer(epoch, 1, (1 << 63) - 1, 'record epoch')
    integer(sequence, 1, (1 << 63) - 1, 'record sequence')
    if epoch != p.epoch:
        raise ValueError('configuration epoch mismatch')
    if provenance not in ('manufactured', 'existing_simulation_replay'):
        raise ValueError('provenance')
    read_s = finite(read_clock_s, 'declared read clock')
    if read_s < 0:
        raise ValueError('declared read clock')
    for v in (accel_counts, gyro_counts):
        if type(v) not in (list, tuple) or len(v) != 3:
            raise ValueError('three sensor axes required')
        for c in v:
            integer(c, MIN_COUNT, MAX_COUNT, 'signed16 count')
    return dict(epoch=epoch, sequence=sequence, range_fingerprint=p.fingerprint,
                raw_accel_counts=list(accel_counts), raw_gyro_counts=list(gyro_counts),
                accel_m_s2=[decode_axis(c, p.acc_scale) for c in accel_counts],
                gyro_rad_s=[decode_axis(c, p.gyr_scale) for c in gyro_counts],
                units=['m/s^2', 'rad/s'], frame='unmapped_sensor_xyz',
                raw_rails=dict(accel=[c in (MIN_COUNT, MAX_COUNT) for c in accel_counts],
                               gyro=[c in (MIN_COUNT, MAX_COUNT) for c in gyro_counts]),
                provenance=provenance, original_declared_read_clock_s=read_s,
                acquisition_s=None, calibrated_SI_accuracy_qualified=False,
                factory_cross_axis_compensation_applied=False, mount_alignment_applied=False,
                hardware_driver_executed=False, configuration_physically_qualified=False,
                conversion_freshness_admitted=False, usable_for_feedback=False,
                consumer_reset_authorized=False)
