"""Stop-and-capture tilt experiment. Translation is deliberately NOT estimated."""
from collections import deque, OrderedDict
import math
import struct


def angle_delta(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def rotate(point, roll, pitch, yaw):
    """Rz(yaw) Ry(pitch) Rx(roll), radians, right-handed axes."""
    x, y, z = point
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    y, z = cr * y - sr * z, sr * y + cr * z
    x, z = cp * x + sp * z, -sp * x + cp * z
    return (cy * x - sy * y, sy * x + cy * y, z)


class TiltCloud:
    def __init__(self, voxel_size=0.03, max_points=50000, settle_time=0.6,
                 max_rate_deg=5.0, timeout=0.5, pair_gap=0.15,
                 use_heading=False, roll_sign=1, pitch_sign=1,
                 mount_rpy_deg=(0.0, 0.0, 0.0), sensor_height=0.0,
                 min_status=0, max_range=8.0):
        values = (voxel_size, settle_time, max_rate_deg, timeout, pair_gap, max_range)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("positive finite limits required")
        if (not isinstance(max_points, int) or isinstance(max_points, bool)
                or max_points < 1 or roll_sign not in (-1, 1)
                or pitch_sign not in (-1, 1) or min_status not in (0, 1, 2, 3)):
            raise ValueError("invalid capacity, signs or status threshold")
        if not math.isfinite(sensor_height) or len(mount_rpy_deg) != 3 or not all(
                math.isfinite(v) for v in mount_rpy_deg):
            raise ValueError("invalid mounting parameters")
        self.voxel_size, self.max_points = voxel_size, max_points
        self.settle_time, self.max_rate = settle_time, max_rate_deg
        self.timeout, self.pair_gap, self.max_range = timeout, pair_gap, max_range
        self.use_heading, self.roll_sign, self.pitch_sign = use_heading, roll_sign, pitch_sign
        self.mount = tuple(math.radians(v) for v in mount_rpy_deg)
        self.sensor_height, self.min_status = sensor_height, min_status
        self.history = deque(maxlen=100)
        self.voxels = OrderedDict()
        self.tick = self.transport_tick = self.yaw_zero = None
        self.input_valid = False
        self.reason = "waiting for PNI"
        self.accepted_scans = 0

    def clear(self):
        self.voxels.clear()
        self.yaw_zero = None
        self.accepted_scans = 0

    def ingest(self, data, now):
        try:
            if not math.isfinite(now):
                raise ValueError("invalid receive time")
            p = data["pni"]
            tick, update = data["tick"], p["update_tick"]
            if not all(isinstance(v, int) and not isinstance(v, bool)
                       and 0 <= v <= 0xFFFFFFFF for v in (tick, update)):
                raise ValueError("invalid MCU timestamps")
            if self.transport_tick is not None and (
                    (tick - self.transport_tick) & 0xFFFFFFFF) > 0x80000000:
                self.history.clear()
                self.tick = None
                self.clear()  # Never combine a new MCU compass zero with old points.
            self.transport_tick = tick
            age = ((tick - update) & 0xFFFFFFFF) / 1000.0
            status = p["status"]
            if p["valid"] is not True or age > self.timeout:
                raise ValueError("PNI invalid or stale")
            if (not isinstance(status, int) or isinstance(status, bool)
                    or not self.min_status <= status <= 3):
                raise ValueError("PNI below quality threshold")
            values = tuple(float(p[key]) for key in ("roll", "pitch", "heading"))
            if not all(math.isfinite(v) for v in values) or not (
                    abs(values[0]) <= 90 and abs(values[1]) <= 90 and abs(values[2]) <= 180):
                raise ValueError("PNI angle out of range")
            if update == self.tick:
                return False  # A cached value never extends its measurement age.
            stamp = now - age
            if self.history and stamp <= self.history[-1][0]:
                raise ValueError("PNI timestamps out of order")
            self.tick = update
            self.history.append((stamp, values, status))
            self.input_valid = True
            self.reason = "PNI ready; waiting for steady scan"
            return True
        except (KeyError, TypeError, ValueError) as exc:
            self.input_valid = False
            self.reason = str(exc)
            return False

    def pose_for_scan(self, start, duration, now):
        end = start + duration
        if not all(math.isfinite(v) for v in (start, duration, now)) or (
                start <= 0 or duration < 0 or duration > self.timeout
                or end > now + 0.05 or now - start > self.timeout):
            self.reason = "scan timestamp missing, stale or future"
            return None
        if not self.input_valid or not self.history or now - self.history[-1][0] > self.timeout:
            self.reason = "waiting for fresh PNI"
            return None
        # Require approximately settle_time of unique, steady measurements
        # spanning the scan. Heading is checked even when it is not used.
        window = [s for s in self.history if end - self.settle_time <= s[0] <= end + self.pair_gap]
        if len(window) < 3 or window[-1][0] - window[0][0] < self.settle_time * 0.75:
            self.reason = "waiting for settling history"
            return None
        if abs(window[-1][0] - end) > self.pair_gap:
            self.reason = "scan/PNI time mismatch"
            return None
        for a, b in zip(window, window[1:]):
            dt = b[0] - a[0]
            if dt > self.pair_gap * 2 or any(
                    abs(angle_delta(y, x)) / dt > self.max_rate for x, y in zip(a[1], b[1])):
                self.reason = "moving or PNI jump; hold still"
                return None
        sample = min(window, key=lambda s: abs(s[0] - (start + duration / 2)))
        if abs(sample[0] - (start + duration / 2)) > self.pair_gap:
            self.reason = "scan/PNI time mismatch"
            return None
        if self.yaw_zero is None:
            self.yaw_zero = sample[1][2]
        return sample[1]

    def add_scan(self, ranges, angle_min, angle_increment, range_min, range_max,
                 start, duration, now):
        if not all(math.isfinite(v) for v in (angle_min, angle_increment, range_min, range_max)) or (
                range_min < 0 or range_max <= range_min or angle_increment == 0):
            self.reason = "invalid scan geometry"
            return 0
        pose = self.pose_for_scan(start, duration, now)
        if pose is None:
            return 0
        roll, pitch, heading = pose
        angles = (math.radians(roll * self.roll_sign), math.radians(pitch * self.pitch_sign),
                  math.radians(angle_delta(heading, self.yaw_zero)) if self.use_heading else 0.0)
        count = 0
        for i, distance in enumerate(ranges):
            if not math.isfinite(distance) or not range_min <= distance <= min(range_max, self.max_range):
                continue
            theta = angle_min + i * angle_increment
            point = rotate((distance * math.cos(theta), distance * math.sin(theta), 0.0), *self.mount)
            x, y, z = rotate(point, *angles)
            z += self.sensor_height
            key = tuple(math.floor(v / self.voxel_size) for v in (x, y, z))
            # One point per voxel; newest observation replaces the old one.
            self.voxels[key] = (float(x), float(y), float(z))
            self.voxels.move_to_end(key)
            if len(self.voxels) > self.max_points:
                self.voxels.popitem(last=False)
            count += 1
        if count:
            self.accepted_scans += 1
        self.reason = "capturing steady scan" if count else "no valid ranges"
        return count

    def packed_points(self):
        return b"".join(struct.pack("<fff", *point) for point in self.voxels.values())
