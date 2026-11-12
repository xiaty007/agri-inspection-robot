"""Experimental yaw-only filter. Does not estimate position or publish TF."""
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def constrain_to_short_arc(estimate, first, second):
    """Project onto the shortest arc, not the numeric min/max across +/-pi."""
    span = wrap(second - first)
    correction = wrap(estimate - first)
    return wrap(first + min(max(correction, min(0.0, span)), max(0.0, span)))


class YawFusion:
    def __init__(self, sign=1, min_status=0, timeout=0.5, pair_gap=0.25,
                 pni_std_deg=8.0, drift_std_deg=2.0, max_innovation_deg=30.0):
        if sign not in (-1, 1) or min_status not in (0, 1, 2, 3):
            raise ValueError("invalid heading sign or quality threshold")
        if not all(math.isfinite(v) and v > 0 for v in
                   (timeout, pair_gap, pni_std_deg, drift_std_deg, max_innovation_deg)):
            raise ValueError("filter parameters must be positive and finite")
        if max_innovation_deg >= 180:
            raise ValueError("disagreement limit must be below 180 degrees")
        self.sign, self.min_status = sign, min_status
        self.timeout, self.pair_gap = timeout, pair_gap
        self.r = math.radians(pni_std_deg) ** 2
        self.q = math.radians(drift_std_deg) ** 2
        self.max_innovation = math.radians(max_innovation_deg)
        self.pni = None
        self.last_tick = self.last_transport_tick = self.used_tick = None
        self.offset = self.previous_yaw = self.previous_stamp = self.estimate = None
        self.variance = self.r
        self.reason = "waiting for valid PNI"

    def ingest_pni(self, data, now):
        """Use MCU age and unique measurement tick; receive time is approximate."""
        try:
            if not math.isfinite(now):
                raise ValueError("invalid receive time")
            pni = data["pni"]
            tick, update = data["tick"], pni["update_tick"]
            if not all(isinstance(v, int) and not isinstance(v, bool) and
                       0 <= v <= 0xFFFFFFFF for v in (tick, update)):
                raise ValueError("invalid MCU timestamps")
            if self.last_transport_tick is not None and (
                    (tick - self.last_transport_tick) & 0xFFFFFFFF) > 0x80000000:
                # MCU restarted: its new compass zero may differ.
                self.pni = None
                self.last_tick = self.used_tick = self.offset = None
                self.reason = "MCU restarted; re-aligning"
            self.last_transport_tick = tick
            age = ((tick - update) & 0xFFFFFFFF) / 1000.0
            heading, status = float(pni["heading"]), pni["status"]
            if pni["valid"] is not True or age > self.timeout:
                raise ValueError("PNI invalid or stale")
            if (not isinstance(status, int) or isinstance(status, bool)
                    or not self.min_status <= status <= 3):
                raise ValueError("PNI below quality threshold")
            if not math.isfinite(heading) or not -180 <= heading <= 180:
                raise ValueError("invalid PNI heading")
            if update == self.last_tick:
                return False
            self.last_tick = update
            self.pni = {"yaw": self.sign * math.radians(heading),
                        "stamp": now - age, "tick": update, "status": status}
            self.reason = "PNI ready"
            return True
        except (KeyError, TypeError, ValueError) as exc:
            self.pni = None
            self.reason = str(exc)
            return False

    def update_odom(self, yaw, stamp, now):
        if not all(math.isfinite(v) for v in (yaw, stamp, now)):
            raise ValueError("non-finite odometry")
        if stamp <= 0 or now - stamp > self.timeout or stamp - now > 0.1:
            raise ValueError("odometry timestamp missing, stale or future")
        dt = 0 if self.previous_stamp is None else stamp - self.previous_stamp
        if self.previous_stamp is not None and dt <= 0:
            if dt < -self.timeout:
                self.previous_stamp = self.previous_yaw = self.offset = self.estimate = None
                self.used_tick = None
            else:
                return None
        delta = 0 if self.previous_yaw is None else wrap(yaw - self.previous_yaw)
        reset = self.estimate is None or dt > self.timeout or abs(delta) > math.pi / 2
        if reset:
            self.estimate = yaw
            self.variance = self.r
            self.offset = self.used_tick = None
        else:
            self.estimate = wrap(self.estimate + delta)
            self.variance += self.q * dt
        self.previous_yaw, self.previous_stamp = yaw, stamp
        sample = self.pni
        fresh = sample is not None and -0.1 <= now - sample["stamp"] <= self.timeout
        paired = fresh and abs(stamp - sample["stamp"]) <= self.pair_gap
        used, innovation, gain = False, None, 0.0
        reason = self.reason if not fresh else "prediction only"
        disagreement, clamped = None, False
        fallback = True
        if paired:
            if self.offset is None:
                self.offset = wrap(yaw - sample["yaw"])
                self.estimate = yaw
                reason = "initial compass/odom zero aligned"
            aligned_yaw = wrap(sample["yaw"] + self.offset)
            # Gate against the current independent RF2O input. Gating only
            # against the recursive estimate lets slow disagreement drag the
            # filter away and then rejects a returning, normal PNI sample.
            disagreement = wrap(aligned_yaw - yaw)
            if abs(disagreement) <= self.max_innovation:
                fallback = False
                if sample["tick"] != self.used_tick:
                    innovation = wrap(aligned_yaw - self.estimate)
                    gain = self.variance / (self.variance + self.r)
                    self.estimate = wrap(self.estimate + gain * innovation)
                    self.variance *= 1 - gain
                    used = True
                    if reason != "initial compass/odom zero aligned":
                        reason = "PNI correction applied"
                bounded = constrain_to_short_arc(self.estimate, yaw, aligned_yaw)
                clamped = abs(wrap(bounded - self.estimate)) > 1e-12
                self.estimate = bounded
                if clamped:
                    # The old covariance no longer describes a clipped state.
                    self.variance = self.r
                    reason += "; shortest-arc guard applied"
            else:
                reason = "PNI/RF2O disagreement rejected; RF2O fallback"
            self.used_tick = sample["tick"]
        elif fresh and not paired:
            reason = "waiting for temporally matching PNI/odom samples"
        elif not fresh and sample is not None:
            reason = "PNI stale; RF2O fallback"
        if fallback:
            # Never propagate a previous compass correction through a dropout
            # or disagreement. Keep the initial offset: do not hide divergence
            # by silently re-aligning on every rejected input.
            self.estimate = wrap(yaw)
            self.variance = self.r
        return {
            "aligned": self.offset is not None, "pni_fresh": fresh,
            "pni_used": used, "pni_status": None if sample is None else sample["status"],
            "rf2o_yaw": wrap(yaw), "fused_yaw": wrap(self.estimate),
            "pni_yaw": None if not fresh or self.offset is None else wrap(sample["yaw"] + self.offset),
            "offset_deg": None if self.offset is None else math.degrees(self.offset),
            "innovation_deg": None if innovation is None else math.degrees(innovation),
            "rf2o_pni_delta_deg": None if disagreement is None else math.degrees(disagreement),
            "fallback_to_rf2o": fallback, "arc_clamped": clamped,
            "gain": gain, "variance": self.variance, "reason": reason,
        }
