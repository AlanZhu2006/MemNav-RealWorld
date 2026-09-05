"""Body-yaw feedback for one continuous turn to a camera-relative bearing."""

from collections import deque
import math

from trajectory_control import VelocityCommand


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class HeadingTurn:
    """Latch one world heading; never infer body rotation from sent commands.

    IMU samples are stamped at the bridge's DDS reception. Matching a sample
    to image capture bounds the local transport alignment error, not the
    hardware clock error. A stale or discontinuous IMU terminates the turn.
    """

    def __init__(self):
        self.samples = deque(maxlen=400)
        self.reset()

    def reset(self):
        self.active = False
        self.target_yaw = None
        self.started_s = None
        self.phase = "idle"
        self.error_rad = None
        self.last_yaw = None
        self.last_stamp_ns = None
        self.completed_s = None

    def observe(self, stamp_ns, yaw):
        if stamp_ns <= 0 or not math.isfinite(yaw):
            return
        if self.samples and stamp_ns <= self.samples[-1][0]:
            return
        self.samples.append((stamp_ns, wrap(yaw)))

    def age(self, now_ns):
        return None if not self.samples else (now_ns - self.samples[-1][0]) / 1e9

    def reference(self, image_ns):
        if not self.samples or not image_ns:
            return None
        stamp, yaw = min(self.samples, key=lambda sample: abs(sample[0] - image_ns))
        return yaw if abs(stamp - image_ns) <= 150_000_000 else None

    def start(self, bearing, image_ns, now_ns, now_s):
        reference = self.reference(image_ns)
        age = self.age(now_ns)
        if reference is None or age is None or not 0 <= age <= 0.35:
            self.phase = "heading_feedback_unavailable"
            return False
        if not math.isfinite(bearing):
            self.phase = "invalid_heading_target"
            return False
        self.target_yaw = wrap(reference + bearing)
        self.started_s = now_s
        self.active = True
        self.phase = "turning"
        self.last_stamp_ns, self.last_yaw = self.samples[-1]
        return True

    def step(self, now_ns, now_s, gain, max_wz):
        if not self.active:
            return VelocityCommand()
        age = self.age(now_ns)
        if age is None or not 0 <= age <= 0.35:
            self.phase = "heading_feedback_stale"
        else:
            stamp, yaw = self.samples[-1]
            if stamp > self.last_stamp_ns:
                dt = (stamp - self.last_stamp_ns) / 1e9
                if abs(wrap(yaw - self.last_yaw)) > 3.0 * dt + 0.10:
                    self.phase = "heading_feedback_discontinuity"
                self.last_stamp_ns, self.last_yaw = stamp, yaw
            self.error_rad = wrap(self.target_yaw - yaw)
            if self.phase == "turning":
                if abs(self.error_rad) <= math.radians(8):
                    self.phase = "complete"
                    self.completed_s = now_s
                elif now_s - self.started_s >= 20.0:
                    self.phase = "heading_turn_timeout"
                else:
                    return VelocityCommand(angular_z=max(-max_wz, min(max_wz, gain * self.error_rad)))
        self.active = False
        return VelocityCommand()

    def audit(self, now_ns, now_s=None):
        return dict(active=self.active, phase=self.phase, target_yaw_rad=self.target_yaw,
                    error_rad=self.error_rad, feedback_age_s=self.age(now_ns),
                    feedback_source="go2_lowstate_imu", translation_mps=0.0,
                    completed_age_s=(None if self.completed_s is None or now_s is None
                                     else max(0.0, now_s - self.completed_s)))
