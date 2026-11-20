"""ROS-independent frame conversion and manual-transmission state machine."""

import math
from dataclasses import dataclass


def rotate_vector(quaternion, vector):
    """Rotate a body vector with a normalized xyzw quaternion."""
    x, y, z, w = quaternion
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid orientation quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    vx, vy, vz = vector
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx)


def shift_velocity(velocity, angular_velocity, offset):
    """v(new origin) = v(old origin) + omega cross offset, all in body axes."""
    vx, vy, vz = velocity
    wx, wy, wz = angular_velocity
    x, y, z = offset
    return (vx+wy*z-wz*y, vy+wz*x-wx*z, vz+wx*y-wy*x)


def shift_acceleration(acceleration, omega, alpha, offset):
    """Acceleration at a fixed body offset includes tangential/centripetal terms."""
    centripetal = shift_velocity((0., 0., 0.), omega,
        shift_velocity((0., 0., 0.), omega, offset))
    tangential = shift_velocity((0., 0., 0.), alpha, offset)
    return tuple(a+c+t for a,c,t in zip(acceleration, centripetal, tangential))


@dataclass(frozen=True)
class TransmissionCommand:
    gear: int
    clutch: float
    gas: float


class ManualTransmission:
    """Conservative automatic operation of the unchanged five-speed manual gearbox.

    Clutch 1 disconnects, clutch 0 connects. Thresholds are initial settings;
    production pedal calibration must use this same shift strategy.
    """

    def __init__(self, release_sec=1.5, shift_sec=0.6, thresholds=(5., 9., 13., 18.)):
        if release_sec <= 0 or shift_sec <= 0:
            raise ValueError("Clutch and shift durations must be positive")
        if len(thresholds) != 4 or any(a >= b for a, b in zip(thresholds, thresholds[1:])):
            raise ValueError("Four increasing upshift speeds are required")
        self.release_sec = release_sec
        self.shift_sec = shift_sec
        self.thresholds = tuple(thresholds)
        self.reset()

    def reset(self):
        self.gear = 0
        self.started = None
        self.shifting = None
        self.last_time = None

    def step(self, now, speed, direction, gas, brake, safe_stop=False):
        if not all(math.isfinite(x) for x in (now, speed, gas, brake)):
            self.reset()
            return TransmissionCommand(0, 1., 0.)
        if self.last_time is not None and now < self.last_time:
            self.reset()
        self.last_time = now
        gas = max(0., min(1., gas))
        if safe_stop or direction == 0:
            self.gear, self.started, self.shifting = 0, None, None
            return TransmissionCommand(0, 1., 0.)
        # Direction changes require a stop; never select reverse while moving forward.
        if speed * direction < -0.2:
            return TransmissionCommand(self.gear, 1., 0.)
        target = -1 if direction < 0 else 1
        if direction > 0:
            target += sum(abs(speed) > threshold for threshold in self.thresholds)
            # Hysteresis avoids shifting repeatedly near a speed threshold.
            if self.gear > target and self.gear > 1:
                if abs(speed) > self.thresholds[self.gear-2] - 1.0:
                    target = self.gear
        if abs(speed) < 0.5:
            self.gear, self.shifting = target, None
            if gas == 0 or brake > 0.1:
                self.started = None
                return TransmissionCommand(target, 1., 0.)
            if self.started is None:
                self.started = now
            clutch = max(0., 1. - (now-self.started)/self.release_sec)
            return TransmissionCommand(target, clutch, min(gas, 0.25))
        if self.gear != target:
            self.gear, self.shifting = target, now
            self.started = None
        if self.started is not None:
            clutch = max(0., 1. - (now-self.started)/self.release_sec)
            if clutch > 0.:
                return TransmissionCommand(self.gear, clutch, min(gas, 0.25) if brake <= 0.1 else 0.)
            self.started = None
        if self.shifting is not None:
            elapsed = now-self.shifting
            if elapsed < self.shift_sec:
                clutch = min(1., max(0., 2.-2.*elapsed/self.shift_sec))
                return TransmissionCommand(target, clutch, 0.)
            self.shifting = None
        return TransmissionCommand(target, 0., gas if brake <= 0.1 else 0.)
