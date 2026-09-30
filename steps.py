"""Simple orientation-independent step detection for MPU-6050 samples."""

from __future__ import annotations


class StepCounter:
    """Count acceleration peaks while suppressing vibration and double counts."""

    def __init__(self, threshold: float = 1.20, reset_threshold: float = 1.05, refractory: float = 0.30):
        self.threshold = threshold
        self.reset_threshold = reset_threshold
        self.refractory = refractory
        self.count = 0
        self._armed = True
        self._last_step = -float("inf")

    def update(self, accel_mag: float, timestamp: float) -> bool:
        if accel_mag <= self.reset_threshold:
            self._armed = True
        if self._armed and accel_mag >= self.threshold and timestamp - self._last_step >= self.refractory:
            self.count += 1
            self._last_step = timestamp
            self._armed = False
            return True
        return False
