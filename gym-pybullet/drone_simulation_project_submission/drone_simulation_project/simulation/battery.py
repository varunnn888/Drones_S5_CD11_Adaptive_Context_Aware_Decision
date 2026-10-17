"""Simulated battery model (MODELED state — not a physical sensor).

Consumption is tied to elapsed simulation time and an electrical-power
estimate computed from the commanded thrust fraction: harder work draws more
current. The model is deliberately simple and fully documented; it exists so
ACMFDS can reason about energy, not to emulate a specific LiPo chemistry.
"""

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class BatteryConfig:
    """Battery model parameters."""

    capacity_wh: float = 4.0         # total stored energy, watt-hours (~5 min hover)
    initial_charge_percent: float = 100.0
    base_draw_w: float = 15.0        # avionics + idle motors
    thrust_draw_w: float = 85.0      # additional draw at full thrust
    low_threshold: float = 30.0      # %
    critical_threshold: float = 15.0  # %
    emergency_threshold: float = 5.0  # %


class BatteryModel:
    """Tracks remaining simulated charge as a function of time and workload.

    Update each physics step with the currently commanded thrust fraction
    (0 = no thrust, 1 = max thrust). All state changes are dt-based; nothing
    is subtracted without relating to elapsed simulation time.
    """

    def __init__(self, config: Optional[BatteryConfig] = None) -> None:
        self.config = config or BatteryConfig()
        self._charge_wh: float = self.config.capacity_wh * (
            self.config.initial_charge_percent / 100.0
        )
        self._last_thrust_fraction = 0.0
        self._consumed_wh = 0.0

    @property
    def percentage(self) -> float:
        """Remaining charge, percent, clamped to [0, 100]."""
        pct = 100.0 * self._charge_wh / self.config.capacity_wh
        return max(0.0, min(100.0, pct))

    @property
    def consumed_wh(self) -> float:
        """Total energy consumed since construction, watt-hours."""
        return self._consumed_wh

    @property
    def last_power_w(self) -> float:
        """Most recent electrical power estimate, watts."""
        frac = self._last_thrust_fraction
        return self.config.base_draw_w + self.config.thrust_draw_w * (frac ** 1.5)

    def update(self, dt: float, thrust_fraction: float) -> float:
        """Advance the model by ``dt`` seconds of sim time.

        Args:
            dt: elapsed simulation time, seconds (> 0)
            thrust_fraction: commanded thrust in [0, 1]

        Returns:
            remaining percentage after the update
        """
        if dt <= 0.0 or not math.isfinite(dt):
            return self.percentage
        frac = max(0.0, min(1.0, float(thrust_fraction)))
        if not math.isfinite(frac):
            frac = 0.0
        self._last_thrust_fraction = frac

        power_w = self.config.base_draw_w + self.config.thrust_draw_w * (frac ** 1.5)
        used_wh = power_w * (dt / 3600.0)  # W * h
        self._charge_wh = max(0.0, self._charge_wh - used_wh)
        self._consumed_wh += used_wh
        return self.percentage

    def is_low(self) -> bool:
        return self.percentage <= self.config.low_threshold

    def is_critical(self) -> bool:
        return self.percentage <= self.config.critical_threshold

    def is_emergency(self) -> bool:
        return self.percentage <= self.config.emergency_threshold

    def set_charge_percent(self, percent: float) -> None:
        """Directly set charge (scenario setup). Clamped to [0, 100]."""
        p = max(0.0, min(100.0, float(percent)))
        if not math.isfinite(p):
            p = 100.0
        self._charge_wh = self.config.capacity_wh * (p / 100.0)
