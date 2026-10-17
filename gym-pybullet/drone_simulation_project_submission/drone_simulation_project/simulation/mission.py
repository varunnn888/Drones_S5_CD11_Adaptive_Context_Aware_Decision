"""Mission sequencer for the standard ACMFDS demonstration flight.

Implements the required basic sequence::

    INITIALIZE -> TAKEOFF -> HOVER -> MOVE_FORWARD -> SLOW_DOWN
        -> HOVER (stop) -> LAND -> DONE

The sequencer expresses *mission intent* only (target altitude, target
position, desired heading, speed scale). It never touches motors directly:
intent fills the flight state, ACMFDS decides, and the actuation layer
executes. Safety overrides from ACMFDS (avoidance, wind, energy, emergency)
therefore apply unchanged at every phase.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence, Tuple


class MissionPhase(str, Enum):
    INITIALIZE = "INITIALIZE"
    TAKEOFF = "TAKEOFF"
    HOVER = "HOVER"
    MOVE_FORWARD = "MOVE_FORWARD"
    SLOW_DOWN = "SLOW_DOWN"
    LAND = "LAND"
    DONE = "DONE"


@dataclass
class MissionConfig:
    """Mission parameters (SI units)."""

    cruise_altitude: float = 2.0
    hover_time: float = 2.0            # s to hold before/after movement
    takeoff_tolerance: float = 0.15    # m
    slow_radius: float = 0.9           # m: begin slowing near goal
    reach_radius: float = 0.35         # m: goal accepted
    landing_altitude: float = 0.0
    landed_threshold: float = 0.08     # m: considered on ground
    max_duration: float = 90.0         # s safety timeout

    # Trapezoidal speed profile (purposeful acceleration and braking).
    cruise_speed: float = 1.2          # m/s nominal cruise (range 0.8-1.2)
    max_speed: float = 1.5             # m/s hard cap
    accel: float = 0.7                 # m/s^2 acceleration limit
    decel: float = 1.0                 # m/s^2 deceleration limit


@dataclass
class MissionIntent:
    """What the mission wants this step (consumed via the flight state)."""

    phase: MissionPhase
    target_altitude: float
    target_position: Tuple[float, float, float]
    heading: Optional[float]     # desired yaw, rad; None = keep current
    speed_scale: float           # 0..1 multiplier on cruise speed
    desired_speed: float = 0.0   # m/s planned forward speed (accel/decel-limited)
    description: str = ""


class MissionSequencer:
    """Advances the demonstration mission through its phases."""

    def __init__(
        self,
        goal: Sequence[float] = (4.5, 0.0),
        config: Optional[MissionConfig] = None,
    ) -> None:
        self.config = config or MissionConfig()
        self.goal = (float(goal[0]), float(goal[1]))
        self.phase = MissionPhase.INITIALIZE
        self._phase_time = 0.0
        self._hover_remaining = 0.0
        self._next_after_hover = MissionPhase.MOVE_FORWARD
        self.cruise_altitude = self.config.cruise_altitude
        self._v_cmd = 0.0                 # planned forward speed, m/s

    def reset(self) -> None:
        self.phase = MissionPhase.INITIALIZE
        self._phase_time = 0.0
        self._hover_remaining = 0.0
        self._next_after_hover = MissionPhase.MOVE_FORWARD
        self._v_cmd = 0.0

    @property
    def finished(self) -> bool:
        return self.phase == MissionPhase.DONE

    # ------------------------------------------------------------------
    def update(
        self,
        t: float,
        position: Sequence[float],
        altitude: float,
        velocity_z: float,
        goal_distance: float,
    ) -> MissionIntent:
        """Advance state machine; returns the intent for this step."""
        cfg = self.config
        dt = 1.0 / 120.0
        self._phase_time += dt

        heading_to_goal = self._heading_to(position)
        intent = MissionIntent(
            phase=self.phase,
            target_altitude=self.cruise_altitude,
            target_position=(self.goal[0], self.goal[1], self.cruise_altitude),
            heading=heading_to_goal,
            speed_scale=1.0,
            desired_speed=self._v_cmd,
        )

        # ---- Trapezoidal speed plan ---------------------------------
        # v_target: distance-limited braking curve v = sqrt(2*a*(d - r_stop))
        # so the drone decelerates purposefully instead of crawling to the
        # goal. v_cmd integrates toward v_target under accel/decel limits.
        if self.phase in (MissionPhase.MOVE_FORWARD, MissionPhase.SLOW_DOWN):
            brake_dist = max(0.0, goal_distance - cfg.reach_radius)
            v_target = min(
                cfg.cruise_speed,
                cfg.max_speed,
                math.sqrt(2.0 * cfg.decel * brake_dist),
            )
        else:
            v_target = 0.0
        rate = cfg.accel if v_target > self._v_cmd else cfg.decel
        delta = v_target - self._v_cmd
        self._v_cmd += max(-rate * dt, min(rate * dt, delta))
        self._v_cmd = max(0.0, min(cfg.max_speed, self._v_cmd))
        intent.desired_speed = self._v_cmd
        cruise = max(cfg.cruise_speed, 1e-6)
        intent.speed_scale = self._v_cmd / cruise

        if self.phase == MissionPhase.INITIALIZE:
            intent.target_altitude = altitude       # stay put while settling
            intent.target_position = (position[0], position[1], altitude)
            intent.heading = None
            intent.speed_scale = 0.0
            intent.description = "initializing on ground"
            if self._phase_time >= 0.5:
                self._enter(MissionPhase.TAKEOFF)

        elif self.phase == MissionPhase.TAKEOFF:
            intent.speed_scale = 0.0
            intent.description = f"takeoff to {self.cruise_altitude:.1f} m"
            if abs(altitude - self.cruise_altitude) <= cfg.takeoff_tolerance:
                self._hover_remaining = cfg.hover_time
                self._next_after_hover = MissionPhase.MOVE_FORWARD
                self._enter(MissionPhase.HOVER)

        elif self.phase == MissionPhase.HOVER:
            intent.target_position = (position[0], position[1], self.cruise_altitude)
            intent.heading = heading_to_goal
            intent.speed_scale = 0.0
            intent.description = f"hover (hold {self._hover_remaining:3.1f}s)"
            self._hover_remaining -= dt
            if self._hover_remaining <= 0.0:
                self._enter(self._next_after_hover)

        elif self.phase == MissionPhase.MOVE_FORWARD:
            intent.description = f"cruise to goal ({goal_distance:.2f} m)"
            if goal_distance <= cfg.slow_radius:
                self._enter(MissionPhase.SLOW_DOWN)

        elif self.phase == MissionPhase.SLOW_DOWN:
            intent.description = (
                f"braking for goal ({goal_distance:.2f} m, v={self._v_cmd:.2f} m/s)"
            )
            if goal_distance <= cfg.reach_radius:
                self._hover_remaining = 1.0
                self._next_after_hover = MissionPhase.LAND
                self._enter(MissionPhase.HOVER)

        elif self.phase == MissionPhase.LAND:
            intent.target_altitude = cfg.landing_altitude
            intent.target_position = (position[0], position[1], cfg.landing_altitude)
            intent.heading = None
            intent.speed_scale = 0.0
            intent.description = "landing"
            if altitude <= cfg.landed_threshold and abs(velocity_z) < 0.15:
                self._enter(MissionPhase.DONE)

        elif self.phase == MissionPhase.DONE:
            intent.target_altitude = cfg.landing_altitude
            intent.target_position = (position[0], position[1], 0.0)
            intent.heading = None
            intent.speed_scale = 0.0
            intent.description = "mission complete"

        return intent

    def request_landing(self) -> None:
        """Force transition to LAND (safe-landing / fail-safe path)."""
        if self.phase not in (MissionPhase.LAND, MissionPhase.DONE):
            self._enter(MissionPhase.LAND)

    # ------------------------------------------------------------------
    def _enter(self, phase: MissionPhase) -> None:
        self.phase = phase
        self._phase_time = 0.0

    def _heading_to(self, position: Sequence[float]) -> Optional[float]:
        dx = self.goal[0] - position[0]
        dy = self.goal[1] - position[1]
        # Deadband: within 0.5 m of the goal the bearing is numerically noisy
        # and a re-orientation would be violent — keep the current heading.
        if math.hypot(dx, dy) < 0.5:
            return None
        return math.atan2(dy, dx)
