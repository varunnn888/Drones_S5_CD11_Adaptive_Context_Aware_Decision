"""Unified, validated flight state for the ACMFDS decision system.

One well-defined state representation feeds every consumer (ACMFDS,
the actuation layer, and logging).

Value provenance is explicit through :class:`SourceTag`:

* ``SIMULATION`` — read directly from the PyBullet rigid body (position,
  velocity, orientation) or derived from it by plain geometry.
* ``SENSOR`` — computed from PyBullet ray tests (obstacle distances).
* ``MODELED`` — simulated models (battery, wind). These are NOT physical
  sensors provided by the environment and are labelled as such.

Units (project-wide contract):
    position / distance : meters
    velocity            : meters per second
    angles              : radians
    battery             : percent, 0..100
    wind speed          : meters per second
    thrust / force      : newtons
"""

import math
from dataclasses import dataclass, field, fields
from enum import Enum
from typing import Dict, List, Optional, Tuple


class SourceTag(str, Enum):
    """Origin of a state value."""

    SIMULATION = "SIMULATION"  # PyBullet rigid-body state
    SENSOR = "SENSOR"          # PyBullet ray-cast sensing
    MODELED = "MODELED"        # simulated battery / wind models


# Field name -> source tag. Kept next to the dataclass so the contract
# is visible in exactly one place.
FIELD_SOURCES: Dict[str, SourceTag] = {
    "position_x": SourceTag.SIMULATION,
    "position_y": SourceTag.SIMULATION,
    "altitude": SourceTag.SIMULATION,
    "velocity_x": SourceTag.SIMULATION,
    "velocity_y": SourceTag.SIMULATION,
    "velocity_z": SourceTag.SIMULATION,
    "roll": SourceTag.SIMULATION,
    "pitch": SourceTag.SIMULATION,
    "yaw": SourceTag.SIMULATION,
    "roll_rate": SourceTag.SIMULATION,
    "pitch_rate": SourceTag.SIMULATION,
    "yaw_rate": SourceTag.SIMULATION,
    "target_altitude": SourceTag.SIMULATION,   # commanded by mission plan
    "target_position": SourceTag.SIMULATION,   # commanded by mission plan
    "goal_distance": SourceTag.SIMULATION,     # geometry from simulation pose
    "mission_phase": SourceTag.SIMULATION,     # commanded by mission sequencer
    "maneuver_cooldown_s": SourceTag.SIMULATION,  # runner maneuver tracker
    "obstacle_distance": SourceTag.SENSOR,     # min forward-cone clearance
    "clearance_front": SourceTag.SENSOR,       # forward ray, m
    "clearance_left": SourceTag.SENSOR,        # front-left ray, m
    "clearance_right": SourceTag.SENSOR,       # front-right ray, m
    "clearance_up": SourceTag.SENSOR,          # overhead ray, m
    "obstacle_top": SourceTag.SENSOR,          # AABB top of front-hit body, m
    "battery_percentage": SourceTag.MODELED,   # simulated battery model
    "wind_speed": SourceTag.MODELED,           # simulated wind model
    "wind_direction_x": SourceTag.MODELED,
    "wind_direction_y": SourceTag.MODELED,
    "wind_direction_z": SourceTag.MODELED,
}


@dataclass
class FlightState:
    """Single validated snapshot of everything ACMFDS needs.

    Attributes follow the project units contract above. ``target_position``
    is a 3-tuple (x, y, z) in meters; ``obstacle_distance`` is the smallest
    forward-relevant ray clearance in meters (max = sensor range when clear).
    """

    position_x: float = 0.0
    position_y: float = 0.0
    altitude: float = 0.0
    target_altitude: float = 0.0

    velocity_x: float = 0.0
    velocity_y: float = 0.0
    velocity_z: float = 0.0

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0

    obstacle_distance: float = 5.0
    clearance_front: float = 5.0
    clearance_left: float = 5.0
    clearance_right: float = 5.0
    clearance_up: float = 5.0
    obstacle_top: Optional[float] = None       # m; None = nothing ahead

    battery_percentage: float = 100.0

    wind_speed: float = 0.0
    wind_direction_x: float = 1.0
    wind_direction_y: float = 0.0
    wind_direction_z: float = 0.0

    target_position: Tuple[float, float, float] = (0.0, 0.0, 2.0)
    goal_distance: float = 0.0

    # Mission sequencer phase name (e.g. "TAKEOFF", "LAND", "DONE"). The
    # decision layer uses this — not goal distance — to classify the
    # LANDING / MISSION_COMPLETE contexts.
    mission_phase: str = ""

    # Seconds remaining in the post-maneuver caution window (0 = none).
    # Body-frame rays can lose an obstacle during a heading change right
    # after a detour; the caution keeps speed limited until the sweep is
    # provably clear again. Derived by the runner from its measured
    # maneuver windows.
    maneuver_cooldown_s: float = 0.0

    # Filled by StateValidator.sanitize(); useful for logging/telemetry.
    issues: List[str] = field(default_factory=list)

    @property
    def horizontal_speed(self) -> float:
        """Speed in the ground plane, m/s."""
        return math.hypot(self.velocity_x, self.velocity_y)

    @property
    def wind_vector(self) -> Tuple[float, float, float]:
        """Wind velocity vector (m/s) built from magnitude + direction."""
        if self.wind_speed <= 0.0:
            return (0.0, 0.0, 0.0)
        return (
            self.wind_speed * self.wind_direction_x,
            self.wind_speed * self.wind_direction_y,
            self.wind_speed * self.wind_direction_z,
        )

    @property
    def altitude_error(self) -> float:
        """target - current altitude, m. Positive means drone should climb."""
        return self.target_altitude - self.altitude

    def source_of(self, field_name: str) -> SourceTag:
        """Return the provenance tag for a state field."""
        return FIELD_SOURCES[field_name]

    def as_dict(self) -> Dict[str, object]:
        """Plain-dict view (target_position kept as tuple)."""
        return {f.name: getattr(self, f.name) for f in fields(self)}


class StateValidator:
    """Deliberate handling of missing / non-finite / out-of-range values.

    Policy: never crash the flight loop on bad telemetry. Replace non-finite
    numbers with safe defaults, clamp out-of-range values to their physical
    bounds, and record every correction in ``state.issues`` so the log shows
    that sanitization happened (no silent rewrites).
    """

    # (low, high, safe_default) per clampable field.
    _RANGES: Dict[str, Tuple[Optional[float], Optional[float], float]] = {
        "altitude": (0.0, None, 0.0),
        "target_altitude": (0.0, None, 0.0),
        "velocity_x": (-15.0, 15.0, 0.0),
        "velocity_y": (-15.0, 15.0, 0.0),
        "velocity_z": (-15.0, 15.0, 0.0),
        "roll": (-math.pi, math.pi, 0.0),
        "pitch": (-math.pi, math.pi, 0.0),
        "yaw": (-math.pi, math.pi, 0.0),
        "roll_rate": (-20.0, 20.0, 0.0),
        "pitch_rate": (-20.0, 20.0, 0.0),
        "yaw_rate": (-20.0, 20.0, 0.0),
        "obstacle_distance": (0.0, None, 5.0),
        "maneuver_cooldown_s": (0.0, None, 0.0),
        "clearance_front": (0.0, None, 5.0),
        "clearance_left": (0.0, None, 5.0),
        "clearance_right": (0.0, None, 5.0),
        "clearance_up": (0.0, None, 5.0),
        "battery_percentage": (0.0, 100.0, 100.0),
        "wind_speed": (0.0, None, 0.0),
        "goal_distance": (0.0, None, 0.0),
    }

    @classmethod
    def sanitize(cls, state: FlightState) -> FlightState:
        """Return the same state after in-place correction; corrections are
        appended to ``state.issues``."""
        numeric_fields = [f.name for f in fields(state) if f.type in ("float", float)]

        for name in numeric_fields:
            value = getattr(state, name)
            try:
                x = float(value)
            except (TypeError, ValueError):
                state.issues.append(f"{name}: non-numeric ({value!r}) -> 0.0")
                setattr(state, name, 0.0)
                continue
            if math.isnan(x) or math.isinf(x):
                default = cls._RANGES.get(name, (None, None, 0.0))[2]
                state.issues.append(f"{name}: non-finite ({x}) -> {default}")
                setattr(state, name, default)
                continue
            limits = cls._RANGES.get(name)
            if limits is not None:
                low, high, default = limits
                if low is not None and x < low:
                    state.issues.append(f"{name}: {x} below {low} -> clamped")
                    setattr(state, name, low)
                elif high is not None and x > high:
                    state.issues.append(f"{name}: {x} above {high} -> clamped")
                    setattr(state, name, high)

        # Wind direction must be a unit vector; normalize defensively.
        dx, dy, dz = (
            state.wind_direction_x,
            state.wind_direction_y,
            state.wind_direction_z,
        )
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if state.wind_speed > 0.0:
            if norm < 1e-6:
                state.issues.append("wind_direction: zero vector with wind>0 -> +X")
                state.wind_direction_x, state.wind_direction_y, state.wind_direction_z = 1.0, 0.0, 0.0
            elif abs(norm - 1.0) > 1e-3:
                state.issues.append(f"wind_direction: norm {norm:.3f} -> normalized")
                state.wind_direction_x, state.wind_direction_y, state.wind_direction_z = dx / norm, dy / norm, dz / norm

        # Target position tuple sanity.
        tp = state.target_position
        if not isinstance(tp, (tuple, list)) or len(tp) != 3:
            state.issues.append("target_position: malformed -> (0, 0, 2)")
            state.target_position = (0.0, 0.0, 2.0)
        else:
            try:
                state.target_position = (float(tp[0]), float(tp[1]), float(tp[2]))
            except (TypeError, ValueError):
                state.issues.append("target_position: non-numeric -> (0, 0, 2)")
                state.target_position = (0.0, 0.0, 2.0)

        return state
