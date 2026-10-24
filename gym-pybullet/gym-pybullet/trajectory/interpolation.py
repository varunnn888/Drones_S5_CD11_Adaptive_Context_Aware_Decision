"""Trajectory interpolation and waypoint navigation.

Provides:
- Smooth multi-segment interpolation between 3D waypoints
- Real-time waypoint tracking and steering angle calculation
- Reference obstacle avoidance flight path preserved from reference project
"""

import math
from typing import List, Tuple, Sequence, Optional
import numpy as np


# Reference obstacle avoidance waypoints from the reference simulation
DEFAULT_WAYPOINTS: List[List[float]] = [
    [-4.0, 0.0, 2.0],   # Start position
    [-2.0, -1.5, 2.0],  # Clear obstacle 1
    [0.0, -1.5, 2.0],   # Safely pass obstacle 1
    [1.5, -1.5, 2.0],   # Clear obstacle 2
    [3.0, -1.8, 2.0],   # Clear obstacle 3
    [4.5, 0.0, 2.0],    # Goal / Destination
]


def linear_interpolate(p1: Sequence[float], p2: Sequence[float], alpha: float) -> List[float]:
    """Linear interpolation between two 3D points.

    Args:
        p1: Start point [x, y, z]
        p2: End point [x, y, z]
        alpha: Interpolation parameter in [0, 1]

    Returns:
        Interpolated point [x, y, z]
    """
    alpha = max(0.0, min(1.0, float(alpha)))
    return [
        p1[0] + alpha * (p2[0] - p1[0]),
        p1[1] + alpha * (p2[1] - p1[1]),
        p1[2] + alpha * (p2[2] - p1[2]),
    ]


def interpolate_waypoints(
    waypoints: Sequence[Sequence[float]],
    num_samples_per_segment: int = 15,
) -> List[List[float]]:
    """Generate dense interpolated trajectory through waypoints.

    Args:
        waypoints: List of [x, y, z] waypoints
        num_samples_per_segment: Number of interpolated steps per segment

    Returns:
        Dense trajectory path as list of points
    """
    if len(waypoints) < 2:
        return [list(wp) for wp in waypoints]

    dense_path: List[List[float]] = []
    for i in range(len(waypoints) - 1):
        p_start = waypoints[i]
        p_end = waypoints[i + 1]

        for s in range(num_samples_per_segment):
            alpha = s / float(num_samples_per_segment)
            dense_path.append(linear_interpolate(p_start, p_end, alpha))

    # Add the final destination point
    dense_path.append(list(waypoints[-1]))
    return dense_path


class WaypointNavigator:
    """Manages sequential waypoint navigation for baseline drone flight.

    Tracks active waypoint, calculates heading/yaw, computes step movement,
    and signals mission completion.
    """

    def __init__(
        self,
        waypoints: Optional[Sequence[Sequence[float]]] = None,
        acceptance_radius: float = 0.35,
        loop: bool = False,
    ) -> None:
        self.waypoints = [list(wp) for wp in (waypoints or DEFAULT_WAYPOINTS)]
        self.acceptance_radius = float(acceptance_radius)
        self.loop = loop
        self.current_index = 0

    def reset(self) -> None:
        """Reset navigator to the first waypoint."""
        self.current_index = 0

    @property
    def current_target(self) -> List[float]:
        """Get the current active waypoint coordinates."""
        if not self.waypoints:
            return [0.0, 0.0, 2.0]
        return self.waypoints[min(self.current_index, len(self.waypoints) - 1)]

    @property
    def is_finished(self) -> bool:
        """True if final waypoint reached (when not looping)."""
        return not self.loop and self.current_index >= len(self.waypoints)

    def get_desired_heading(self, current_pos: Sequence[float]) -> float:
        """Calculate desired yaw angle (radians) facing current target."""
        target = self.current_target
        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]
        if abs(dx) > 1e-4 or abs(dy) > 1e-4:
            return math.atan2(dy, dx)
        return 0.0

    def update(self, current_pos: Sequence[float]) -> Tuple[List[float], bool, bool]:
        """Update waypoint status given drone's current position.

        Args:
            current_pos: [x, y, z] of drone

        Returns:
            Tuple of:
                - target: active [x, y, z] target
                - waypoint_reached: True if current waypoint was reached on this step
                - mission_completed: True if all waypoints have been completed
        """
        target = self.current_target
        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]
        dz = target[2] - current_pos[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        waypoint_reached = False
        mission_completed = False

        if dist <= self.acceptance_radius:
            waypoint_reached = True
            if self.current_index < len(self.waypoints) - 1:
                self.current_index += 1
            else:
                if self.loop:
                    self.current_index = 0
                else:
                    mission_completed = True

        return self.current_target, waypoint_reached, mission_completed
