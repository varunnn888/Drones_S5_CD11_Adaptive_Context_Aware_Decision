"""Trajectory planning and interpolation package."""

from .interpolation import (
    WaypointNavigator,
    interpolate_waypoints,
    linear_interpolate,
    DEFAULT_WAYPOINTS,
)

__all__ = [
    "WaypointNavigator",
    "interpolate_waypoints",
    "linear_interpolate",
    "DEFAULT_WAYPOINTS",
]
