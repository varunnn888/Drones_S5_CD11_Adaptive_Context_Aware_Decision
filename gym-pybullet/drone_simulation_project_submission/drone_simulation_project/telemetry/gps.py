"""Simulated GPS position (MODELED state — not a real satellite receiver).

Convention (exact): the geographic reference point ``(lat0, lon0)`` —
the ORIGIN — corresponds to world position ``(X=0, Y=0)`` of the PyBullet
simulation frame, where X points east and Y points north. The drone does
NOT start at the origin: the scenarios spawn it at ``(-4.0, 0.0)``, i.e.
4 m west of the origin. Callers must pass true world coordinates to
:meth:`GpsReference.fix`; no offset is applied inside.

World X/Y metres are mapped onto geographic degrees around the origin using
the flat-earth small-area approximation:

    lat = lat0 + y / 111320
    lon = lon0 + x / (111320 * cos(lat0))

Accuracy is well under a metre at the ~15 m scale of these scenarios, but
every value produced here is simulated and must be labeled as such in any
user-facing output. It is never a real GPS reading and never an operator
location.
"""

import math
from dataclasses import dataclass
from typing import Tuple

M_PER_DEG_LAT = 111320.0  # metres per degree of latitude (WGS84 mean)


@dataclass
class GpsReference:
    """Configurable geographic reference (degrees) at local origin (0, 0)."""

    lat0: float = 47.3769
    lon0: float = 8.5417

    def fix(self, x_m: float, y_m: float) -> Tuple[float, float]:
        """Convert local X/Y metres to simulated (lat, lon) degrees."""
        lat = self.lat0 + y_m / M_PER_DEG_LAT
        lon = self.lon0 + x_m / (
            M_PER_DEG_LAT * math.cos(math.radians(self.lat0))
        )
        return lat, lon
