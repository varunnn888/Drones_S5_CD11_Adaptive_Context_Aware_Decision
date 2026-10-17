"""Configuration and coordinate helpers for the ACMFDS quadrotor demo.

GPS values are simulated from local ENU-like (x=east, y=north) coordinates.
They are not readings from a physical GPS receiver.
"""
from __future__ import annotations

import numpy as np

# Mission in local metres. Keep the existing project's destination.
START = (0.0, 0.0, 2.0)
DESTINATION = (12.0, 10.0, 2.0)
SIM_HZ = 60
BATTERY_START = 100.0

# Editable example origin used to translate local XY to simulated lat/lon.
# Replace these values with the origin for your own experiment if required.
GPS_ORIGIN_LAT = 13.0827
GPS_ORIGIN_LON = 80.2707
GPS_ORIGIN_ALT_M = 0.0
EARTH_RADIUS_M = 6_371_000.0


def latlon_to_local_xy(
    lat: float,
    lon: float,
    origin_lat: float = GPS_ORIGIN_LAT,
    origin_lon: float = GPS_ORIGIN_LON,
) -> tuple[float, float]:
    """Convert latitude/longitude to approximate local X/Y metres.

    X is east, Y is north. This small-area equirectangular approximation is
    appropriate for this dashboard demo, not high-accuracy geodesy.
    """
    lat0 = np.deg2rad(float(origin_lat))
    x = np.deg2rad(float(lon) - float(origin_lon)) * EARTH_RADIUS_M * np.cos(lat0)
    y = np.deg2rad(float(lat) - float(origin_lat)) * EARTH_RADIUS_M
    return float(x), float(y)


def local_xy_to_latlon(
    x: float,
    y: float,
    origin_lat: float = GPS_ORIGIN_LAT,
    origin_lon: float = GPS_ORIGIN_LON,
) -> tuple[float, float]:
    """Convert local X/Y metres to simulated latitude and longitude."""
    lat = float(origin_lat) + np.rad2deg(float(y) / EARTH_RADIUS_M)
    cos_lat = max(abs(np.cos(np.deg2rad(float(origin_lat)))), 1e-9)
    lon = float(origin_lon) + np.rad2deg(float(x) / (EARTH_RADIUS_M * cos_lat))
    return float(lat), float(lon)
