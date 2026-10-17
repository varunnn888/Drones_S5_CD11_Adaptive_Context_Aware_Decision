"""Configurable wind model (MODELED state — not a physical sensor).

The model produces a time-varying wind velocity vector. The physics coupling
happens in the simulation loop via ``apply_wind_force``; ACMFDS receives the
*same* modeled vector through the flight state, so decision and physics
always agree.

Force conversion (applied only when enabled):
    F_wind = 0.5 * rho * Cd * A * |v_wind - v_drone|^2 * dir(v_wind - v_drone)
with rho = 1.225 kg/m^3, Cd = 1.0, A = 0.09 m^2 (documented guesses for a
1 kg quadcopter). Relative velocity is used so the gust pushes the physical
body — and the effect is verifiable by observing acceleration.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

AIR_DENSITY = 1.225      # kg/m^3
DRAG_COEFFICIENT = 1.0   # blunt-body approximation
EXPOSED_AREA = 0.09      # m^2


@dataclass
class WindConfig:
    """Wind scenario parameters (all SI)."""

    enabled: bool = False
    base_speed: float = 0.0             # m/s steady component
    direction: Tuple[float, float, float] = (1.0, 0.0, 0.0)  # unit vector
    gust_amplitude: float = 0.0         # m/s sinusoidal gust amplitude
    gust_period: float = 8.0            # s
    turbulence: float = 0.0             # m/s band-limited noise amplitude
    seed: int = 42


class WindModel:
    """Time-varying wind velocity vector with reproducible behavior."""

    def __init__(self, config: Optional[WindConfig] = None) -> None:
        self.config = config or WindConfig()
        d = np.asarray(self.config.direction, dtype=np.float64)
        n = np.linalg.norm(d)
        if n < 1e-9:
            d = np.array([1.0, 0.0, 0.0])
        else:
            d = d / n
        self._direction = d
        # Two independent smooth noise streams (reproducible via seed).
        rng = np.random.default_rng(self.config.seed)
        self._noise_phase = rng.uniform(0.0, 2.0 * math.pi, size=4)
        self._noise_freq = rng.uniform(0.05, 0.4, size=4)

    def velocity(self, t: float) -> np.ndarray:
        """Wind velocity vector (world frame, m/s) at sim time t."""
        cfg = self.config
        if not cfg.enabled:
            return np.zeros(3, dtype=np.float64)
        speed = cfg.base_speed
        if cfg.gust_amplitude > 0.0 and cfg.gust_period > 0.0:
            speed += cfg.gust_amplitude * math.sin(2.0 * math.pi * t / cfg.gust_period)
        if cfg.turbulence > 0.0:
            jitter = sum(
                math.sin(2.0 * math.pi * f * t + ph)
                for f, ph in zip(self._noise_freq, self._noise_phase)
            ) / 4.0
            speed += cfg.turbulence * jitter
        return self._direction * max(0.0, speed)

    def speed(self, t: float) -> float:
        """Wind magnitude (m/s) at sim time t."""
        return float(np.linalg.norm(self.velocity(t)))

    def relative_force(self, wind_velocity: np.ndarray, drone_velocity: Sequence[float]) -> np.ndarray:
        """Aerodynamic force exerted by wind on the drone body, newtons.

        Uses relative airspeed so the disturbance physically couples to the
        simulated rigid body.
        """
        v_rel = np.asarray(wind_velocity, dtype=np.float64) - np.asarray(drone_velocity, dtype=np.float64)
        speed = float(np.linalg.norm(v_rel))
        if speed < 1e-6:
            return np.zeros(3, dtype=np.float64)
        dynamic = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * EXPOSED_AREA * speed * speed
        return dynamic * (v_rel / speed)

    def apply(self, physics_client_id: int, drone_id: int, t: float) -> np.ndarray:
        """Apply the wind force to the drone in PyBullet (WORLD_FRAME).

        Returns the force vector applied (zeros when disabled).
        """
        import pybullet as p

        if not self.config.enabled or drone_id < 0:
            return np.zeros(3, dtype=np.float64)
        wind_v = self.velocity(t)
        lin_vel, _ = p.getBaseVelocity(drone_id, physicsClientId=physics_client_id)
        base_pos, _ = p.getBasePositionAndOrientation(drone_id, physicsClientId=physics_client_id)
        force = self.relative_force(wind_v, lin_vel)
        # WORLD_FRAME posObj is a *world-coordinate* application point;
        # [0,0,0] would be the world origin meters away, creating a spurious
        # torque. Apply at the body's center of mass.
        p.applyExternalForce(
            drone_id,
            -1,
            forceObj=force.tolist(),
            posObj=list(base_pos),
            flags=p.WORLD_FRAME,
            physicsClientId=physics_client_id,
        )
        return force
