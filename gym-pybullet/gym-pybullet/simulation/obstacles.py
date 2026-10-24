"""3D Obstacle generation, collision geometry, and management for PyBullet.

Preserves the original reference obstacle configurations while supporting
configurable and randomized generation for reinforcement learning.

Moving obstacles are kinematic bodies (mass 0) whose position is an analytic
function of simulation time, so their trajectories are exactly reproducible:
``pos(t) = center + axis * amplitude * sin(2*pi*t/period + phase)``. They
have real collision geometry — the drone can physically hit them and the ray
sensors see them like any other body.
"""

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Tuple, Sequence, Optional
import numpy as np
import pybullet as p


@dataclass
class ObstacleConfig:
    """Specification of an obstacle."""
    position: Tuple[float, float, float]
    half_extents: Tuple[float, float, float]
    color: Tuple[float, float, float, float] = (0.85, 0.15, 0.15, 1.0)


@dataclass
class MovingObstacleConfig:
    """Specification of a reproducible moving (kinematic) obstacle.

    The body oscillates along ``axis`` through ``center`` with the given
    amplitude, period, and phase — a deterministic closed-form trajectory.
    """
    center: Tuple[float, float, float]
    half_extents: Tuple[float, float, float]
    axis: Tuple[float, float, float] = (0.0, 1.0, 0.0)
    amplitude: float = 2.0              # m
    period: float = 8.0                 # s
    phase: float = 0.0                  # rad
    color: Tuple[float, float, float, float] = (0.95, 0.65, 0.1, 1.0)


class MovingObstacle:
    """Runtime handle for one kinematic obstacle following its analytic path."""

    def __init__(self, config: MovingObstacleConfig, body_id: int = -1) -> None:
        self.config = config
        self.body_id = body_id
        self.position: Tuple[float, float, float] = self.position_at(0.0)
        self.velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    def position_at(self, t: float) -> Tuple[float, float, float]:
        cfg = self.config
        s = cfg.amplitude * math.sin(2.0 * math.pi * t / cfg.period + cfg.phase)
        return (
            cfg.center[0] + cfg.axis[0] * s,
            cfg.center[1] + cfg.axis[1] * s,
            cfg.center[2] + cfg.axis[2] * s,
        )

    def velocity_at(self, t: float) -> Tuple[float, float, float]:
        cfg = self.config
        s = (
            cfg.amplitude
            * (2.0 * math.pi / cfg.period)
            * math.cos(2.0 * math.pi * t / cfg.period + cfg.phase)
        )
        return (cfg.axis[0] * s, cfg.axis[1] * s, cfg.axis[2] * s)


# Default static obstacles from the reference project
DEFAULT_STATIC_OBSTACLES: List[ObstacleConfig] = [
    ObstacleConfig(
        position=(-1.0, 0.0, 1.2),
        half_extents=(0.35, 0.35, 1.2),
        color=(0.9, 0.1, 0.1, 1.0),
    ),
    ObstacleConfig(
        position=(1.0, 0.8, 1.4),
        half_extents=(0.35, 0.35, 1.4),
        color=(0.9, 0.2, 0.1, 1.0),
    ),
    ObstacleConfig(
        position=(2.5, -0.5, 1.1),
        half_extents=(0.35, 0.35, 1.1),
        color=(0.85, 0.1, 0.2, 1.0),
    ),
]


class ObstacleManager:
    """Manages creation, clearing, and queries for physical 3D obstacles."""

    def __init__(
        self,
        physics_client_id: int = 0,
        randomize: bool = False,
        num_random_obstacles: int = 4,
    ) -> None:
        self.client_id = physics_client_id
        self.randomize = randomize
        self.num_random_obstacles = num_random_obstacles
        self.obstacle_ids: List[int] = []
        self.obstacle_configs: List[ObstacleConfig] = []
        self.moving_obstacles: List[MovingObstacle] = []

    def create_obstacle(self, config: ObstacleConfig) -> int:
        """Create a single box obstacle with collision and visual shape.

        Args:
            config: ObstacleConfig containing position and half extents

        Returns:
            PyBullet multi-body ID
        """
        collision_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=list(config.half_extents),
            physicsClientId=self.client_id,
        )

        visual_id = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=list(config.half_extents),
            rgbaColor=list(config.color),
            physicsClientId=self.client_id,
        )

        body_id = p.createMultiBody(
            baseMass=0,  # Static obstacle
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            basePosition=list(config.position),
            physicsClientId=self.client_id,
        )

        self.obstacle_ids.append(body_id)
        self.obstacle_configs.append(config)
        return body_id

    def setup_from_configs(self, configs: Sequence[ObstacleConfig]) -> List[int]:
        """Spawn an explicit, reproducible obstacle layout (scenario use)."""
        self.clear_obstacles()
        for cfg in configs:
            self.create_obstacle(cfg)
        return self.obstacle_ids

    def setup_obstacles(
        self,
        np_random: Optional[np.random.Generator] = None,
        start_pos: Sequence[float] = (-4.0, 0.0, 2.0),
        goal_pos: Sequence[float] = (4.5, 0.0, 2.0),
    ) -> List[int]:
        """Clear existing obstacles and spawn configured or randomized obstacles.

        Args:
            np_random: Numpy random generator for reproducibility
            start_pos: Start location to avoid blocking with obstacles
            goal_pos: Goal location to avoid blocking with obstacles

        Returns:
            List of created obstacle body IDs
        """
        self.clear_obstacles()

        if not self.randomize:
            # Spawn default reference obstacles
            for cfg in DEFAULT_STATIC_OBSTACLES:
                self.create_obstacle(cfg)
        else:
            # Spawn randomized obstacles
            rng = np_random if np_random is not None else np.random.default_rng()
            count = 0
            attempts = 0
            max_attempts = 100

            while count < self.num_random_obstacles and attempts < max_attempts:
                attempts += 1
                # Sample position between start and goal
                x = float(rng.uniform(-2.5, 3.5))
                y = float(rng.uniform(-1.8, 1.8))
                z_half = float(rng.uniform(0.8, 1.5))
                z = z_half  # Grounded obstacle

                # Check clearance from start and goal
                dist_start = np.hypot(x - start_pos[0], y - start_pos[1])
                dist_goal = np.hypot(x - goal_pos[0], y - goal_pos[1])

                if dist_start < 1.2 or dist_goal < 1.2:
                    continue

                half_x = float(rng.uniform(0.25, 0.45))
                half_y = float(rng.uniform(0.25, 0.45))

                cfg = ObstacleConfig(
                    position=(x, y, z),
                    half_extents=(half_x, half_y, z_half),
                    color=(
                        float(rng.uniform(0.75, 0.95)),
                        float(rng.uniform(0.1, 0.3)),
                        float(rng.uniform(0.1, 0.3)),
                        1.0,
                    ),
                )
                self.create_obstacle(cfg)
                count += 1

        return self.obstacle_ids

    def setup_moving(self, configs: Sequence[MovingObstacleConfig]) -> List[int]:
        """Spawn kinematic moving obstacles at their t=0 trajectory position.

        Bodies have mass 0 and real box collision geometry; ``update_moving``
        repositions them each step. They are registered in ``obstacle_ids``
        so contact/collision checks and ray sensors treat them like any other
        obstacle.
        """
        ids: List[int] = []
        for cfg in configs:
            collision_id = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=list(cfg.half_extents),
                physicsClientId=self.client_id,
            )
            visual_id = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=list(cfg.half_extents),
                rgbaColor=list(cfg.color),
                physicsClientId=self.client_id,
            )
            obstacle = MovingObstacle(cfg)
            body_id = p.createMultiBody(
                baseMass=0,  # kinematic
                baseCollisionShapeIndex=collision_id,
                baseVisualShapeIndex=visual_id,
                basePosition=list(obstacle.position),
                physicsClientId=self.client_id,
            )
            obstacle.body_id = body_id
            self.obstacle_ids.append(body_id)
            self.moving_obstacles.append(obstacle)
            ids.append(body_id)
        return ids

    def update_moving(self, t: float, enabled: bool = True) -> None:
        """Move every moving obstacle to its analytic position at time t.

        When ``enabled`` is False the obstacles hold their current position
        (dynamic motion disabled, static geometry remains).
        """
        for obstacle in self.moving_obstacles:
            if enabled:
                obstacle.position = obstacle.position_at(t)
                obstacle.velocity = obstacle.velocity_at(t)
            else:
                obstacle.velocity = (0.0, 0.0, 0.0)
            p.resetBasePositionAndOrientation(
                obstacle.body_id,
                list(obstacle.position),
                [0.0, 0.0, 0.0, 1.0],
                physicsClientId=self.client_id,
            )

    def get_moving_positions(self) -> List[Tuple[float, float, float]]:
        """Current position of every moving obstacle (for telemetry/logs)."""
        return [obs.position for obs in self.moving_obstacles]

    def clear_obstacles(self) -> None:
        """Remove all obstacles from PyBullet physics world."""
        for oid in self.obstacle_ids:
            try:
                p.removeBody(oid, physicsClientId=self.client_id)
            except Exception:
                pass
        self.obstacle_ids.clear()
        self.obstacle_configs.clear()
        self.moving_obstacles.clear()

    def get_obstacle_positions(self) -> List[Tuple[float, float, float]]:
        """Return list of obstacle positions."""
        return [cfg.position for cfg in self.obstacle_configs]

    def describe(self) -> Dict[str, List[Dict[str, Any]]]:
        """JSON-safe geometry of every obstacle for external visualization.

        Values mirror the actual PyBullet bodies (same configs the collision
        shapes were built from); moving obstacles carry their closed-form
        trajectory parameters so a viewer can reproduce their exact path.
        """
        return {
            "static": [
                {
                    "position": [float(v) for v in cfg.position],
                    "half_extents": [float(v) for v in cfg.half_extents],
                }
                for cfg in self.obstacle_configs
            ],
            "moving": [
                {
                    "half_extents": [float(v) for v in obs.config.half_extents],
                    "axis": [float(v) for v in obs.config.axis],
                    "amplitude": float(obs.config.amplitude),
                    "period": float(obs.config.period),
                    "phase": float(obs.config.phase),
                }
                for obs in self.moving_obstacles
            ],
        }

    def is_obstacle(self, body_id: int) -> bool:
        """Check if a body ID belongs to managed obstacles."""
        return body_id in self.obstacle_ids
