"""Reproducible ACMFDS test scenarios (A-E).

Every scenario fixes obstacle layout, wind, battery start, goal, and seed so
runs are directly comparable. Scenarios only change the *environment and
initial conditions* — never ACMFDS thresholds, which stay at documented
defaults in every scenario.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from simulation.battery import BatteryConfig
from simulation.obstacles import MovingObstacleConfig, ObstacleConfig
from simulation.wind import WindConfig


@dataclass
class Scenario:
    """One reproducible demonstration scenario."""

    name: str
    description: str
    seed: int = 42
    start: Tuple[float, float] = (-4.0, 0.0)
    goal: Tuple[float, float] = (4.5, 0.0)
    cruise_altitude: float = 2.0
    obstacles: List[ObstacleConfig] = field(default_factory=list)
    moving_obstacles: List[MovingObstacleConfig] = field(default_factory=list)
    randomize_obstacles: bool = False
    num_random_obstacles: int = 3
    wind: WindConfig = field(default_factory=WindConfig)
    battery: BatteryConfig = field(default_factory=BatteryConfig)


def _pillar(x: float, y: float, height: float, half: float = 0.35) -> ObstacleConfig:
    """Axis-aligned square pillar spanning z in [0, height] (true height)."""
    return ObstacleConfig(
        position=(x, y, height / 2.0),
        half_extents=(half, half, height / 2.0),
        color=(0.85, 0.15, 0.15, 1.0),
    )


SCENARIOS = {
    "normal": Scenario(
        name="normal",
        description=(
            "A: Normal flight — adequate battery, low wind, no obstacle in the "
            "direct route. Demonstrates takeoff, hover, cruise, slowdown, landing."
        ),
        wind=WindConfig(enabled=True, base_speed=0.5, direction=(1.0, 0.0, 0.0)),
    ),
    "obstacle": Scenario(
        name="obstacle",
        description=(
            "B: Obstacle ahead — a known pillar blocks the planned straight route. "
            "Sensor must detect it; ACMFDS must issue AVOID_OBSTACLE and the "
            "actuation layer must fly a clear route. Collision outcome is logged."
        ),
        obstacles=[
            _pillar(-0.5, 0.0, 3.0, 0.4),   # directly on the route, taller than cruise alt
            _pillar(1.8, 0.6, 2.6, 0.35),   # secondary obstacle past the detour
        ],
    ),
    "low-battery": Scenario(
        name="low-battery",
        description=(
            "C: Low battery — intentionally reduced initial charge. ACMFDS must "
            "recognize the energy context and demonstrate energy conservation "
            "followed by a safe landing."
        ),
        battery=BatteryConfig(initial_charge_percent=32.0),
        obstacles=[_pillar(0.5, 1.2, 2.6, 0.35)],
    ),
    "strong-wind": Scenario(
        name="strong-wind",
        description=(
            "D: Strong wind — reproducible steady+gust disturbance. The physical "
            "drone must respond; ACMFDS must recognize WIND/STRONG_WIND context "
            "and command a validated wind response."
        ),
        wind=WindConfig(
            enabled=True,
            base_speed=4.5,
            direction=(0.0, 1.0, 0.0),   # crosswind from the south
            gust_amplitude=1.5,
            gust_period=7.0,
            seed=42,
        ),
        obstacles=[_pillar(0.0, 1.5, 2.6, 0.35)],  # downwind: wind pushes drone toward it
    ),
    "combined": Scenario(
        name="combined",
        description=(
            "E: Combined conditions — obstacle present, battery low, wind active. "
            "Demonstrates explicit priority handling: collision safety wins over "
            "energy, which wins over mission progress."
        ),
        battery=BatteryConfig(initial_charge_percent=28.0),
        obstacles=[_pillar(-0.5, 0.1, 3.0, 0.4)],
        wind=WindConfig(
            enabled=True,
            base_speed=3.5,
            direction=(0.0, 1.0, 0.0),
            gust_amplitude=1.0,
            gust_period=6.0,
            seed=7,
        ),
    ),
    "moving-obstacles": Scenario(
        name="moving-obstacles",
        description=(
            "F: Dynamic obstacles — static pillars plus two genuinely moving "
            "obstacles (kinematic boxes) sweeping across the cruise corridor on "
            "reproducible analytic trajectories. ACMFDS evaluates LEFT/RIGHT/"
            "FORWARD/CLIMB candidates from live sensing, rejects unsafe ones "
            "with explicit reasons, and the executed maneuver is verified by "
            "the logged positions and collision outcome."
        ),
        obstacles=[
            _pillar(-2.0, 1.2, 2.8, 0.35),   # static, left of the early corridor
            _pillar(3.0, -1.5, 2.4, 0.35),   # static, right side, clear of the goal approach
        ],
        moving_obstacles=[
            MovingObstacleConfig(
                center=(-0.6, 0.0, 1.2),
                half_extents=(0.35, 0.35, 1.2),
                axis=(0.0, 1.0, 0.0),
                amplitude=2.2,
                period=7.0,
                phase=-1.71,   # y=0 crossing timed to the drone arrival (~t 8.9 s)
            ),
            MovingObstacleConfig(
                center=(2.2, 0.0, 1.2),
                half_extents=(0.30, 0.30, 1.2),
                axis=(0.0, 1.0, 0.0),
                amplitude=1.9,
                period=12.0,
                phase=0.26,    # crossing at t~11.5 s: visible ahead during approach,
                               # corridor clear before the drone arrives (~13.3 s)
            ),
        ],
        wind=WindConfig(enabled=True, base_speed=0.5, direction=(1.0, 0.0, 0.0)),
    ),
}

SCENARIO_NAMES = list(SCENARIOS.keys())


def get_scenario(name: str) -> Scenario:
    """Fetch a scenario by name with a helpful error."""
    if name not in SCENARIOS:
        raise KeyError(f"Unknown scenario '{name}'. Choose from: {', '.join(SCENARIO_NAMES)}")
    return SCENARIOS[name]
