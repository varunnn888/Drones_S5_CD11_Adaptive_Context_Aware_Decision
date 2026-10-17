"""MuJoCo-backed ACMFDS quadrotor simulation.

The quadrotor is advanced with MuJoCo rigid-body dynamics and a simple
position/velocity controller. GPS fields are simulated from the local world
coordinates and the configurable origin in config.py.
"""
from __future__ import annotations

import math
from typing import Iterable

import mujoco
import numpy as np

from config import (
    BATTERY_START,
    DESTINATION,
    GPS_ORIGIN_ALT_M,
    GPS_ORIGIN_LAT,
    GPS_ORIGIN_LON,
    SIM_HZ,
    START,
    local_xy_to_latlon,
)
from data_model import DataDrivenRiskModel, adaptive_weights
from planner import (
    MAXX,
    MAXY,
    MINX,
    MINY,
    OBSTACLES,
    blocked,
    evaluate_path,
    nearest_obstacle_distance,
    plan_xy,
    rank_paths,
)


BUILDINGS = [
    # x, y, half-width, half-depth, height. Decorative, non-colliding city blocks.
    (1.2, 4.3, 0.72, 0.82, 2.15),
    (2.0, 8.3, 0.66, 0.72, 2.55),
    (3.7, 7.3, 0.72, 0.65, 1.85),
    (6.0, 9.6, 0.82, 0.62, 2.40),
    (8.8, 11.0, 0.76, 0.60, 1.75),
    (12.2, 8.0, 0.64, 0.90, 2.80),
    (11.7, 4.1, 0.66, 0.72, 2.10),
    (9.3, 1.0, 0.80, 0.62, 1.45),
    (5.7, 0.7, 0.84, 0.60, 1.75),
    (0.9, 1.8, 0.64, 0.70, 2.25),
    (6.6, 5.1, 0.56, 0.65, 2.15),
]

# Dynamic obstacles move along predictable paths inside the test arena.
# Each tuple: (base_x, base_y, half_width, half_depth, height,
#              axis, amplitude_m, angular_rate_rad_s, phase_rad).
DYNAMIC_OBSTACLE_SPECS = [
    # High-visibility patrol hazards; each moves along one axis in the expanded arena.
    # tuple: base_x, base_y, half_x, half_y, height, axis, amplitude, angular rate, phase.
    (2.2, 6.5, 0.50, 0.50, 3.20, "x", 1.00, 0.42, 0.00),
    (10.5, 3.0, 0.50, 0.50, 3.05, "y", 0.90, 0.34, 1.10),
]

PATH_MARKER_COUNT = 64
DRONE_MASS_KG = 1.2


def _dynamic_obstacle_position(spec: tuple, sim_time: float) -> tuple[float, float]:
    bx, by, _hx, _hy, _height, axis, amplitude, angular_rate, phase = spec
    offset = float(amplitude) * math.sin(float(angular_rate) * float(sim_time) + float(phase))
    if axis == "x":
        return float(bx + offset), float(by)
    return float(bx), float(by + offset)


def _build_mjcf() -> str:
    """Construct the urban test world, obstacle cylinders and quadrotor."""
    building_geoms: list[str] = []
    palettes = [
        "0.20 0.28 0.37 1",
        "0.24 0.32 0.42 1",
        "0.16 0.24 0.33 1",
        "0.27 0.35 0.45 1",
    ]
    for i, (x, y, hx, hy, height) in enumerate(BUILDINGS):
        # Visual buildings are deliberately non-colliding. The red cylinders
        # below are the collision hazards used by both the controller/planner.
        building_geoms.append(
            f'<geom name="building_{i}" type="box" pos="{x:.3f} {y:.3f} {height / 2:.3f}" '
            f'size="{hx:.3f} {hy:.3f} {height / 2:.3f}" rgba="{palettes[i % len(palettes)]}" '
            'contype="0" conaffinity="0"/>'
        )

    obstacle_geoms: list[str] = []
    for i, (x, y, hx, hy, height) in enumerate(OBSTACLES):
        radius = max(float(hx), float(hy))
        obstacle_geoms.append(
            f'<geom name="obstacle_{i}" type="cylinder" pos="{x:.3f} {y:.3f} {height / 2:.3f}" '
            f'size="{radius:.3f} {height / 2:.3f}" rgba="0.88 0.12 0.18 0.92" '
            'friction="0.9 0.1 0.1" material="hazardMat"/>'
        )

    dynamic_obstacle_bodies = []
    for i, spec in enumerate(DYNAMIC_OBSTACLE_SPECS):
        x, y = _dynamic_obstacle_position(spec, 0.0)
        _bx, _by, hx, hy, height, _axis, _amp, _rate, _phase = spec
        radius = max(float(hx), float(hy))
        # Tall, bright orange moving column with a yellow hazard band and beacon.
        # The main cylinder has collision; the band/beacon are visual-only accents.
        dynamic_obstacle_bodies.append(
            f'<body name="dynamic_obstacle_body_{i}" mocap="true" pos="{x:.3f} {y:.3f} 0">'
            f'<geom name="dynamic_obstacle_{i}" type="cylinder" '
            f'pos="0 0 {height / 2:.3f}" size="{radius:.3f} {height / 2:.3f}" '
            'rgba="1.0 0.34 0.035 1" material="dynamicMat" '
            'friction="0.9 0.1 0.1"/>'
            f'<geom name="dynamic_band_{i}" type="cylinder" '
            f'pos="0 0 {height * 0.70:.3f}" size="{radius + 0.025:.3f} 0.065" '
            'rgba="1.0 0.86 0.12 1" contype="0" conaffinity="0"/>'
            f'<geom name="dynamic_beacon_{i}" type="sphere" '
            f'pos="0 0 {height + 0.14:.3f}" size="0.14" '
            'rgba="1.0 0.92 0.32 1" contype="0" conaffinity="0"/>'
            '</body>'
        )

    path_markers = []
    for i in range(PATH_MARKER_COUNT):
        path_markers.append(
            f'<body name="path_marker_{i}" mocap="true" pos="0 0 0.075">'
            '<geom type="sphere" size="0.045" rgba="0.15 0.72 1.0 0.90" '
            'contype="0" conaffinity="0" material="pathMat"/>'
            '</body>'
        )

    start_x, start_y, _ = START
    goal_x, goal_y, _ = DESTINATION
    return f'''<mujoco model="ACMFDS Autonomous Quadrotor">
  <compiler angle="radian" coordinate="local" inertiafromgeom="false"/>
  <option timestep="{1.0 / SIM_HZ:.8f}" gravity="0 0 -9.81" integrator="RK4" iterations="80"/>
  <size njmax="5000" nconmax="1000"/>
  <visual>
    <!-- Give the offscreen renderer enough framebuffer space for the 900px view. -->
    <global offwidth="1200" offheight="900"/>
    <headlight ambient="0.40 0.46 0.55" diffuse="0.78 0.83 0.92" specular="0.22 0.22 0.22"/>
    <quality shadowsize="2048" offsamples="4"/>
    <map znear="0.02" zfar="80" fogstart="12" fogend="28"/>
  </visual>
  <default>
    <joint damping="0.65" armature="0.015"/>
    <geom condim="3" friction="0.8 0.1 0.1" solref="0.008 1" solimp="0.9 0.95 0.001"/>
    <site size="0.03" rgba="0.1 0.8 1 1"/>
  </default>
  <asset>
    <texture name="groundTex" type="2d" builtin="checker" width="512" height="512"
      rgb1="0.10 0.15 0.20" rgb2="0.14 0.20 0.26"/>
    <material name="floorMat" texture="groundTex" texrepeat="20 18" reflectance="0.16"/>
    <material name="buildingMat" rgba="0.22 0.30 0.40 1" reflectance="0.20"/>
    <material name="hazardMat" rgba="0.88 0.12 0.18 0.92" reflectance="0.28" emission="0.15"/>
    <material name="dynamicMat" rgba="1.0 0.52 0.12 0.96" reflectance="0.20" emission="0.15"/>
    <material name="pathMat" rgba="0.15 0.72 1.0 0.90" emission="0.30"/>
    <material name="startMat" rgba="0.04 0.28 1.0 1" emission="0.30"/>
    <material name="targetMat" rgba="0.06 0.95 0.32 1" emission="0.30"/>
    <material name="droneMat" rgba="0.10 0.13 0.17 1" reflectance="0.55"/>
  </asset>
  <worldbody>
    <light name="keyLight" pos="6 -5 17" dir="0.1 0.3 -1" diffuse="0.95 0.95 1" specular="0.3 0.3 0.3"/>
    <light name="fillLight" pos="13 10 14" dir="-0.4 -0.3 -1" diffuse="0.38 0.50 0.72" specular="0.15 0.15 0.15"/>
    <geom name="ground" type="plane" pos="6.0 5.0 0" size="24 22 0.1" material="floorMat"/>
    <geom name="start_pad" type="cylinder" pos="{start_x:.3f} {start_y:.3f} 0.045" size="0.28 0.045" material="startMat" contype="0" conaffinity="0"/>
    <geom name="target_pad" type="cylinder" pos="{goal_x:.3f} {goal_y:.3f} 0.045" size="0.30 0.045" material="targetMat" contype="0" conaffinity="0"/>
    {''.join(building_geoms)}
    {''.join(obstacle_geoms)}
    {''.join(dynamic_obstacle_bodies)}
    <body name="drone" pos="{START[0]} {START[1]} {START[2]}">
      <freejoint name="drone_free"/>
      <inertial pos="0 0 0" mass="{DRONE_MASS_KG}" diaginertia="0.025 0.025 0.045"/>
      <geom name="drone_core" type="ellipsoid" size="0.24 0.16 0.075" material="droneMat"/>
      <geom name="arm_x" type="capsule" fromto="-0.43 0 0 0.43 0 0" size="0.035" rgba="0.12 0.15 0.18 1"/>
      <geom name="arm_y" type="capsule" fromto="0 -0.43 0 0 0.43 0" size="0.035" rgba="0.12 0.15 0.18 1"/>
      <geom name="rotor_1" type="cylinder" pos="0.36 0.36 0.025" size="0.15 0.018" rgba="0.05 0.07 0.09 0.96"/>
      <geom name="rotor_2" type="cylinder" pos="-0.36 0.36 0.025" size="0.15 0.018" rgba="0.05 0.07 0.09 0.96"/>
      <geom name="rotor_3" type="cylinder" pos="0.36 -0.36 0.025" size="0.15 0.018" rgba="0.05 0.07 0.09 0.96"/>
      <geom name="rotor_4" type="cylinder" pos="-0.36 -0.36 0.025" size="0.15 0.018" rgba="0.05 0.07 0.09 0.96"/>
      <geom type="sphere" pos="0.16 0 0.07" size="0.05" rgba="1 0.12 0.12 1" contype="0" conaffinity="0"/>
      <geom type="sphere" pos="-0.16 0 0.07" size="0.05" rgba="0.1 0.5 1 1" contype="0" conaffinity="0"/>
    </body>
    {''.join(path_markers)}
  </worldbody>
</mujoco>'''


class QuadrotorSimulation:
    """Stateful MuJoCo simulation API used by dashboard.py."""

    def __init__(self) -> None:
        self.destination = tuple(float(v) for v in DESTINATION)
        self.requested_destination = tuple(self.destination)
        self.model = mujoco.MjModel.from_xml_string(_build_mjcf())
        self.model.opt.timestep = 1.0 / SIM_HZ
        self.data = mujoco.MjData(self.model)
        self.target_geom_id = int(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "target_pad")
        )
        self.drone_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "drone"
        )
        self.drone_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "drone_free"
        )
        self.qpos_address = int(self.model.jnt_qposadr[self.drone_joint_id])
        self.dof_address = int(self.model.jnt_dofadr[self.drone_joint_id])
        self.path_marker_mocap_ids = []
        for index in range(PATH_MARKER_COUNT):
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"path_marker_{index}"
            )
            self.path_marker_mocap_ids.append(int(self.model.body_mocapid[body_id]))

        self.dynamic_obstacle_mocap_ids: list[int] = []
        self.dynamic_obstacle_geom_ids: list[int] = []
        self.dynamic_obstacle_aux_geom_ids: list[tuple[int, int]] = []
        for index in range(len(DYNAMIC_OBSTACLE_SPECS)):
            body_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"dynamic_obstacle_body_{index}"
            )
            geom_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, f"dynamic_obstacle_{index}"
            )
            band_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, f"dynamic_band_{index}"
            )
            beacon_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, f"dynamic_beacon_{index}"
            )
            self.dynamic_obstacle_mocap_ids.append(int(self.model.body_mocapid[body_id]))
            self.dynamic_obstacle_geom_ids.append(int(geom_id))
            self.dynamic_obstacle_aux_geom_ids.append((int(band_id), int(beacon_id)))

        # Runtime specs are per simulation instance and can be customized from the dashboard.
        # Keep the module constants only as initial MJCF defaults.
        self.dynamic_obstacle_specs = [tuple(spec) for spec in DYNAMIC_OBSTACLE_SPECS]
        self.last_moved_dynamic_index = 0

        self.static_obstacle_geom_ids = [
            int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"obstacle_{i}"))
            for i in range(len(OBSTACLES))
        ]
        self.risk_model = DataDrivenRiskModel()
        self.static_obstacles = list(OBSTACLES)
        self.last_moved_static_index = 0
        self.dynamic_obstacles: list[tuple[float, float, float, float, float]] = []
        self.static_enabled = True
        self.dynamic_enabled = True
        # The dashboard pauses physics while the user drags a static obstacle.
        self.obstacle_editing = False
        self.obstacles: list[tuple[float, float, float, float, float]] = []
        self.buildings = BUILDINGS
        self.running = True
        self.sim_time = 0.0
        self._last_dynamic_replan_time = 0.0
        self._dynamic_replan_interval = 3.0
        # Environmental conditions are user-adjustable from the dashboard.
        # Wind direction is a world-frame angle: 0° pushes toward +X/east,
        # 90° toward +Y/north. This is a simulation disturbance, not weather data.
        self.wind_speed_mps = 0.0
        self.wind_direction_deg = 0.0
        self.battery_start_setting = float(BATTERY_START)
        self.home_return_active = False
        self.landing_started = False
        self.emergency_landing = False
        self.condition_message = "Calm wind · full battery"
        self.reset()

    def _active_obstacles(self) -> list[tuple[float, float, float, float, float]]:
        active = []
        if self.static_enabled:
            active.extend(self.static_obstacles)
        if self.dynamic_enabled:
            active.extend(self.dynamic_obstacles)
        self.obstacles = active
        return active

    def _make_initial_plan(self) -> None:
        start_xy = (float(START[0]), float(START[1]))
        goal_xy = (float(self.destination[0]), float(self.destination[1]))
        wind_ratio = float(np.clip(self.wind_speed_mps / 12.0, 0.0, 1.0))
        battery_ratio = float(np.clip(self.battery / 100.0, 0.0, 1.0))
        initial_weights = {
            "safety": 0.30 + 0.12 * wind_ratio,
            "energy": 0.15 + 0.22 * (1.0 - battery_ratio) + 0.06 * wind_ratio,
            "time": 0.15,
            "tracking": 0.15,
            "mission": 0.25,
        }
        total_weight = sum(initial_weights.values())
        initial_weights = {key: value / total_weight for key, value in initial_weights.items()}
        try:
            self.path_ranking = rank_paths(
                start_xy,
                goal_xy,
                obstacles=self._active_obstacles(),
                weights=initial_weights,
                battery=self.battery,
                speed=max(0.65, 1.4 - 0.035 * self.wind_speed_mps),
            )
        except Exception as exc:
            # Keep the UI usable but visibly explain a planning failure.
            self.path_ranking = []
            self.planner_warning = str(exc)
        if not self.path_ranking:
            self.path_ranking = [
                {
                    "rank": 1,
                    "path_id": 1,
                    "profile": "DIRECT FALLBACK",
                    "preference": 0.0,
                    "path": [start_xy, goal_xy],
                    "score": 1.0,
                    "length": math.dist(start_xy, goal_xy),
                    "clearance": 0.0,
                    "safety": 1.0,
                    "waypoints": 2,
                    "planner_warning": getattr(self, "planner_warning", "No collision-free route was found."),
                }
            ]
        self.path_ranking.sort(key=lambda item: float(item.get("score", 1.0)))
        for rank, item in enumerate(self.path_ranking, start=1):
            item["rank"] = rank
        best = self.path_ranking[0]
        self._activate_ranked_route(best, route_points=best.get("path", [start_xy, goal_xy]))
        self.replan_reason = "INITIAL SAFE ROUTE"

    def _activate_ranked_route(self, item: dict, route_points=None) -> None:
        """Activate a ranked route; route_points may begin at the current position."""
        self.selected_path_rank = int(item.get("rank", 1))
        self.selected_path_id = int(item.get("path_id", 1))
        self.selected_path_profile = str(item.get("profile", "BALANCED"))
        self.selected_path_score = float(item.get("score", 0.0))
        points = route_points if route_points is not None else item.get("path", [START[:2], self.destination[:2]])
        self.waypoints = [
            (float(point[0]), float(point[1]), float(self.destination[2]))
            for point in points
        ]
        if not self.waypoints:
            self.waypoints = [START, self.destination]
        self.wp_index = 0
        self._update_path_markers()

    def select_route_by_id(self, path_id: int) -> bool:
        """Choose a route profile from the options list and plan from current position."""
        choice = next(
            (item for item in self.path_ranking if int(item.get("path_id", -1)) == int(path_id)),
            None,
        )
        if choice is None:
            return False
        preference = float(choice.get("preference", 0.0))
        obstacles = self._active_obstacles()
        try:
            path = plan_xy(
                (float(self.pos[0]), float(self.pos[1])),
                (float(self.destination[0]), float(self.destination[1])),
                obstacles=obstacles,
                clearance_weight=0.045 + 0.025 * abs(preference) + 0.004 * self.wind_speed_mps,
                preference=preference,
            )
            metrics = evaluate_path(
                path,
                obstacles=obstacles,
                weights=self.weights,
                battery=self.battery,
                speed=max(self.speed, 0.8),
            )
        except Exception as exc:
            self.replan_reason = f"ROUTE NOT AVAILABLE: {str(exc)[:48]}"
            return False

        choice.update(metrics)
        choice["path"] = path
        self._activate_ranked_route(choice, route_points=path)
        self.replan_reason = f"PROFILE SELECTED: {self.selected_path_profile}"
        return True

    @property
    def return_home_threshold(self) -> float:
        """Reserve threshold rises with wind because return travel costs more energy."""
        return float(min(43.0, 25.0 + 1.5 * self.wind_speed_mps))

    def set_flight_conditions(
        self, wind_speed_mps: float, wind_direction_deg: float, battery_percent: float
    ) -> str:
        """Apply simulated wind and battery conditions, then adapt the mission.

        Wind is modelled as an external force on the quadrotor. Battery changes
        immediately affect energy-aware weights/speed and may trigger return-home
        or emergency landing behaviours. These are simulation behaviours only.
        """
        self.wind_speed_mps = float(np.clip(float(wind_speed_mps), 0.0, 12.0))
        self.wind_direction_deg = float(float(wind_direction_deg) % 360.0)
        self.battery_start_setting = float(np.clip(float(battery_percent), 0.0, 100.0))
        self.battery = self.battery_start_setting

        # Re-arming after a previous low-battery stop is permitted when a new,
        # usable battery value is applied; START AGAIN still resets the mission.
        if self.status in ("LOW BATTERY", "LANDED SAFELY", "EMERGENCY LANDED") and self.battery > 8.0:
            self.running = True
            self.status = "RUNNING"
            self.home_return_active = False
            self.landing_started = False
            self.emergency_landing = False
            self.destination = tuple(self.requested_destination)
            self.model.geom_pos[self.target_geom_id, 0] = float(self.destination[0])
            self.model.geom_pos[self.target_geom_id, 1] = float(self.destination[1])

        # If battery recovers before the home-landing phase, resume the requested
        # mission goal rather than continue an unnecessary return-home manoeuvre.
        if self.home_return_active and not self.landing_started and self.battery >= 35.0:
            self.home_return_active = False
            self.destination = tuple(self.requested_destination)
            self.model.geom_pos[self.target_geom_id, 0] = float(self.destination[0])
            self.model.geom_pos[self.target_geom_id, 1] = float(self.destination[1])
            self.status = "RUNNING"
            self.phase = "NAVIGATING"
            self.decision = "FORWARD"

        mujoco.mj_forward(self.model, self.data)
        self._update_telemetry()
        if self.battery <= 8.0 and self.status not in ("LANDED SAFELY", "EMERGENCY LANDED"):
            self.running = True
            self._begin_emergency_landing()
        elif self.battery <= self.return_home_threshold and self.running and not self.home_return_active:
            self._begin_return_home()
        else:
            self.replan(reason="WIND / BATTERY CONDITIONS UPDATED")

        wind_label = "CALM" if self.wind_speed_mps < 2.0 else (
            "BREEZY" if self.wind_speed_mps < 6.0 else (
                "STRONG WIND" if self.wind_speed_mps < 9.0 else "SEVERE WIND"
            )
        )
        if self.emergency_landing:
            response = "emergency landing initiated"
        elif self.home_return_active:
            response = "return-to-home initiated"
        elif self.battery <= 35.0:
            response = "energy-saving mode"
        elif self.wind_speed_mps >= 8.0:
            response = "stabilising against wind"
        else:
            response = "adaptive navigation active"
        self.condition_message = (
            f"{wind_label} · {self.wind_speed_mps:.1f} m/s @ {self.wind_direction_deg:.0f}° · "
            f"battery {self.battery:.0f}% · {response}"
        )
        return self.condition_message

    def _set_visual_target(self) -> None:
        """Keep the green target pad aligned to the active mission/landing target."""
        self.model.geom_pos[self.target_geom_id, 0] = float(self.destination[0])
        self.model.geom_pos[self.target_geom_id, 1] = float(self.destination[1])
        mujoco.mj_forward(self.model, self.data)

    def _begin_return_home(self) -> None:
        """Switch the active target to the blue home pad and replan safely."""
        if self.home_return_active or self.emergency_landing:
            return
        self.home_return_active = True
        self.landing_started = False
        self.destination = tuple(float(v) for v in START)
        self.status = "RETURNING HOME"
        self.phase = "RETURNING HOME"
        self.context = "LOW BATTERY"
        self.decision = "RETURN HOME"
        self.running = True
        self._set_visual_target()
        self.replan(reason="LOW BATTERY: RETURNING HOME")
        self._update_path_markers()

    def _begin_home_landing(self) -> None:
        """Begin a slow vertical descent once the drone reaches the home pad."""
        if self.landing_started:
            return
        self.landing_started = True
        self.destination = (float(START[0]), float(START[1]), 0.24)
        self.waypoints = [self.destination]
        self.wp_index = 0
        self.status = "RETURNING HOME"
        self.phase = "LANDING AT HOME"
        self.context = "HOME REACHED"
        self.decision = "LAND"
        self._set_visual_target()
        self._update_path_markers()

    def _begin_emergency_landing(self) -> None:
        """Command a vertical landing at the current XY location for critical battery."""
        if self.emergency_landing:
            return
        self.home_return_active = False
        self.emergency_landing = True
        self.landing_started = True
        landing_x = float(self.pos[0])
        landing_y = float(self.pos[1])
        self.destination = (landing_x, landing_y, 0.24)
        self.waypoints = [self.destination]
        self.wp_index = 0
        self.running = True
        self.status = "EMERGENCY LANDING"
        self.phase = "EMERGENCY LANDING"
        self.context = "CRITICAL BATTERY"
        self.decision = "EMERGENCY LAND"
        self._set_visual_target()
        self._update_path_markers()

    def _finish_landing(self) -> None:
        """Stop the simulated mission after the controlled descent is complete."""
        self.running = False
        self.status = "EMERGENCY LANDED" if self.emergency_landing else "LANDED SAFELY"
        self.phase = "LANDED"
        self.context = "LANDED"
        self.decision = "LAND"
        self._sample_history(force=True)

    def set_obstacle_modes(self, static_enabled: bool, dynamic_enabled: bool) -> None:
        """Enable/disable static and moving hazards in both physics and planning."""
        self.static_enabled = bool(static_enabled)
        self.dynamic_enabled = bool(dynamic_enabled)
        self._apply_obstacle_collision_modes()
        self._active_obstacles()
        self.replan(reason="OBSTACLE OPTIONS CHANGED")

    def move_static_obstacle(
        self, index: int, x: float, y: float, *, replan: bool = False
    ) -> bool:
        """Move one static obstacle in both MuJoCo and the planner's world model.

        Coordinates are clamped to the expanded arena. During a drag, callers can
        pass ``replan=False`` and request one new route when the drag is released.
        """
        if not 0 <= int(index) < len(self.static_obstacles):
            return False

        index = int(index)
        _old_x, _old_y, hx, hy, height = self.static_obstacles[index]
        # Keep the obstacle body within the arena while leaving a small visual gap
        # from the edges. Its original size and height remain unchanged.
        radius_x = max(float(hx), float(hy))
        radius_y = radius_x
        x = float(np.clip(float(x), MINX + radius_x + 0.10, MAXX - radius_x - 0.10))
        y = float(np.clip(float(y), MINY + radius_y + 0.10, MAXY - radius_y - 0.10))

        # Do not allow a manual drag to place a solid hazard on top of the drone
        # or directly over the destination pad. This keeps the edit itself safe.
        current_x, current_y = float(self.pos[0]), float(self.pos[1])
        goal_x, goal_y = float(self.destination[0]), float(self.destination[1])
        if (
            abs(x - current_x) < radius_x + 0.62
            and abs(y - current_y) < radius_y + 0.62
        ):
            return False
        if (
            abs(x - goal_x) < radius_x + 0.38
            and abs(y - goal_y) < radius_y + 0.38
        ):
            return False

        old = self.static_obstacles[index]
        if abs(old[0] - x) < 1e-4 and abs(old[1] - y) < 1e-4:
            return False

        updated = (x, y, float(hx), float(hy), float(height))
        self.static_obstacles[index] = updated
        self.last_moved_static_index = index
        # OBSTACLES is the planner module's shared defaults list; keep it in sync
        # so helper calls that omit an explicit obstacle list see the moved object.
        OBSTACLES[index] = updated

        geom_id = self.static_obstacle_geom_ids[index]
        self.model.geom_pos[geom_id, 0] = x
        self.model.geom_pos[geom_id, 1] = y
        mujoco.mj_forward(self.model, self.data)
        self._active_obstacles()
        self._update_path_markers()

        if replan:
            self.replan(reason="STATIC OBSTACLE REPOSITIONED")
        return True

    def configure_dynamic_obstacle(
        self,
        index: int,
        base_x: float,
        base_y: float,
        *,
        axis: str | None = None,
        amplitude: float | None = None,
        angular_rate: float | None = None,
        replan: bool = False,
    ) -> bool:
        """Customize a moving obstacle's trajectory and update MuJoCo + planner.

        ``base_x/base_y`` are the centre of the patrol track, not necessarily the
        obstacle's instantaneous position. Amplitude is measured in metres and
        angular_rate is in radians/second. The entire track is kept inside the arena.
        """
        if not 0 <= int(index) < len(self.dynamic_obstacle_specs):
            return False
        index = int(index)
        old = self.dynamic_obstacle_specs[index]
        _old_x, _old_y, hx, hy, height, old_axis, old_amp, old_rate, phase = old
        new_axis = (axis or old_axis).lower()
        if new_axis not in ("x", "y"):
            return False
        new_amp = float(np.clip(old_amp if amplitude is None else amplitude, 0.0, 4.0))
        new_rate = float(np.clip(old_rate if angular_rate is None else angular_rate, 0.05, 1.50))

        # Keep the full motion segment inside the map with a small edge margin.
        margin_x = float(hx) + 0.12
        margin_y = float(hy) + 0.12
        if new_axis == "x":
            min_x, max_x = MINX + margin_x + new_amp, MAXX - margin_x - new_amp
            min_y, max_y = MINY + margin_y, MAXY - margin_y
        else:
            min_x, max_x = MINX + margin_x, MAXX - margin_x
            min_y, max_y = MINY + margin_y + new_amp, MAXY - margin_y - new_amp
        if min_x > max_x or min_y > max_y:
            return False
        new_x = float(np.clip(float(base_x), min_x, max_x))
        new_y = float(np.clip(float(base_y), min_y, max_y))
        updated = (
            new_x, new_y, float(hx), float(hy), float(height),
            new_axis, new_amp, new_rate, float(phase),
        )
        current_x, current_y = _dynamic_obstacle_position(updated, self.sim_time)

        # Prevent a manual edit from placing the live moving hazard directly on
        # the quadrotor or target. The rest of the patrol remains a replannable hazard.
        drone_x, drone_y = float(self.pos[0]), float(self.pos[1])
        goal_x, goal_y = float(self.destination[0]), float(self.destination[1])
        if abs(current_x - drone_x) < float(hx) + 0.58 and abs(current_y - drone_y) < float(hy) + 0.58:
            return False
        if abs(current_x - goal_x) < float(hx) + 0.38 and abs(current_y - goal_y) < float(hy) + 0.38:
            return False

        if old == updated:
            return False

        self.dynamic_obstacle_specs[index] = updated
        self.last_moved_dynamic_index = index
        self._update_dynamic_obstacles()
        mujoco.mj_forward(self.model, self.data)
        self._update_path_markers()
        if replan:
            self._last_dynamic_replan_time = self.sim_time
            self.replan(reason="DYNAMIC OBSTACLE CUSTOMIZED")
        return True

    def move_dynamic_obstacle_to_position(
        self, index: int, x: float, y: float, *, replan: bool = False
    ) -> bool:
        """Move a dynamic obstacle's live position while preserving its patrol motion."""
        if not 0 <= int(index) < len(self.dynamic_obstacle_specs):
            return False
        spec = self.dynamic_obstacle_specs[int(index)]
        _bx, _by, _hx, _hy, _height, axis, amplitude, rate, phase = spec
        offset = float(amplitude) * math.sin(float(rate) * self.sim_time + float(phase))
        if axis == "x":
            base_x, base_y = float(x) - offset, float(y)
        else:
            base_x, base_y = float(x), float(y) - offset
        return self.configure_dynamic_obstacle(
            int(index), base_x, base_y, axis=axis, amplitude=amplitude,
            angular_rate=rate, replan=replan,
        )

    def move_destination(
        self, x: float, y: float, *, replan: bool = False
    ) -> bool:
        """Move the goal pad and update the mission's live destination.

        Invalid locations (outside the safe arena, too close to the drone, or
        inside an obstacle's inflated safety margin) are rejected. Set
        ``replan=False`` during a drag and call ``replan`` once on release.
        """
        if self.home_return_active or self.emergency_landing:
            return False
        x = float(np.clip(float(x), MINX + 0.20, MAXX - 0.20))
        y = float(np.clip(float(y), MINY + 0.20, MAXY - 0.20))
        if math.hypot(x - float(self.pos[0]), y - float(self.pos[1])) < 0.55:
            return False
        active_obstacles = self._active_obstacles()
        if blocked(x, y, active_obstacles):
            return False
        old_x, old_y, goal_z = self.destination
        if abs(old_x - x) < 1e-4 and abs(old_y - y) < 1e-4:
            return False

        self.destination = (x, y, float(goal_z))
        if not self.home_return_active and not self.emergency_landing:
            self.requested_destination = tuple(self.destination)
        # The target pad is a non-colliding world geom, so update its position
        # directly to make the visual target match the planner's new goal.
        self.model.geom_pos[self.target_geom_id, 0] = x
        self.model.geom_pos[self.target_geom_id, 1] = y
        mujoco.mj_forward(self.model, self.data)

        # Allow moving the goal after a completed mission to resume navigation.
        if self.status == "MISSION COMPLETE" and self.battery > 5.0:
            self.running = True
            self.status = "RUNNING"
            self.phase = "NAVIGATING"
            self.decision = "FORWARD"

        self._update_path_markers()
        if replan:
            self.replan(reason="DESTINATION REPOSITIONED")
        return True

    def _apply_obstacle_collision_modes(self) -> None:
        for geom_id in self.static_obstacle_geom_ids:
            active = self.static_enabled
            self.model.geom_contype[geom_id] = 1 if active else 0
            self.model.geom_conaffinity[geom_id] = 1 if active else 0
            self.model.geom_rgba[geom_id] = (
                np.array([0.88, 0.12, 0.18, 0.95])
                if active else np.array([0.28, 0.32, 0.37, 0.35])
            )
        for index, geom_id in enumerate(self.dynamic_obstacle_geom_ids):
            active = self.dynamic_enabled
            self.model.geom_contype[geom_id] = 1 if active else 0
            self.model.geom_conaffinity[geom_id] = 1 if active else 0
            self.model.geom_rgba[geom_id] = (
                np.array([1.00, 0.34, 0.035, 1.00])
                if active else np.array([0.30, 0.31, 0.33, 0.30])
            )
            band_id, beacon_id = self.dynamic_obstacle_aux_geom_ids[index]
            self.model.geom_rgba[band_id] = (
                np.array([1.00, 0.86, 0.12, 1.00])
                if active else np.array([0.36, 0.34, 0.22, 0.30])
            )
            self.model.geom_rgba[beacon_id] = (
                np.array([1.00, 0.92, 0.32, 1.00])
                if active else np.array([0.36, 0.34, 0.22, 0.30])
            )

    def _update_dynamic_obstacles(self) -> None:
        current = []
        tracks = []
        for index, spec in enumerate(self.dynamic_obstacle_specs):
            x, y = _dynamic_obstacle_position(spec, self.sim_time)
            bx, by, hx, hy, height, axis, amplitude, _rate, _phase = spec
            current.append((x, y, float(hx), float(hy), float(height)))
            if axis == "x":
                tracks.append((float(bx - amplitude), float(by), float(bx + amplitude), float(by)))
            else:
                tracks.append((float(bx), float(by - amplitude), float(bx), float(by + amplitude)))
            mocap_id = self.dynamic_obstacle_mocap_ids[index]
            self.data.mocap_pos[mocap_id, :] = (x, y, 0.0)
        self.dynamic_obstacles = current
        self.dynamic_obstacle_tracks = tracks
        self._active_obstacles()

    def reset(self) -> None:
        """Restart the mission with the currently selected goal and test conditions."""
        self.destination = tuple(self.requested_destination)
        self.home_return_active = False
        self.landing_started = False
        self.emergency_landing = False
        mujoco.mj_resetData(self.model, self.data)
        self.model.geom_pos[self.target_geom_id, 0] = float(self.destination[0])
        self.model.geom_pos[self.target_geom_id, 1] = float(self.destination[1])
        self.data.qpos[self.qpos_address : self.qpos_address + 3] = np.asarray(START, dtype=float)
        self.data.qpos[self.qpos_address + 3 : self.qpos_address + 7] = np.array([1.0, 0.0, 0.0, 0.0])
        self.data.qvel[:] = 0.0
        self.data.xfrc_applied[:] = 0.0
        self.sim_time = 0.0
        self._last_dynamic_replan_time = 0.0
        self.battery = float(self.battery_start_setting)
        self.wp_index = 0
        self.running = True
        self.status = "RUNNING"
        self.phase = "NAVIGATING"
        self.context = "CLEAR"
        self.decision = "FORWARD"
        self.risk = 0.0
        self.nearest_obstacle = 0.0
        self.pos = np.asarray(START, dtype=float).copy()
        self.vel = np.zeros(3, dtype=float)
        self.speed = 0.0
        self.force_x = 0.0
        self.force_y = 0.0
        self.thrust_z = DRONE_MASS_KG * 9.81
        self.control_magnitude = self.thrust_z
        self.replan_count = 0
        self.weights = {
            "safety": 0.20,
            "energy": 0.20,
            "time": 0.20,
            "tracking": 0.20,
            "mission": 0.20,
        }
        self.history_time: list[float] = []
        self.distance_history: list[float] = []
        self.battery_history: list[float] = []
        self.speed_history: list[float] = []
        self._update_dynamic_obstacles()
        self._apply_obstacle_collision_modes()
        self._make_initial_plan()
        mujoco.mj_forward(self.model, self.data)
        self._update_path_markers()
        mujoco.mj_forward(self.model, self.data)
        self._update_telemetry()
        self._sample_history(force=True)

        if self.battery <= 8.0:
            self._begin_emergency_landing()
        elif self.battery <= self.return_home_threshold:
            self._begin_return_home()

    @property
    def gps_latitude(self) -> float:
        lat, _ = local_xy_to_latlon(self.pos[0], self.pos[1])
        return lat

    @property
    def gps_longitude(self) -> float:
        _, lon = local_xy_to_latlon(self.pos[0], self.pos[1])
        return lon

    @property
    def gps_altitude(self) -> float:
        # This is a simulated altitude above the configured simulation origin.
        return float(GPS_ORIGIN_ALT_M + self.pos[2])

    @property
    def distance_to_target(self) -> float:
        return float(np.linalg.norm(np.asarray(self.destination, dtype=float) - self.pos))

    @property
    def mission_progress(self) -> float:
        total = max(float(np.linalg.norm(np.asarray(self.destination) - np.asarray(START))), 1e-9)
        return float(np.clip(1.0 - self.distance_to_target / total, 0.0, 1.0) * 100.0)

    def _current_waypoint(self) -> np.ndarray:
        index = min(self.wp_index, len(self.waypoints) - 1)
        return np.asarray(self.waypoints[index], dtype=float)

    def _update_telemetry(self) -> None:
        p = np.asarray(self.data.qpos[self.qpos_address : self.qpos_address + 3], dtype=float)
        v = np.asarray(self.data.qvel[self.dof_address : self.dof_address + 3], dtype=float)
        self.pos = p.copy()
        self.vel = v.copy()
        self.speed = float(np.linalg.norm(v))
        self.nearest_obstacle = float(
            nearest_obstacle_distance(float(p[0]), float(p[1]), self._active_obstacles())
        )
        target_error = float(np.linalg.norm(np.asarray(self.destination, dtype=float) - p))
        tracking_error = min(1.0, float(np.linalg.norm(self._current_waypoint() - p)) / 2.0)
        learned_risk = self.risk_model.predict(
            max(self.nearest_obstacle, 0.01), self.speed, self.battery, tracking_error
        )
        wind_ratio = float(np.clip(self.wind_speed_mps / 12.0, 0.0, 1.0))
        # Wind contributes directly to the risk estimate, in addition to the
        # learned proximity/tracking/battery signal.
        self.risk = float(np.clip(0.66 * learned_risk + 0.34 * wind_ratio, 0.0, 1.0))
        self.weights = adaptive_weights(
            self.risk,
            self.battery,
            target_error,
            self.speed,
            max(0.0, 150.0 - self.sim_time),
        )
        # Emphasize energy management when wind is costly or the battery is low.
        self.weights["energy"] += 0.10 * (1.0 - float(np.clip(self.battery / 100.0, 0.0, 1.0)))
        self.weights["energy"] += 0.06 * wind_ratio
        weight_total = max(sum(self.weights.values()), 1e-9)
        self.weights = {key: float(value / weight_total) for key, value in self.weights.items()}

        if self.status in ("LANDED SAFELY", "EMERGENCY LANDED"):
            self.context = "LANDED"
            self.decision = "LAND"
            self.phase = "LANDED"
        elif self.emergency_landing:
            self.context = "CRITICAL BATTERY"
            self.decision = "EMERGENCY LAND"
            self.phase = "EMERGENCY LANDING"
        elif self.home_return_active:
            self.context = "LOW BATTERY / HOME"
            self.decision = "LAND" if self.landing_started else "RETURN HOME"
            self.phase = "LANDING AT HOME" if self.landing_started else "RETURNING HOME"
        elif self.status == "MISSION COMPLETE":
            self.context = "TARGET REACHED"
            self.decision = "HOLD"
            self.phase = "COMPLETE"
        elif self.status == "LOW BATTERY":
            self.context = "LOW BATTERY"
            self.decision = "HOLD"
            self.phase = "RETURN / HOLD"
        elif self.risk >= 0.58:
            self.context = "OBSTACLE / WIND RISK"
            self.decision = "AVOID / STABILIZE"
            self.phase = "AVOIDING"
        elif self.battery <= 35.0:
            self.context = "ENERGY CONSERVATION"
            self.decision = "SLOW / CONSERVE"
            self.phase = "ENERGY-SAVING"
        elif self.wind_speed_mps >= 8.0:
            self.context = "HIGH WIND"
            self.decision = "STABILIZE"
            self.phase = "STABILIZING"
        elif self.risk >= 0.36:
            self.context = "MONITORING"
            self.decision = "ADJUST"
            self.phase = "NAVIGATING"
        else:
            self.context = "CLEAR"
            self.decision = "FORWARD"
            self.phase = "NAVIGATING"
        self.control_magnitude = float(math.sqrt(self.force_x**2 + self.force_y**2 + self.thrust_z**2))

    def _update_path_markers(self) -> None:
        points = np.asarray(self.waypoints, dtype=float)
        if len(points) < 2:
            points = np.asarray([START, self.destination], dtype=float)
        xy = points[:, :2]
        lengths = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        total = max(float(cumulative[-1]), 1e-9)
        marker_z = 0.075
        for index, mocap_id in enumerate(self.path_marker_mocap_ids):
            distance = total * index / max(PATH_MARKER_COUNT - 1, 1)
            segment = min(int(np.searchsorted(cumulative, distance, side="right") - 1), len(points) - 2)
            segment = max(segment, 0)
            segment_length = max(float(lengths[segment]), 1e-9)
            alpha = np.clip((distance - cumulative[segment]) / segment_length, 0.0, 1.0)
            xy_point = xy[segment] + alpha * (xy[segment + 1] - xy[segment])
            self.data.mocap_pos[mocap_id, :] = (float(xy_point[0]), float(xy_point[1]), marker_z)

    def _sample_history(self, force: bool = False) -> None:
        if not force and self.history_time and self.sim_time - self.history_time[-1] < 0.20:
            return
        self.history_time.append(float(self.sim_time))
        self.distance_history.append(float(self.distance_to_target))
        self.battery_history.append(float(self.battery))
        self.speed_history.append(float(self.speed))
        # A 120-second plot window at 5 samples/sec is enough for the dashboard.
        max_samples = 610
        if len(self.history_time) > max_samples:
            self.history_time = self.history_time[-max_samples:]
            self.distance_history = self.distance_history[-max_samples:]
            self.battery_history = self.battery_history[-max_samples:]
            self.speed_history = self.speed_history[-max_samples:]

    def step(self, n_steps: int = 1) -> None:
        """Advance physics with wind disturbance and battery-aware behaviours."""
        if not self.running or self.obstacle_editing:
            return
        dt = float(self.model.opt.timestep)
        for _ in range(max(1, int(n_steps))):
            if not self.running:
                break
            self.pos = np.asarray(self.data.qpos[self.qpos_address : self.qpos_address + 3], dtype=float).copy()
            self.vel = np.asarray(self.data.qvel[self.dof_address : self.dof_address + 3], dtype=float).copy()

            while self.wp_index < len(self.waypoints) - 1:
                if np.linalg.norm(self._current_waypoint() - self.pos) > 0.36:
                    break
                self.wp_index += 1

            target = self._current_waypoint()
            error = target - self.pos
            # High winds and low battery make the policy less aggressive.
            wind_scale = max(0.62, 1.0 - 0.032 * self.wind_speed_mps)
            battery_scale = 1.0 if self.battery >= 35.0 else (0.62 + 0.38 * self.battery / 35.0)
            speed_scale = min(wind_scale, battery_scale)
            desired_velocity = 1.65 * speed_scale * error
            component_limit = 1.45 * speed_scale
            desired_velocity[:2] = np.clip(desired_velocity[:2], -component_limit, component_limit)
            horizontal_limit = max(0.55, 1.55 * speed_scale)
            if np.linalg.norm(desired_velocity[:2]) > horizontal_limit:
                desired_velocity[:2] *= horizontal_limit / np.linalg.norm(desired_velocity[:2])
            vertical_limit = max(0.35, 0.65 * speed_scale)
            desired_velocity[2] = float(np.clip(1.8 * error[2], -vertical_limit, vertical_limit))

            # Feedback controller force.
            acceleration = 2.8 * (desired_velocity - self.vel)
            accel_xy_norm = float(np.linalg.norm(acceleration[:2]))
            if accel_xy_norm > 3.2:
                acceleration[:2] *= 3.2 / accel_xy_norm
            acceleration[2] = float(np.clip(acceleration[2], -3.0, 3.0))
            controller_force = DRONE_MASS_KG * np.array(
                [acceleration[0], acceleration[1], acceleration[2] + 9.81], dtype=float
            )

            # Apply a world-frame wind force. Direction uses 0°=+X and 90°=+Y.
            wind_angle = math.radians(self.wind_direction_deg)
            wind_accel = 0.18 * self.wind_speed_mps
            wind_force = DRONE_MASS_KG * wind_accel * np.array(
                [math.cos(wind_angle), math.sin(wind_angle), 0.0], dtype=float
            )
            total_force = controller_force + wind_force
            self.data.xfrc_applied[self.drone_body_id, :] = 0.0
            self.data.xfrc_applied[self.drone_body_id, :3] = total_force
            self.force_x = float(total_force[0])
            self.force_y = float(total_force[1])
            self.thrust_z = float(total_force[2])

            mujoco.mj_step(self.model, self.data)
            self.sim_time += dt
            self._update_dynamic_obstacles()
            speed_cost = 0.48 + 0.14 * min(self.speed / 1.6, 1.0)
            wind_cost_multiplier = 1.0 + 0.025 * (self.wind_speed_mps ** 1.25)
            self.battery = max(0.0, self.battery - speed_cost * wind_cost_multiplier * dt)

            if (
                self.dynamic_enabled
                and self.sim_time - self._last_dynamic_replan_time >= self._dynamic_replan_interval
                and self.status in ("RUNNING", "RETURNING HOME")
            ):
                self._last_dynamic_replan_time = self.sim_time
                self.replan(reason="DYNAMIC OBSTACLES MOVED")
            self._update_telemetry()
            self._sample_history()
            self._update_path_markers()
            mujoco.mj_forward(self.model, self.data)

            # Critical battery: land in place. Otherwise, start RTH before the
            # critical reserve is reached, and descend only after reaching home.
            if self.battery <= 8.0 and not self.emergency_landing:
                self._begin_emergency_landing()
            elif self.battery <= self.return_home_threshold and not self.home_return_active and not self.emergency_landing:
                self._begin_return_home()

            if self.home_return_active and not self.landing_started:
                home_xy_error = math.hypot(float(self.pos[0]) - float(START[0]), float(self.pos[1]) - float(START[1]))
                home_altitude_error = abs(float(self.pos[2]) - float(START[2]))
                if home_xy_error <= 0.45 and home_altitude_error <= 0.42:
                    self._begin_home_landing()

            if (self.emergency_landing or (self.home_return_active and self.landing_started)):
                land_xy_error = math.hypot(float(self.pos[0]) - float(self.destination[0]), float(self.pos[1]) - float(self.destination[1]))
                if float(self.pos[2]) <= 0.34 and land_xy_error <= 0.48:
                    self._finish_landing()
                    break
            elif not self.home_return_active and not self.emergency_landing and self.distance_to_target <= 0.24:
                self.status = "MISSION COMPLETE"
                self.phase = "COMPLETE"
                self.decision = "HOLD"
                self.running = False
                self._sample_history(force=True)
                break

        self._update_telemetry()

    def replan(self, reason: str = "UPDATED RISK / ADAPTIVE WEIGHTS") -> None:
        """Refresh candidate routes using the currently enabled obstacles."""
        try:
            ranking = rank_paths(
                (float(self.pos[0]), float(self.pos[1])),
                (float(self.destination[0]), float(self.destination[1])),
                obstacles=self._active_obstacles(),
                weights=self.weights,
                battery=self.battery,
                speed=max(self.speed, 0.8),
            )
        except Exception as exc:
            self.replan_reason = f"NO SAFE ROUTE: {str(exc)[:48]}"
            return
        if not ranking:
            self.replan_reason = "NO SAFE ROUTE FOUND"
            return
        self.path_ranking = ranking
        for rank, item in enumerate(self.path_ranking, start=1):
            item["rank"] = rank
        # Keep the selected profile if it still exists; otherwise use the top-ranked route.
        chosen = next(
            (item for item in self.path_ranking if int(item.get("path_id", -1)) == int(self.selected_path_id)),
            self.path_ranking[0],
        )
        preference = float(chosen.get("preference", 0.0))
        try:
            path = plan_xy(
                (float(self.pos[0]), float(self.pos[1])),
                (float(self.destination[0]), float(self.destination[1])),
                obstacles=self._active_obstacles(),
                clearance_weight=0.045 + 0.025 * abs(preference) + 0.004 * self.wind_speed_mps,
                preference=preference,
            )
            chosen["path"] = path
        except Exception:
            # rank_paths already returned a valid path; use its route if the
            # profile-specific second calculation cannot be made.
            path = chosen.get("path", self.path_ranking[0].get("path", []))
        self._activate_ranked_route(chosen, route_points=path)
        self.replan_count += 1
        self.replan_reason = reason
        self._last_dynamic_replan_time = float(self.sim_time)

