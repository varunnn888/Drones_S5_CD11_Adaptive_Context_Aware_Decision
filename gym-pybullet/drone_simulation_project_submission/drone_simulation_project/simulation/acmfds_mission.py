"""ACMFDS-driven mission runner — the primary demonstration execution path.

Pipeline per control step (real physics, no teleports)::

    read PyBullet state + ray sensors
        -> update battery / wind models
        -> mission intent fills target fields of the flight state
        -> ACMFDS decides (explainable Decision)
        -> command composition (mission intent + ACMFDS constraints/veto)
        -> SMC actuation -> normalized action
        -> PyBullet sub-steps (wind force applied each sub-step)
        -> collision / bounds / fail-safe checks, logging, HUD

The composition rule between mission intent and the ACMFDS command is
explicit and documented in ``_compose_command``.
"""

import math
import os
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import pybullet as p

from state.flight_state import FlightState, StateValidator
from decision.acmfds import (
    ACMFDS, AvoidanceCandidate, Decision, DecisionType, FlightCommand, FlightContext,
)
from controllers.actuation import ActuationLayer
from simulation.drone import DroneModel
from simulation.obstacles import ObstacleManager
from simulation.pybullet_sim import PyBulletSimulation
from simulation.mission import MissionSequencer, MissionPhase, MissionConfig
from simulation.battery import BatteryModel
from simulation.wind import WindModel
from telemetry.logger import FlightLogger, compute_summary, format_summary, EpisodeSummary


@dataclass
class MissionReport:
    """Result of one mission run."""

    outcome: str                     # landed | collision | timeout | battery_empty | aborted
    mission_completed: bool
    summary: Optional[EpisodeSummary]
    csv_path: str
    steps: int
    duration_s: float
    termination_reason: str
    # Actual clearance flown during the last obstacle-avoidance maneuver,
    # measured from live sensors throughout the window (never synthesized).
    min_maneuver_clearance: Optional[float] = None
    # True when that measured minimum fell below the configured safety
    # margin: a close pass, reported as such even if no collision occurred.
    maneuver_close_pass: bool = False


# Decisions where ACMFDS fully overrides mission motion (veto set).
_VETO_DECISIONS = {
    DecisionType.EMERGENCY,
    DecisionType.SAFE_LANDING,
    DecisionType.MISSION_COMPLETE,
    DecisionType.HOLD,
    DecisionType.AVOID_OBSTACLE,  # detour: creep/back-off forward intent must
                                  # not be overridden by mission cruise speed
    DecisionType.CLIMB,       # altitude work only, no forward motion
    DecisionType.DESCEND,
}

# Contexts / decisions that count as an obstacle-avoidance maneuver window.
_OBSTACLE_CONTEXTS = {
    FlightContext.OBSTACLE_NEAR,
    FlightContext.OBSTACLE_CRITICAL,
    FlightContext.EMERGENCY,
}
_MANEUVER_DECISIONS = {
    DecisionType.AVOID_OBSTACLE,
    DecisionType.EMERGENCY,
    DecisionType.HOLD,
    DecisionType.CLIMB,
}

# Camera pose shared by the follow camera and the billboarded HUD panel.
_HUD_CAM_DIST = 5.5
_HUD_CAM_YAW_DEG = 50.0
_HUD_CAM_PITCH_DEG = -27.0


def _wrap_text(text: str, width: int, max_lines: int) -> List[str]:
    """Word-wrap to ``width`` chars, at most ``max_lines``; overflow elided."""
    words = text.split()
    lines: List[str] = []
    cur = ""
    pending = False
    for w in words:
        cand = (cur + " " + w).strip()
        if len(cand) <= width:
            cur = cand
        elif cur:
            lines.append(cur)
            cur = w
        else:
            lines.append(w[: max(1, width - 3)] + "...")
            cur = ""
        if len(lines) == max_lines:
            pending = True
            break
    if cur and len(lines) < max_lines:
        lines.append(cur)
    if pending and lines:
        lines[-1] = lines[-1][: max(1, width - 3)] + "..."
    return lines


def _cand_brief(c: AvoidanceCandidate) -> str:
    """Compact candidate string for the HUD candidate lines."""
    return f"{c.direction} {c.clearance:.2f}m {'ok' if c.safe else 'x'}"


class ACMFDSMissionRunner:
    """Runs one ACMFDS-driven mission for a given scenario."""

    # HUD palette: cyan/blue telemetry, green normal, amber warnings, red emergencies.
    _HUD_WHITE = [0.92, 0.95, 0.98]
    _HUD_CYAN = [0.25, 0.85, 1.0]
    _HUD_GREEN = [0.30, 1.0, 0.45]
    _HUD_AMBER = [1.0, 0.72, 0.15]
    _HUD_RED = [1.0, 0.28, 0.25]
    _HUD_DIM = [0.60, 0.72, 0.84]

    # HUD layout geometry (meters, world space). Panel is sized so every
    # line fits inside it with single spacing between lines, based on the
    # observed PyBullet debug-text metrics (line height ~0.13-0.25 per unit
    # of textSize): textSize 0.42 -> max line height ~0.105 m < step 0.115 m,
    # and the longest 40-char line stays within the panel width.
    _HUD_TEXT_SIZE = 0.42
    _HUD_LINE_STEP = 0.115
    _HUD_LINE_COUNT = 16

    def __init__(
        self,
        scenario,
        gui: bool = True,
        seed: int = 42,
        csv_path: str = "results/acmfds_flight_log.csv",
        time_step: float = 1.0 / 120.0,
    ) -> None:
        self.scenario = scenario
        self.gui = gui
        self.seed = seed
        self.csv_path = csv_path
        self.dt = float(time_step)

        self.sim = PyBulletSimulation(gui=gui, time_step=self.dt)
        self.acmfds = ACMFDS()
        self.actuation = ActuationLayer()
        self.mission = MissionSequencer(
            goal=scenario.goal,
            config=MissionConfig(cruise_altitude=scenario.cruise_altitude),
        )
        self.battery = BatteryModel(scenario.battery)
        self.wind = WindModel(scenario.wind)

        self.drone: Optional[DroneModel] = None
        self.obstacles: Optional[ObstacleManager] = None
        self.logger = FlightLogger(csv_path, episode=1)

        self._step = 0
        self._trail_points: List[Tuple[float, float, float]] = []
        self._hud_ids: List[int] = []
        self._hud_panel_id: int = -1
        self._hud_border_ids: List[int] = []
        self._hud_line_positions: List[Tuple[float, float, float]] = []
        self._last_hud_view = None          # frozen (state, decision, intent, battery)
        self._last_decision: Optional[Decision] = None
        self._moving_enabled = True
        self._maneuver = {
            "direction": None, "start": None, "min_clearance": math.inf,
            "end_clearance": math.inf, "state": "idle", "result": None,
        }
        # Post-maneuver caution deadline (mission time, s): while the mission
        # clock is below this, ACMFDS keeps cautious speed because the body-
        # frame rays can lose the obstacle mid heading-sweep right after a
        # detour window closes.
        self._maneuver_cooldown_until: float = 0.0

    # ------------------------------------------------------------------
    def setup(self) -> None:
        """Connect physics and spawn the world."""
        self.sim.connect()
        self.drone = DroneModel(
            physics_client_id=self.sim.client_id,
            enable_visual_parts=self.gui,
        )
        start = (self.scenario.start[0], self.scenario.start[1], 0.12)
        self.drone.spawn(position=start, orientation_euler=(0.0, 0.0, 0.0))

        self.obstacles = ObstacleManager(
            physics_client_id=self.sim.client_id,
            randomize=self.scenario.randomize_obstacles,
            num_random_obstacles=self.scenario.num_random_obstacles,
        )
        if self.scenario.obstacles:
            self.obstacles.setup_from_configs(self.scenario.obstacles)
        elif self.scenario.randomize_obstacles:
            self.obstacles.setup_obstacles(
                start_pos=start, goal_pos=(self.scenario.goal[0], self.scenario.goal[1], 2.0)
            )
        # else: scenario defines no obstacles and randomization is off ->
        # the world is intentionally obstacle-free (e.g. scenario A).

        if getattr(self.scenario, "moving_obstacles", None):
            self.obstacles.setup_moving(self.scenario.moving_obstacles)

        # Sync actuation inertia and drag with the actual PyBullet body.
        # getDynamicsInfo index 2 is localInertiaDiagonal (principal moments);
        # index 3 is the inertia position, NOT the inertia.
        info = p.getDynamicsInfo(self.drone.drone_id, -1, physicsClientId=self.sim.client_id)
        self.actuation.config.inertia = (
            float(info[2][0]), float(info[2][1]), float(info[2][2])
        )
        self.actuation.config.linear_drag = self.drone.linear_drag

        if self.gui:
            self.sim.set_target_marker(
                (self.scenario.goal[0], self.scenario.goal[1], self.scenario.cruise_altitude)
            )
            self._draw_goal_marker()
            self._init_hud()

        # Reproducible start state.
        for _ in range(5):
            p.stepSimulation(physicsClientId=self.sim.client_id)

    # ------------------------------------------------------------------
    def run(
        self,
        max_duration: Optional[float] = None,
        realtime: bool = False,
        on_step=None,
        control=None,
    ) -> MissionReport:
        """Execute the mission. Returns a :class:`MissionReport`.

        Args:
            max_duration: override the scenario duration limit (s)
            realtime: pace the loop near real time (GUI/dashboard playback)
            on_step: optional callback receiving the logged row dict each step
                (used by the dashboard bridge; the CSV log is authoritative)
            control: optional duck-typed controller with boolean attributes
                ``pause_requested``, ``stop_requested``, ``moving_enabled``.
                The physics loop pauses/stops on its own thread — it is never
                driven by a viewer, so browser refresh rate cannot affect it.
        """
        cfg = self.mission.config
        duration_limit = max_duration or cfg.max_duration
        outcome = "timeout"
        reason = "max duration reached"
        thrust_n = self.actuation.hover_thrust

        try:
            while True:
                if control is not None:
                    if control.stop_requested:
                        outcome, reason = "aborted", "stopped by operator"
                        break
                    while control.pause_requested and not control.stop_requested:
                        time.sleep(0.05)
                    if control.stop_requested:
                        outcome, reason = "aborted", "stopped by operator"
                        break
                t = self._step * self.dt

                # ---- 0. Advance moving obstacles (kinematic, analytic) -----
                moving_enabled = (
                    control.moving_enabled if control is not None else self._moving_enabled
                )
                self.obstacles.update_moving(t, enabled=moving_enabled)

                # ---- 1. Read physical state ------------------------------
                pos = self.drone.get_position()
                vel = self.drone.get_linear_velocity()
                euler = self.drone.get_orientation_euler()
                omega = self.drone.get_angular_velocity()

                # ---- 2. Sensors -------------------------------------------
                # Rays are drawn only while obstacle-related decisions are
                # active (previous step), keeping the view uncluttered;
                # sensing itself always runs.
                draw_rays = (
                    self.gui
                    and self._last_decision is not None
                    and self._last_decision.context in _OBSTACLE_CONTEXTS
                )
                clearances = self.drone.get_forward_clearances(draw_debug_rays=draw_rays)
                clearance_front = clearances["front"]
                clearance_left = clearances["left"]
                clearance_right = clearances["right"]
                obstacle_distance = min(clearance_front, clearance_left, clearance_right)

                # ---- 3. Battery / wind models -----------------------------
                thrust_frac = max(0.0, min(1.0, thrust_n / self.drone.max_thrust))
                battery_pct = self.battery.update(self.dt, thrust_frac)
                wind_v = self.wind.velocity(t)
                wind_speed = float(np.linalg.norm(wind_v))
                wind_dir = (
                    (float(wind_v[0] / wind_speed), float(wind_v[1] / wind_speed), float(wind_v[2] / wind_speed))
                    if wind_speed > 1e-6 else (1.0, 0.0, 0.0)
                )

                # ---- 4. Mission intent ------------------------------------
                gd = math.hypot(
                    self.scenario.goal[0] - pos[0], self.scenario.goal[1] - pos[1]
                )
                intent = self.mission.update(t, pos, pos[2], vel[2], gd)

                # ---- 5. Unified flight state ------------------------------
                state = StateValidator.sanitize(FlightState(
                    position_x=float(pos[0]), position_y=float(pos[1]),
                    altitude=float(pos[2]),
                    target_altitude=intent.target_altitude,
                    velocity_x=float(vel[0]), velocity_y=float(vel[1]), velocity_z=float(vel[2]),
                    roll=float(euler[0]), pitch=float(euler[1]), yaw=float(euler[2]),
                    roll_rate=float(omega[0]), pitch_rate=float(omega[1]), yaw_rate=float(omega[2]),
                    obstacle_distance=obstacle_distance,
                    clearance_front=clearance_front,
                    clearance_left=clearance_left,
                    clearance_right=clearance_right,
                    clearance_up=clearances["up"],
                    obstacle_top=clearances["front_obstacle_top"],
                    battery_percentage=battery_pct,
                    wind_speed=wind_speed,
                    wind_direction_x=wind_dir[0],
                    wind_direction_y=wind_dir[1],
                    wind_direction_z=wind_dir[2],
                    target_position=intent.target_position,
                    goal_distance=gd,
                    mission_phase=intent.phase.value,
                    maneuver_cooldown_s=max(0.0, self._maneuver_cooldown_until - t),
                ))

                # ---- 6. ACMFDS decision ------------------------------------
                decision = self.acmfds.decide(state)
                self._last_decision = decision
                self._track_maneuver(state, decision, t)
                self._last_hud_view = (state, decision, intent, battery_pct)

                # ---- 7. Command composition --------------------------------
                command = self._compose_command(decision, intent)

                # ---- 8. Actuation ------------------------------------------
                action = self.actuation.compute_action(state, command, self.dt)
                thrust_n = self.actuation._prev_thrust

                # ---- 9. Physics sub-steps ----------------------------------
                for _ in range(1):
                    self.wind.apply(self.sim.client_id, self.drone.drone_id, t)
                    self.drone.apply_action(action)
                    p.stepSimulation(physicsClientId=self.sim.client_id)

                # ---- 10. Safety checks --------------------------------------
                collision = self._check_collision()
                tilt = max(abs(euler[0]), abs(euler[1]))
                out_of_bounds = self._check_bounds(pos)
                terminated = False
                if collision:
                    outcome, reason, terminated = "collision", "contact with obstacle", True
                elif out_of_bounds:
                    outcome, reason, terminated = "aborted", "left permitted flight region", True
                elif tilt > 1.0:
                    outcome, reason, terminated = "aborted", f"excessive tilt {tilt:.2f} rad (crash)", True
                elif battery_pct <= 0.0:
                    outcome, reason, terminated = "battery_empty", "battery depleted", True
                elif t >= duration_limit:
                    terminated = True
                elif self.mission.finished:
                    outcome, reason, terminated = "landed", "mission completed: landed at goal", True

                # ---- 11. Presentation / logging ----------------------------
                # The terminating step (including the collision step) is
                # logged too, so the CSV honestly contains the final state.
                if self.gui and not terminated:
                    self.drone.update_visual_parts()
                    if self._step % 2 == 0:
                        self._update_camera(pos)
                    if self._step % 10 == 0:
                        self._update_trail(pos)
                        self._update_hud(state, decision, intent, battery_pct)
                # Realtime pacing is independent of the GUI (the dashboard
                # runs headless and must still play near real time).
                if realtime and not terminated:
                    time.sleep(self.dt * 0.9)

                row = self.logger.log(
                    timestamp=t, step=self._step, state=state, decision=decision,
                    command=command, thrust_n=thrust_n,
                    collision=collision, mission_completed=self.mission.finished,
                    mission_phase=intent.phase.value,
                    moving_obstacle_positions=self.obstacles.get_moving_positions(),
                )
                if on_step is not None:
                    on_step(row)
                self._step += 1
                if terminated:
                    break

        except KeyboardInterrupt:
            outcome, reason = "aborted", "interrupted by user"
        finally:
            if self.mission.finished and outcome == "timeout":
                outcome, reason = "landed", "mission completed: landed at goal"
            self.logger.flush()
            if self.gui:
                self._update_hud_final(reason)

        summary = compute_summary(self.logger.rows, outcome)
        maneuver = self._maneuver.get("result")
        return MissionReport(
            outcome=outcome,
            mission_completed=self.mission.finished,
            summary=summary,
            csv_path=os.path.abspath(self.csv_path),
            steps=self._step,
            duration_s=self._step * self.dt,
            termination_reason=reason,
            min_maneuver_clearance=(
                float(maneuver["min"]) if maneuver is not None else None
            ),
            maneuver_close_pass=(
                maneuver is not None
                and maneuver["min"] < self.acmfds.config.safety_margin
            ),
        )

    # ------------------------------------------------------------------
    def close(self) -> None:
        if self.drone:
            self.drone.cleanup()
        if self.obstacles:
            self.obstacles.clear_obstacles()
        self.sim.close()

    # ------------------------------------------------------------------
    def _compose_command(self, decision: Decision, intent) -> FlightCommand:
        """Merge mission intent with the ACMFDS recommendation.

        Rule (documented): the mission proposes, ACMFDS disposes. Forward
        motion is the *minimum* of what the mission requests and what the
        ACMFDS speed limit allows; lateral avoidance comes from ACMFDS; the
        veto decision set (HOLD / CLIMB / DESCEND / EMERGENCY / SAFE_LANDING
        / MISSION_COMPLETE) replaces mission motion entirely.
        """
        cmd = decision.command
        if decision.decision in _VETO_DECISIONS:
            final = FlightCommand(
                target_altitude=cmd.target_altitude,
                forward_speed=cmd.forward_speed,
                lateral_speed=cmd.lateral_speed,
                heading=cmd.heading,
                speed_limit=cmd.speed_limit,
            )
        else:
            # Mission proposes the planned (accel/decel-limited) speed;
            # ACMFDS disposes via its speed limit.
            final = FlightCommand(
                target_altitude=cmd.target_altitude,
                forward_speed=min(intent.desired_speed, cmd.speed_limit),
                lateral_speed=cmd.lateral_speed,
                heading=cmd.heading if cmd.heading is not None else intent.heading,
                speed_limit=cmd.speed_limit,
            )
        return final

    def describe_world(self) -> Dict[str, Any]:
        """JSON-safe world geometry for external viewers (dashboard 3D view).

        The exact goal/altitude/bounds/obstacles this runner flies against —
        read from the scenario and the live ObstacleManager, never invented.
        """
        return {
            "goal": [float(self.scenario.goal[0]), float(self.scenario.goal[1]),
                     float(self.scenario.cruise_altitude)],
            "cruise_altitude": float(self.scenario.cruise_altitude),
            "bounds": {"x": [-7.5, 7.5], "y": [-5.0, 5.0], "z": [0.0, 6.0]},
            "obstacles": self.obstacles.describe() if self.obstacles else
                         {"static": [], "moving": []},
        }

    def _check_collision(self) -> bool:
        contacts = p.getContactPoints(
            bodyA=self.drone.drone_id, physicsClientId=self.sim.client_id
        )
        for c in contacts:
            if self.obstacles.is_obstacle(c[2]):
                return True
        return False

    def _check_bounds(self, pos: np.ndarray) -> bool:
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        return not (-7.5 <= x <= 7.5 and -5.0 <= y <= 5.0 and 0.0 <= z <= 6.0)

    # ------------------------------------------------------------------
    # GUI presentation
    # ------------------------------------------------------------------
    def _draw_goal_marker(self) -> None:
        gx, gy = self.scenario.goal
        z = self.scenario.cruise_altitude
        # Vertical post + sphere so the goal reads at a glance.
        post = p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.03, length=z,
            rgbaColor=[0.1, 0.9, 0.2, 0.9], physicsClientId=self.sim.client_id,
        )
        p.createMultiBody(
            baseMass=0, baseVisualShapeIndex=post,
            basePosition=[gx, gy, z / 2.0], physicsClientId=self.sim.client_id,
        )
        ring = p.createVisualShape(
            p.GEOM_SPHERE, radius=0.22,
            rgbaColor=[0.1, 0.95, 0.25, 0.9], physicsClientId=self.sim.client_id,
        )
        p.createMultiBody(
            baseMass=0, baseVisualShapeIndex=ring,
            basePosition=[gx, gy, z], physicsClientId=self.sim.client_id,
        )

    def _track_maneuver(self, state: FlightState, decision: Decision, t: float) -> None:
        """Track obstacle maneuvers: decision *made* vs maneuver *succeeded*.

        ACTIVE covers the window in which ACMFDS keeps issuing an obstacle
        maneuver. When the window closes, the outcome is judged from the
        actual sensor minimum: CLEARED only if the closest obstacle reading
        recovered above the avoidance threshold (and no collision ended the
        run), otherwise ENDED with the clearance left unverified. This
        prevents equating "ACMFDS printed a decision" with "avoidance
        succeeded" — the distinction required by the project spec.
        """
        m = self._maneuver
        avoiding = (
            decision.context in _OBSTACLE_CONTEXTS
            and decision.decision in _MANEUVER_DECISIONS
        )
        if avoiding:
            direction = decision.selected_direction or decision.decision.value
            if m["state"] != "active" or m["direction"] != direction:
                m.update(
                    direction=direction, start=t,
                    min_clearance=state.obstacle_distance,
                    end_clearance=math.inf, state="active", result=None,
                )
            else:
                m["min_clearance"] = min(m["min_clearance"], state.obstacle_distance)
        elif m["state"] == "active":
            m["end_clearance"] = state.obstacle_distance
            m["state"] = (
                "cleared"
                if state.obstacle_distance >= self.acmfds.config.obstacle_avoid
                else "ended"
            )
            m["result"] = {
                "direction": m["direction"],
                "min": m["min_clearance"],
                "end": state.obstacle_distance,
                "outcome": m["state"],
            }
            # Caution window after the maneuver: the drone is still skirting
            # the obstacle corner here, so hold cautious speed for a cooldown
            # instead of letting NORMAL restore cruise instantly.
            self._maneuver_cooldown_until = t + self.acmfds.config.maneuver_cooldown

    def _init_hud(self) -> None:
        """Create the dark translucent HUD panel and its text lines.

        The panel is a *visual-only* body (no collision shape), so the
        sensing rays and contact checks can never hit it even though it
        floats between the camera and the drone. It is billboarded in
        front of the follow camera by :meth:`_update_hud`.
        """
        cid = self.sim.client_id
        panel_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[1.12, 1.35, 0.012],   # half height / half width / half thickness
            rgbaColor=[0.03, 0.05, 0.09, 0.78],
            physicsClientId=cid,
        )
        self._hud_panel_id = p.createMultiBody(
            baseMass=0, baseVisualShapeIndex=panel_shape,
            basePosition=[0.0, 0.0, -10.0], physicsClientId=cid,
        )
        for _ in range(4):
            self._hud_border_ids.append(p.addUserDebugLine(
                [0.0, 0.0, -10.0], [0.0, 0.0, -10.0],
                lineColorRGB=[0.35, 0.65, 0.85], lineWidth=1.6,
                physicsClientId=cid,
            ))
        for _ in range(self._HUD_LINE_COUNT):
            self._hud_ids.append(p.addUserDebugText(
                "", [0.0, 0.0, -10.0],
                textColorRGB=self._HUD_CYAN, textSize=self._HUD_TEXT_SIZE,
                physicsClientId=cid,
            ))

    def _hud_frame(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Camera target and (view, right, up) basis for the follow camera."""
        pos = self.drone.get_position()
        target = np.array([float(pos[0]), float(pos[1]), float(pos[2]) + 0.4])
        yaw = math.radians(_HUD_CAM_YAW_DEG)
        pitch = math.radians(_HUD_CAM_PITCH_DEG)
        view = np.array([
            -math.sin(yaw) * math.cos(pitch),
            math.cos(yaw) * math.cos(pitch),
            math.sin(pitch),
        ])
        right = np.cross(view, np.array([0.0, 0.0, 1.0]))
        right /= max(float(np.linalg.norm(right)), 1e-9)
        up = np.cross(right, view)
        return target, view, right, up

    def _severity_color(self, decision: Decision) -> List[float]:
        """Red for emergencies/critical, amber for cautions, green otherwise."""
        ctx = decision.context
        if ctx in (FlightContext.EMERGENCY, FlightContext.OBSTACLE_CRITICAL):
            return self._HUD_RED
        if ctx in (FlightContext.OBSTACLE_NEAR, FlightContext.LOW_BATTERY, FlightContext.STRONG_WIND):
            return self._HUD_AMBER
        return self._HUD_GREEN

    def _hud_lines(self, state: FlightState, decision: Decision, intent, battery_pct: float) -> List[Tuple[str, List[float]]]:
        """All HUD text lines as (text, color) pairs; indices are stable.

        Obstacle clearances are labeled per direction (FWD/LEFT/RIGHT/UP)
        so each measurement is unambiguous, and every context/decision line
        shows the values that triggered it.
        """
        cfg = self.acmfds.config
        ray_min = min(state.clearance_front, state.clearance_left, state.clearance_right)
        if ray_min <= cfg.obstacle_critical:
            obst_color = self._HUD_RED
        elif ray_min <= cfg.obstacle_near:
            obst_color = self._HUD_AMBER
        else:
            obst_color = self._HUD_CYAN
        bw_color = self._HUD_AMBER if (
            state.battery_percentage <= cfg.battery_low or state.wind_speed >= cfg.wind_strong
        ) else self._HUD_CYAN
        sev = self._severity_color(decision)
        h_spd = math.hypot(state.velocity_x, state.velocity_y)
        finished = self.mission.finished
        top_txt = "--" if state.obstacle_top is None else f"{state.obstacle_top:4.2f}"

        lines: List[Tuple[str, List[float]]] = [
            (f"ACMFDS  |  PHASE {intent.phase.value}  |  {'DONE' if finished else 'RUN'}", sev),
            (f"ALT {state.altitude:5.2f}m   TGT {state.target_altitude:4.2f}m   Vz {state.velocity_z:+5.2f}", self._HUD_CYAN),
            (f"SPD {h_spd:4.2f}m/s   GOAL {state.goal_distance:5.2f}m   ({state.position_x:+4.1f},{state.position_y:+4.1f})", self._HUD_CYAN),
            (f"CLEAR FWD {state.clearance_front:4.2f}  LEFT {state.clearance_left:4.2f}  RIGHT {state.clearance_right:4.2f} m", obst_color),
            (f"CLEAR UP {state.clearance_up:4.2f} m   OBST TOP {top_txt} m", self._HUD_CYAN),
            (f"BATT {state.battery_percentage:5.1f}%   WIND {state.wind_speed:4.2f}m/s ({state.wind_direction_x:+.1f},{state.wind_direction_y:+.1f})", bw_color),
            (f"CTX {decision.context.value} -> {decision.decision.value}", sev),
            (f"SELECTED {decision.selected_direction or '--'}   SPEED LIM {decision.command.speed_limit:.1f}m/s",
             self._HUD_GREEN if decision.selected_direction else self._HUD_DIM),
        ]
        why_lines = [
            ((("WHY " if j == 0 else "    ") + part), self._HUD_WHITE)
            for j, part in enumerate(_wrap_text(decision.reason, 38, 3))
        ]
        lines.extend(why_lines)
        while len(lines) < 11:
            lines.append(("", self._HUD_DIM))
        cand_a = " | ".join(_cand_brief(c) for c in decision.candidates[:2]) or "no candidates evaluated"
        cand_b = " | ".join(_cand_brief(c) for c in decision.candidates[2:4])
        lines.append((f"CAND {cand_a}", self._HUD_DIM))
        lines.append((cand_b, self._HUD_DIM))

        m = self._maneuver
        if m["state"] == "active":
            lines.append((f"MANEUVER {m['direction']} ACTIVE  min {m['min_clearance']:.2f}m", self._HUD_AMBER))
        elif m["result"] is not None:
            r = m["result"]
            close = r["min"] < cfg.safety_margin
            if r["outcome"] == "cleared":
                lines.append((f"MANEUVER {r['direction']} CLEARED  min {r['min']:.2f}m",
                              self._HUD_GREEN if not close else self._HUD_AMBER))
            else:
                lines.append((f"MANEUVER {r['direction']} ENDED  min {r['min']:.2f}m", self._HUD_AMBER))
        else:
            lines.append(("MANEUVER --", self._HUD_DIM))

        lines.append((f"STATUS {'LANDED' if finished else 'IN FLIGHT'}",
                      self._HUD_GREEN if finished else self._HUD_CYAN))

        if m["result"] is not None and m["state"] != "active":
            r = m["result"]
            close = r["min"] < cfg.safety_margin
            if r["outcome"] == "cleared" and not close:
                lines.append((f"RESULT {r['direction']}: min {r['min']:.2f}m, end {r['end']:.2f}m",
                              self._HUD_GREEN))
            elif r["outcome"] == "cleared":
                lines.append((f"RESULT {r['direction']}: min {r['min']:.2f}m < margin {cfg.safety_margin:.2f}m",
                              self._HUD_AMBER))
            else:
                lines.append((f"RESULT {r['direction']}: closed at {r['end']:.2f}m unverified",
                              self._HUD_AMBER))
        else:
            lines.append(("", self._HUD_DIM))
        return lines

    def _update_hud(self, state: FlightState, decision: Decision, intent, battery_pct: float) -> None:
        if self._hud_panel_id < 0 or not self._hud_ids:
            return
        cid = self.sim.client_id
        target, view, right, up = self._hud_frame()
        # Billboard anchor: 2.6 m in front of the drone along the view axis
        # (i.e. 2.9 m from the camera, between camera and drone), shifted
        # left and slightly up so it sits in the upper-left of the frame.
        center = target - view * 2.6 - right * 1.25 + up * 0.30
        quat = p.getQuaternionFromEuler(
            [0.0,
             math.radians(90.0 + _HUD_CAM_PITCH_DEG),
             math.radians(_HUD_CAM_YAW_DEG - 90.0)],
            physicsClientId=cid,
        )
        p.resetBasePositionAndOrientation(self._hud_panel_id, list(center), quat, physicsClientId=cid)

        off = -view * 0.05  # nudge toward the camera to avoid z-fighting
        c_tl = center + off + up * 1.05 + right * 1.28
        c_tr = center + off + up * 1.05 - right * 1.28
        c_br = center + off - up * 1.05 - right * 1.28
        c_bl = center + off - up * 1.05 + right * 1.28
        for uid, (a, b) in zip(
            self._hud_border_ids, [(c_tl, c_tr), (c_tr, c_br), (c_br, c_bl), (c_bl, c_tl)]
        ):
            p.addUserDebugLine(
                list(a), list(b), lineColorRGB=[0.35, 0.65, 0.85], lineWidth=1.6,
                replaceItemUniqueId=uid, physicsClientId=cid,
            )

        lines = self._hud_lines(state, decision, intent, battery_pct)
        anchor = center + off + up * 0.98 - right * 1.22
        self._hud_line_positions = []
        for i, uid in enumerate(self._hud_ids):
            text, color = lines[i] if i < len(lines) else ("", self._HUD_DIM)
            pos = anchor - up * (self._HUD_LINE_STEP * i)
            self._hud_line_positions.append((float(pos[0]), float(pos[1]), float(pos[2])))
            p.addUserDebugText(
                text, list(pos), textColorRGB=list(color), textSize=self._HUD_TEXT_SIZE,
                replaceItemUniqueId=uid, physicsClientId=cid,
            )

    def _update_hud_final(self, reason: str) -> None:
        """Redraw the last real frame, then stamp the termination banner."""
        if self._hud_panel_id < 0 or not self._hud_ids:
            return
        if self._last_hud_view is not None:
            state, decision, intent, battery_pct = self._last_hud_view
            self._update_hud(state, decision, intent, battery_pct)
        banner = _wrap_text(f"TERMINATED  {reason}", 40, 2)
        last = len(self._hud_ids) - 1
        for j, idx in enumerate((last - 1, last)):
            if idx < 0:
                break
            text = banner[j] if j < len(banner) else ""
            if idx < len(self._hud_line_positions):
                pos = list(self._hud_line_positions[idx])
            else:
                # HUD frame never drawn (early abort): place the banner anyway.
                target, view, right, up = self._hud_frame()
                base = (target - view * 2.6 - right * 1.25 + up * 0.30
                        + (-view * 0.05) + up * 0.98 - right * 1.22)
                pos = list(base - up * (self._HUD_LINE_STEP * idx))
            p.addUserDebugText(
                text, pos, textColorRGB=list(self._HUD_RED), textSize=self._HUD_TEXT_SIZE,
                replaceItemUniqueId=self._hud_ids[idx], physicsClientId=self.sim.client_id,
            )

    def _update_trail(self, pos: Sequence[float]) -> None:
        pt = (float(pos[0]), float(pos[1]), float(pos[2]))
        last = self._trail_points[-1] if self._trail_points else None
        if last is not None and math.dist(pt, last) < 0.12:
            return
        if self._trail_points:
            p.addUserDebugLine(
                list(self._trail_points[-1]), list(pt),
                lineColorRGB=[0.2, 0.8, 1.0], lineWidth=2.0,
                lifeTime=0, physicsClientId=self.sim.client_id,
            )
        self._trail_points.append(pt)

    def _update_camera(self, pos: Sequence[float]) -> None:
        self.sim.set_camera(
            distance=_HUD_CAM_DIST, yaw=_HUD_CAM_YAW_DEG, pitch=_HUD_CAM_PITCH_DEG,
            target=(float(pos[0]), float(pos[1]), float(pos[2]) + 0.4),
        )


def run_acmfds_mission(scenario, gui: bool = True, seed: int = 42,
                       csv_path: str = "results/acmfds_flight_log.csv",
                       realtime: bool = False,
                       max_duration: Optional[float] = None) -> MissionReport:
    """Convenience wrapper: setup -> run -> close."""
    runner = ACMFDSMissionRunner(scenario, gui=gui, seed=seed, csv_path=csv_path)
    try:
        runner.setup()
        report = runner.run(max_duration=max_duration, realtime=realtime)
    finally:
        runner.close()
    return report
