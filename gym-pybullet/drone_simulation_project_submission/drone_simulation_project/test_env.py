"""Test suite for the ACMFDS drone simulation project.

Covers the validation checklist from the implementation spec:
1.  Gymnasium API compliance (check_env)
2.  Reset/step contracts
3.  Physical drone dynamics (mass, hover)
4.  Ray sensors vs a KNOWN obstacle geometry
5.  Data-driven feedforward controller (PID-free contract, thrust limits,
    sequential closed-loop accuracy)
6.  ACMFDS rule evaluation across all contexts (headless, no physics)
7.  FlightState conversion/validation and per-field source tags
8.  Battery bounds and consumption model
9.  Wind vector reproducibility and force conversion
10. Mission phase chain INITIALIZE->...->DONE
11. Decision -> command composition -> actuation path
12. Visual parts follow the physics body (orientation-transformed offsets)
13. Headless mission run writes per-step CSV records
14. Default application path is free of RL imports
"""

import os
import sys
import csv
import math
import unittest

import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import pybullet as p
from gymnasium.utils.env_checker import check_env

from envs.drone_env import DroneEnv
from state.flight_state import FIELD_SOURCES, FlightState, SourceTag, StateValidator
from decision.acmfds import (
    ACMFDS, ACMFDSConfig, CONTEXT_WEIGHTS, Decision, DecisionType, FlightCommand,
    FlightContext, Objective,
)
from controllers.actuation import ActuationConfig, ActuationLayer
from controllers.data_driven_controller import DataDrivenAltitudeController, DataDrivenController
from simulation.battery import BatteryConfig, BatteryModel
from simulation.mission import MissionConfig, MissionPhase, MissionSequencer
from simulation.wind import AIR_DENSITY, DRAG_COEFFICIENT, EXPOSED_AREA, WindConfig, WindModel
from simulation.drone import DroneModel
from simulation.obstacles import ObstacleConfig, ObstacleManager
from simulation.pybullet_sim import PyBulletSimulation
from telemetry.logger import CSV_COLUMNS, compute_summary


def make_state(**overrides) -> FlightState:
    """A sane default flight state; keyword overrides for scenario tests."""
    base = dict(
        position_x=0.0, position_y=0.0, altitude=2.0,
        target_altitude=2.0,
        velocity_x=0.0, velocity_y=0.0, velocity_z=0.0,
        roll=0.0, pitch=0.0, yaw=0.0,
        roll_rate=0.0, pitch_rate=0.0, yaw_rate=0.0,
        obstacle_distance=5.0,
        clearance_front=5.0, clearance_left=5.0, clearance_right=5.0,
        battery_percentage=100.0, wind_speed=0.0,
        wind_direction_x=1.0, wind_direction_y=0.0, wind_direction_z=0.0,
        target_position=(4.5, 0.0, 2.0), goal_distance=4.5,
    )
    base.update(overrides)
    return FlightState(**base)


class TestGymnasiumEnv(unittest.TestCase):
    """Gymnasium API compliance and reset/step contracts."""

    def test_01_check_env(self) -> None:
        env = DroneEnv(render_mode=None, max_episode_steps=50)
        try:
            check_env(env.unwrapped)
        finally:
            env.close()

    def test_02_reset_and_step_contract(self) -> None:
        env = DroneEnv(render_mode=None, max_episode_steps=30)
        try:
            obs, info = env.reset(seed=42)
            self.assertEqual(obs.shape, (27,))
            self.assertEqual(obs.dtype, np.float32)
            self.assertIn("distance_to_target", info)
            self.assertIn("acmfds_decision", info)

            obs2, _ = env.reset(seed=42)
            np.testing.assert_allclose(obs, obs2, atol=1e-3)

            action = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
            next_obs, reward, terminated, truncated, step_info = env.step(action)
            self.assertEqual(next_obs.shape, (27,))
            self.assertIsInstance(reward, float)
            self.assertIsInstance(terminated, bool)
            self.assertIsInstance(truncated, bool)
            self.assertIn("acmfds_context", step_info)
            self.assertIn("acmfds_reason", step_info)
        finally:
            env.close()

    def test_03_drone_physics_and_hover(self) -> None:
        """Action [0,0,0,0] must map to hover thrust and hold altitude."""
        env = DroneEnv(render_mode=None, max_episode_steps=100)
        try:
            env.reset(seed=123)
            initial_z = env.drone.get_position()[2]
            obs = None
            for _ in range(30):
                obs, _, _, _, _ = env.step(np.zeros(4, dtype=np.float32))
            self.assertAlmostEqual(initial_z, float(obs[2]), delta=0.35)
        finally:
            env.close()


class TestFlightState(unittest.TestCase):
    """Unified state dataclass: sources, validation, derived quantities."""

    def test_field_sources_cover_all_fields(self) -> None:
        state = make_state()
        for f in state.__dataclass_fields__:
            if f == "issues":
                continue  # validation-report metadata, not a state input
            self.assertIn(f, FIELD_SOURCES, f"field {f} missing source tag")
        # Sensor fields are tagged SENSOR, modeled fields MODELED.
        self.assertEqual(FIELD_SOURCES["clearance_front"], SourceTag.SENSOR)
        self.assertEqual(FIELD_SOURCES["battery_percentage"], SourceTag.MODELED)
        self.assertEqual(FIELD_SOURCES["altitude"], SourceTag.SIMULATION)

    def test_sanitize_clamps_and_reports(self) -> None:
        state = make_state(altitude=-5.0, battery_percentage=150.0,
                           wind_speed=2.0,
                           wind_direction_x=3.0, wind_direction_y=4.0)
        clean = StateValidator.sanitize(state)
        self.assertGreaterEqual(clean.altitude, 0.0)
        self.assertLessEqual(clean.battery_percentage, 100.0)
        # Wind direction normalized to unit length.
        norm = math.sqrt(clean.wind_direction_x ** 2 + clean.wind_direction_y ** 2
                         + clean.wind_direction_z ** 2)
        self.assertAlmostEqual(norm, 1.0, places=5)
        self.assertTrue(clean.issues, "sanitizer must record every correction")

    def test_derived_properties(self) -> None:
        state = make_state(velocity_x=3.0, velocity_y=4.0,
                           wind_direction_x=0.0, wind_direction_y=1.0, wind_speed=2.0)
        self.assertAlmostEqual(state.horizontal_speed, 5.0)
        np.testing.assert_allclose(state.wind_vector, [0.0, 2.0, 0.0], atol=1e-6)
        self.assertAlmostEqual(state.altitude_error, 0.0)


class TestACMFDSRules(unittest.TestCase):
    """Rule evaluation across every context; explainable output; weights."""

    def setUp(self) -> None:
        self.acmfds = ACMFDS()

    def test_context_weights_sum_to_one(self) -> None:
        for ctx, weights in CONTEXT_WEIGHTS.items():
            total = sum(weights.values())
            self.assertAlmostEqual(total, 1.0, places=6, msg=f"context {ctx}")
            self.assertEqual(set(weights), set(Objective))

    def test_normal_move_forward(self) -> None:
        d = self.acmfds.decide(make_state())
        self.assertEqual(d.context, FlightContext.NORMAL)
        self.assertEqual(d.decision, DecisionType.MOVE_FORWARD)
        self.assertTrue(d.reason)
        self.assertTrue(d.constraints)
        self.assertTrue(d.command.speed_limit > 0)

    def test_obstacle_avoid_picks_clearer_side(self) -> None:
        d = self.acmfds.decide(make_state(
            obstacle_distance=1.2, clearance_front=1.2,
            clearance_left=3.0, clearance_right=0.6,
        ))
        self.assertEqual(d.decision, DecisionType.AVOID_OBSTACLE)
        self.assertGreater(d.command.lateral_speed, 0.0)  # detour left = +y body

    def test_obstacle_critical_retreats_toward_clear_side(self) -> None:
        d = self.acmfds.decide(make_state(
            obstacle_distance=0.5, clearance_front=0.5,
            clearance_left=4.0, clearance_right=0.4,
        ))
        self.assertEqual(d.context, FlightContext.EMERGENCY)
        self.assertEqual(d.decision, DecisionType.EMERGENCY)
        self.assertGreater(d.command.lateral_speed, 0.0)   # retreat left
        self.assertLess(d.command.forward_speed, 0.0)      # slow reverse

    def test_obstacle_critical_wedged_climbs_never_descends(self) -> None:
        d = self.acmfds.decide(make_state(
            obstacle_distance=0.5, clearance_front=0.5,
            clearance_left=0.3, clearance_right=0.3, altitude=2.0,
        ))
        self.assertEqual(d.decision, DecisionType.EMERGENCY)
        self.assertGreater(d.command.target_altitude, 2.0)  # climb over
        self.assertEqual(d.command.forward_speed, 0.0)

    def test_low_battery_energy_conservation(self) -> None:
        d = self.acmfds.decide(make_state(battery_percentage=25.0))
        self.assertEqual(d.context, FlightContext.LOW_BATTERY)
        self.assertEqual(d.decision, DecisionType.ENERGY_CONSERVATION)

    def test_critical_battery_safe_landing(self) -> None:
        d = self.acmfds.decide(make_state(battery_percentage=10.0))
        self.assertEqual(d.decision, DecisionType.SAFE_LANDING)
        self.assertEqual(d.command.target_altitude, 0.0)
        self.assertEqual(d.command.speed_limit, 0.0)

    def test_strong_wind_response(self) -> None:
        d = self.acmfds.decide(make_state(wind_speed=5.0))
        self.assertEqual(d.context, FlightContext.STRONG_WIND)
        self.assertEqual(d.decision, DecisionType.WIND_RESPONSE)
        self.assertLessEqual(d.command.speed_limit, ACMFDSConfig().cautious_speed)

    def test_mission_complete(self) -> None:
        # Touchdown-confirmed (phase DONE) completes the mission; goal
        # distance alone must not.
        d = self.acmfds.decide(make_state(goal_distance=0.2, mission_phase="DONE"))
        self.assertEqual(d.context, FlightContext.MISSION_COMPLETE)
        self.assertEqual(d.decision, DecisionType.MISSION_COMPLETE)

    def test_goal_reached_without_touchdown_is_not_complete(self) -> None:
        d = self.acmfds.decide(make_state(goal_distance=0.2, mission_phase="MOVE_FORWARD"))
        self.assertNotEqual(d.context, FlightContext.MISSION_COMPLETE)

    def test_landing_phase_maps_to_landing_context(self) -> None:
        d = self.acmfds.decide(make_state(
            goal_distance=0.2, mission_phase="LAND", target_altitude=0.0,
            altitude=0.8, velocity_z=-0.4,
        ))
        self.assertEqual(d.context, FlightContext.LANDING)
        self.assertEqual(d.decision, DecisionType.DESCEND)
        self.assertEqual(d.command.forward_speed, 0.0)

    def test_ground_skim_climbs(self) -> None:
        d = self.acmfds.decide(make_state(altitude=0.15, target_altitude=2.0,
                                          velocity_z=-0.2))
        self.assertEqual(d.decision, DecisionType.CLIMB)

    def test_output_is_explainable(self) -> None:
        d = self.acmfds.decide(make_state(battery_percentage=20.0, wind_speed=5.0))
        for key in ("SAFETY", "ENERGY_EFFICIENCY", "OBSTACLE_CLEARANCE", "MISSION_PROGRESS"):
            self.assertIn(key, d.weights)
        self.assertIsInstance(d, Decision)


class TestBatteryModel(unittest.TestCase):
    def test_bounds_and_monotonicity(self) -> None:
        bat = BatteryModel(BatteryConfig(capacity_wh=4.0))
        prev = bat.percentage
        for _ in range(500):
            pct = bat.update(1.0 / 120.0, 0.7)
            self.assertLessEqual(pct, prev + 1e-9)
            self.assertGreaterEqual(pct, 0.0)
            self.assertLessEqual(pct, 100.0)
            prev = pct
        self.assertLess(prev, 100.0, "hover workload must consume charge")

    def test_configurable_thresholds(self) -> None:
        cfg = BatteryConfig(initial_charge_percent=32.0)
        self.assertEqual(cfg.initial_charge_percent, 32.0)
        bat = BatteryModel(cfg)
        self.assertAlmostEqual(bat.percentage, 32.0, places=3)


class TestWindModel(unittest.TestCase):
    def test_vector_reproducible(self) -> None:
        cfg = WindConfig(enabled=True, base_speed=3.0, direction=(0.0, 1.0, 0.0),
                         gust_amplitude=1.0, gust_period=6.0, seed=7)
        a, b = WindModel(cfg), WindModel(cfg)
        for t in (0.0, 1.3, 5.7):
            np.testing.assert_allclose(a.velocity(t), b.velocity(t), atol=1e-9)

    def test_force_conversion_documented(self) -> None:
        wind = WindModel(WindConfig(enabled=True, base_speed=5.0, direction=(1.0, 0.0, 0.0)))
        f = wind.relative_force(np.array([5.0, 0.0, 0.0]), [0.0, 0.0, 0.0])
        expected = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * EXPOSED_AREA * 25.0
        self.assertAlmostEqual(float(np.linalg.norm(f)), expected, places=6)
        self.assertGreater(f[0], 0.0)  # pushes downwind

    def test_relative_airspeed(self) -> None:
        # Drone moving downwind at 2 m/s sees only 3 m/s of relative wind.
        wind = WindModel(WindConfig(enabled=True))
        f = wind.relative_force(np.array([5.0, 0.0, 0.0]), [2.0, 0.0, 0.0])
        expected = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * EXPOSED_AREA * 9.0
        self.assertAlmostEqual(float(np.linalg.norm(f)), expected, places=6)


class TestMissionSequencer(unittest.TestCase):
    def test_full_phase_chain(self) -> None:
        seq = MissionSequencer(goal=(4.5, 0.0), config=MissionConfig(hover_time=0.2))
        t, dt = 0.0, 1.0 / 120.0
        pos = [-4.0, 0.0, 0.12]
        visited = [MissionPhase.INITIALIZE]
        for _ in range(20000):
            intent = seq.update(t, pos, pos[2], 0.0, goal_distance=max(0.0, 4.5 - pos[0]))
            if not visited or visited[-1] != seq.phase:
                visited.append(seq.phase)
            t += dt
            if seq.phase == MissionPhase.TAKEOFF:
                pos[2] = min(2.0, pos[2] + 1.2 * dt)
            elif seq.phase == MissionPhase.HOVER:
                pass
            elif seq.phase in (MissionPhase.MOVE_FORWARD, MissionPhase.SLOW_DOWN):
                pos[0] = min(4.5, pos[0] + 1.0 * dt)
                if seq.phase == MissionPhase.SLOW_DOWN and 4.5 - pos[0] <= 0.3:
                    pos[0] = 4.4  # near stop
            elif seq.phase == MissionPhase.LAND:
                pos[2] = max(0.0, pos[2] - 0.6 * dt)
            elif seq.phase == MissionPhase.DONE:
                break
        self.assertEqual(visited[0], MissionPhase.INITIALIZE)
        self.assertIn(MissionPhase.TAKEOFF, visited)
        self.assertIn(MissionPhase.HOVER, visited)
        self.assertIn(MissionPhase.MOVE_FORWARD, visited)
        self.assertIn(MissionPhase.LAND, visited)
        self.assertEqual(visited[-1], MissionPhase.DONE)


class TestActuationPath(unittest.TestCase):
    """Decision -> command composition -> actuation -> bounded action."""

    def test_command_to_action_bounds(self) -> None:
        act = ActuationLayer(ActuationConfig(inertia=(0.023, 0.023, 0.042)))
        acmfds = ACMFDS()
        for overrides in (
            {},
            {"wind_speed": 5.0, "wind_direction_x": 0.0, "wind_direction_y": 1.0},
            {"obstacle_distance": 1.0, "clearance_front": 1.0,
             "clearance_left": 3.0, "clearance_right": 0.5},
            {"altitude": 0.3, "target_altitude": 0.0, "velocity_z": -0.4},  # landing
        ):
            state = StateValidator.sanitize(make_state(**overrides))
            decision = acmfds.decide(state)
            command = decision.command
            action = act.compute_action(state, command, 1.0 / 120.0)
            self.assertEqual(action.shape, (4,))
            self.assertTrue(np.all(np.isfinite(action)))
            self.assertTrue(np.all(action >= -1.0) and np.all(action <= 1.0))

    def test_landing_gate_levels_drone(self) -> None:
        """Below the landing cutoff with a near-zero target: horizontal demand
        must be zeroed (the drone levels for vertical touchdown) even when
        the command asks for forward speed."""
        act = ActuationLayer(ActuationConfig(inertia=(0.023, 0.023, 0.042)))
        state = StateValidator.sanitize(make_state(
            altitude=0.3, target_altitude=0.0, velocity_z=-0.3,
        ))
        cmd = FlightCommand(target_altitude=0.0, forward_speed=0.6, speed_limit=0.6)
        gated = act._horizontal_control(state, cmd)
        # Same state/command above the cutoff must demand tilt (sanity: the
        # gate, not a dead controller, produces the zero).
        state_high = StateValidator.sanitize(make_state(
            altitude=1.5, target_altitude=0.0, velocity_z=-0.3,
        ))
        ungated = act._horizontal_control(state_high, cmd)
        self.assertAlmostEqual(gated[0], 0.0, places=5)
        self.assertAlmostEqual(gated[1], 0.0, places=5)
        self.assertGreater(abs(ungated[0]) + abs(ungated[1]), 1e-3)


class TestDataDrivenController(unittest.TestCase):
    """PID-free learned feedforward: contract, limits, sequential accuracy."""

    def test_model_file_and_features(self) -> None:
        path = os.path.join(PROJECT_ROOT, "models", "data_driven_controller.pkl")
        self.assertTrue(os.path.exists(path))
        ctrl = DataDrivenAltitudeController()
        self.assertTrue(ctrl.load(path))
        self.assertEqual(len(ctrl.FEATURE_NAMES), 7)
        self.assertIn("target_accel", ctrl.FEATURE_NAMES)

    def test_feedforward_contract_and_bounds(self) -> None:
        """The learned model is a *feedforward inverse-dynamics* predictor:
        with zero reference acceleration and zero velocity the correct output
        is hover thrust regardless of position error; nonzero reference
        acceleration shifts the prediction by ~m*a (no PID error feedback)."""
        ctrl = DataDrivenController(hover_thrust=9.81, min_thrust=0.0, max_thrust=20.0)
        dt = 1.0 / 120.0
        ctrl.reset()
        hover = ctrl.compute(target_z=2.0, current_z=2.0, dt=dt, current_vz=0.0)
        ctrl.reset()
        climb_ff = ctrl.compute(target_z=2.0, current_z=2.0, dt=dt,
                                current_vz=0.0, target_accel=1.5)
        ctrl.reset()
        descend_ff = ctrl.compute(target_z=2.0, current_z=2.0, dt=dt,
                                  current_vz=0.0, target_accel=-1.5)
        self.assertAlmostEqual(hover, 9.81, delta=1.0)
        self.assertGreater(climb_ff, hover + 1.0)
        self.assertLess(descend_ff, hover - 1.0)
        for t in (hover, climb_ff, descend_ff):
            self.assertGreaterEqual(t, ctrl.min_thrust)
            self.assertLessEqual(t, ctrl.max_thrust)

    def test_sequential_feedforward_accuracy(self) -> None:
        """Closed-loop sequential use over a minimum-jerk maneuver window."""
        from train_data_driven import minimum_jerk

        mass, gravity, drag, dt = 1.0, 9.81, 0.25, 1.0 / 120.0
        ctrl = DataDrivenAltitudeController()
        self.assertTrue(ctrl.load(os.path.join(PROJECT_ROOT, "models",
                                               "data_driven_controller.pkl")))
        act = ActuationLayer(ActuationConfig(inertia=(0.023, 0.023, 0.042)))
        T, z, vz, t = 2.0, 1.0, 0.0, 0.0
        errs = []
        while t <= T + 0.8:
            z_ref, vz_ref, az_ref = minimum_jerk(t, T, 1.0, 2.0)
            pred = ctrl.compute(target_z=z_ref, current_z=z, dt=dt,
                                current_vz=vz, target_accel=az_ref)
            star = mass * (gravity + az_ref) + drag * vz_ref
            if t > 0.2:
                errs.append(abs(pred - star))
            state = StateValidator.sanitize(make_state(
                altitude=z, target_altitude=z_ref, velocity_z=vz))
            act.compute_action(state, FlightCommand(target_altitude=z_ref), dt)
            thrust = act._prev_thrust
            az = (thrust - mass * gravity - drag * vz) / mass
            vz += az * dt
            z += vz * dt
            t += dt
        self.assertLess(np.mean(errs), 0.15, f"sequential MAE {np.mean(errs):.4f} N")
        self.assertLess(np.max(errs), 0.5, f"sequential max {np.max(errs):.4f} N")

    def test_compute_control_dict_api(self) -> None:
        ctrl = DataDrivenController(hover_thrust=9.81, min_thrust=0.0, max_thrust=20.0)
        out = ctrl.compute_control(state={"current_z": 1.0, "target_z": 2.5,
                                           "vz": 0.0, "target_accel": 0.0})
        self.assertGreater(out, 9.81)
        self.assertLessEqual(out, 20.0)


class TestRaySensorsVsKnownObstacle(unittest.TestCase):
    """Real rayTestBatch sensing against a known pillar geometry."""

    def test_forward_clearance_matches_geometry(self) -> None:
        sim = PyBulletSimulation(gui=False, time_step=1.0 / 120.0)
        sim.connect()
        try:
            drone = DroneModel(physics_client_id=sim.client_id, enable_visual_parts=False)
            drone.spawn(position=(0.0, 0.0, 1.0), orientation_euler=(0.0, 0.0, 0.0))
            obstacles = ObstacleManager(physics_client_id=sim.client_id)
            obstacles.setup_from_configs([
                ObstacleConfig(position=(2.0, 0.0, 1.5), half_extents=(0.4, 0.4, 1.5)),
            ])
            for _ in range(5):
                p.stepSimulation(physicsClientId=sim.client_id)
            c = drone.get_forward_clearances(draw_debug_rays=False)
            # Rays originate at the nose (~0.25 m ahead of center); pillar
            # face at x = 1.6 -> clearance ~= 1.35 m.
            self.assertGreater(c["front"], 1.20)
            self.assertLess(c["front"], 1.50)
            self.assertGreaterEqual(c["left"], 4.9)   # nothing to the sides
            self.assertGreaterEqual(c["right"], 4.9)
            drone.cleanup()
            obstacles.clear_obstacles()
        finally:
            sim.close()


class TestVisualPartsFollowBody(unittest.TestCase):
    """Accessories are separate massless bodies synced via orientation math."""

    def test_offsets_rotate_with_yaw(self) -> None:
        sim = PyBulletSimulation(gui=False, time_step=1.0 / 120.0)
        sim.connect()
        try:
            drone = DroneModel(physics_client_id=sim.client_id, enable_visual_parts=True)
            drone.spawn(position=(1.0, 2.0, 0.5), orientation_euler=(0.0, 0.0, math.pi / 2.0))
            p.stepSimulation(physicsClientId=sim.client_id)
            drone.update_visual_parts()
            self.assertGreaterEqual(len(drone.visual_parts), 4)
            # With yaw = 90 deg, a body-+X accessory must sit at world +Y.
            found = False
            for part in drone.visual_parts:
                ppos, _ = p.getBasePositionAndOrientation(
                    part["id"], physicsClientId=sim.client_id)
                dx, dy = ppos[0] - 1.0, ppos[1] - 2.0
                if abs(dx) < 0.05 and 0.2 < dy < 0.9:  # nose along +y
                    found = True
            self.assertTrue(found, "no accessory at the yaw-rotated nose offset")
            drone.cleanup()
        finally:
            sim.close()


class TestHeadlessMissionLogging(unittest.TestCase):
    """Truncated headless mission produces per-step CSV records."""

    def test_csv_records_written(self) -> None:
        from scenarios import SCENARIOS
        from simulation.acmfds_mission import run_acmfds_mission

        out = os.path.join(PROJECT_ROOT, "results", "_test_mission_log.csv")
        report = run_acmfds_mission(
            SCENARIOS["normal"], gui=False, seed=42, csv_path=out, max_duration=3.0,
        )
        self.assertTrue(os.path.exists(out))
        with open(out, newline="") as fh:
            rows = list(csv.DictReader(fh))
        self.assertGreater(len(rows), 200, "expected ~360 logged steps")
        for col in ("timestamp", "step", "altitude", "target_altitude",
                    "acmfds_context", "acmfds_decision", "decision_reason",
                    "thrust_n", "battery_percentage", "wind_speed"):
            self.assertIn(col, rows[0], f"missing CSV column {col}")
        summary = compute_summary(
            [{k: (float(v) if k not in ("acmfds_context", "acmfds_decision", "decision_reason",
                                        "mission_phase", "selected_direction", "movement_action",
                                        "candidates", "moving_obstacles")
                  else v) for k, v in r.items()} for r in rows[:100]],
            report.outcome,
        )
        self.assertGreater(summary.steps, 0)


class TestDefaultPathFreeOfRL(unittest.TestCase):
    def test_main_imports_no_stable_baselines(self) -> None:
        import main  # noqa: F401
        self.assertNotIn("stable_baselines3", sys.modules)
        src = open(os.path.join(PROJECT_ROOT, "main.py"), encoding="utf-8").read()
        self.assertNotIn("stable_baselines3", src)


class TestWaypointNavigation(unittest.TestCase):
    def test_interpolation_and_stepping(self) -> None:
        from trajectory.interpolation import (
            DEFAULT_WAYPOINTS, WaypointNavigator, interpolate_waypoints,
        )
        nav = WaypointNavigator(DEFAULT_WAYPOINTS, acceptance_radius=0.4)
        dense = interpolate_waypoints(DEFAULT_WAYPOINTS, num_samples_per_segment=10)
        self.assertGreater(len(dense), len(DEFAULT_WAYPOINTS))
        _, reached, _ = nav.update(DEFAULT_WAYPOINTS[0])
        self.assertTrue(reached)
        self.assertEqual(nav.current_index, 1)


class TestObstacleAvoidanceMLModel(unittest.TestCase):
    def test_pretrained_model_loads(self) -> None:
        import joblib
        model_path = os.path.join(PROJECT_ROOT, "models", "obstacle_avoidance_model.pkl")
        if not os.path.exists(model_path):
            self.skipTest("optional pretrained model not present")
        model = joblib.load(model_path)
        pred = model.predict(np.array([[2.0, 0.0]], dtype=np.float32))
        self.assertIn(int(pred[0]), (0, 1, 2))


class TestSpeedPlanner(unittest.TestCase):
    """Trapezoidal speed plan: bounded acceleration, braking curve, caps."""

    def _advance_to_move(self, seq, cfg):
        t, dt = 0.0, 1.0 / 120.0
        pos = [-4.0, 0.0, 0.12]
        for _ in range(6000):
            seq.update(t, pos, pos[2], 0.0, goal_distance=max(0.0, 4.5 - pos[0]))
            if seq.phase == MissionPhase.MOVE_FORWARD:
                return t, pos
            if seq.phase == MissionPhase.TAKEOFF:
                pos[2] = min(cfg.cruise_altitude, pos[2] + 1.2 * dt)
            t += dt
        self.fail("sequence never reached MOVE_FORWARD")

    def test_accel_limit_and_cruise_cap(self) -> None:
        cfg = MissionConfig(hover_time=0.1, cruise_speed=1.2, accel=0.7, max_speed=1.5)
        seq = MissionSequencer(goal=(4.5, 0.0), config=cfg)
        t, pos = self._advance_to_move(seq, cfg)
        dt = 1.0 / 120.0
        prev = 0.0
        peak = 0.0
        for _ in range(400):
            t += dt
            v = seq.update(t, pos, pos[2], 0.0, goal_distance=10.0).desired_speed
            self.assertLessEqual(v - prev, cfg.accel * dt + 1e-9, "acceleration limit exceeded")
            self.assertLessEqual(v, cfg.cruise_speed + 1e-9, "cruise speed cap exceeded")
            prev = peak = v
        self.assertAlmostEqual(peak, cfg.cruise_speed, places=2,
                               msg="plan must reach the cruise speed")

    def test_braking_curve_and_decel_limit(self) -> None:
        cfg = MissionConfig(hover_time=0.1, cruise_speed=1.2, decel=1.0, reach_radius=0.35)
        seq = MissionSequencer(goal=(4.5, 0.0), config=cfg)
        t, pos = self._advance_to_move(seq, cfg)
        dt = 1.0 / 120.0
        for _ in range(600):  # ramp up to cruise first
            t += dt
            seq.update(t, pos, pos[2], 0.0, goal_distance=10.0)
        dist = 0.6
        v_target = math.sqrt(2.0 * cfg.decel * (dist - cfg.reach_radius))
        prev = seq.update(t, pos, pos[2], 0.0, goal_distance=dist).desired_speed
        for _ in range(600):
            t += dt
            v = seq.update(t, pos, pos[2], 0.0, goal_distance=dist).desired_speed
            self.assertLessEqual(prev - v, cfg.decel * dt + 1e-9, "deceleration limit exceeded")
            self.assertGreaterEqual(v, 0.0)
            prev = v
        self.assertLess(abs(prev - v_target), 0.02,
                        "plan must converge to the braking curve sqrt(2*decel*(d-r_stop))")


class TestMovingObstacleReproducibility(unittest.TestCase):
    """Analytic kinematic movers: deterministic, genuinely moving, periodic."""

    def test_analytic_positions(self) -> None:
        from simulation.obstacles import MovingObstacle, MovingObstacleConfig

        cfg = MovingObstacleConfig(center=(-0.6, -2.0, 1.2),
                                   half_extents=(0.25, 0.25, 0.25),
                                   axis=(0.0, 1.0, 0.0),
                                   amplitude=1.8, period=7.0, phase=0.4)
        a, b = MovingObstacle(cfg), MovingObstacle(cfg)
        for t in (0.0, 1.7, 3.3, 5.9):
            self.assertEqual(a.position_at(t), b.position_at(t))
        p0 = a.position_at(0.0)
        p_half = a.position_at(cfg.period / 2.0)
        disp = math.sqrt(sum((q - p) ** 2 for p, q in zip(p0, p_half)))
        self.assertGreater(disp, 0.5 * cfg.amplitude, "mover must genuinely translate")
        for t in (0.3, 2.2):
            for x, y in zip(a.position_at(t), a.position_at(t + cfg.period)):
                self.assertAlmostEqual(x, y, places=9)
        t, eps = 1.6, 1e-5
        v = a.velocity_at(t)
        p_plus, p_minus = a.position_at(t + eps), a.position_at(t - eps)
        for i in range(3):
            fd = (p_plus[i] - p_minus[i]) / (2.0 * eps)
            self.assertAlmostEqual(v[i], fd, places=4)

    def test_scenario_run_reproducible(self) -> None:
        from scenarios import SCENARIOS
        from simulation.acmfds_mission import run_acmfds_mission

        paths = []
        for i in (1, 2):
            out = os.path.join(PROJECT_ROOT, "results", f"_test_repro_{i}.csv")
            run_acmfds_mission(SCENARIOS["moving-obstacles"], gui=False, seed=42,
                               csv_path=out, max_duration=2.0)
            paths.append(out)
        with open(paths[0], newline="") as fh:
            rows_a = list(csv.DictReader(fh))
        with open(paths[1], newline="") as fh:
            rows_b = list(csv.DictReader(fh))
        self.assertGreater(len(rows_a), 150)
        self.assertEqual(rows_a, rows_b, "same seed must reproduce identical telemetry")
        movers = [r["moving_obstacles"] for r in rows_a if r["moving_obstacles"]]
        self.assertTrue(movers, "mover positions must be logged")
        self.assertNotEqual(movers[0], movers[-1], "movers must change position over time")


class TestDashboardAPI(unittest.TestCase):
    """FastAPI dashboard endpoints over the real shared state object."""

    def test_status_scenarios_and_validation(self) -> None:
        from fastapi.testclient import TestClient
        from dashboard.server import app

        client = TestClient(app)
        self.assertEqual(client.get("/").status_code, 200)
        status = client.get("/api/status").json()
        # /api/status is the row-free variant by design (rows are WebSocket-only)
        for key in ("status", "total", "outcome", "summary", "config",
                    "scenario_name", "moving_enabled", "gps"):
            self.assertIn(key, status)
        self.assertNotIn("rows", status)
        self.assertEqual(status["status"], "idle")
        self.assertEqual(len(client.get("/api/scenarios").json()), 6)
        bad = client.post("/api/start", json={"target_altitude": 9.0})
        self.assertEqual(bad.status_code, 422)
        self.assertEqual(client.get("/api/status").json()["status"], "idle",
                         "a rejected request must not start a run")

    def test_native_window_is_opt_in(self) -> None:
        from dashboard.server import FlightConfig

        self.assertFalse(FlightConfig().native_window,
                         "default dashboard run must not open a native window")
        cfg = FlightConfig(native_window=True)   # accepted and validated
        self.assertTrue(cfg.native_window)


class TestSimulatedGps(unittest.TestCase):
    """GPS is simulated, server-side, and reference-configurable."""

    def test_reference_conversion_units(self) -> None:
        from telemetry.gps import M_PER_DEG_LAT, GpsReference

        ref = GpsReference(47.0, 8.0)
        self.assertEqual(ref.fix(0.0, 0.0), (47.0, 8.0))
        lat, _ = ref.fix(0.0, 111.32)
        self.assertAlmostEqual(lat - 47.0, 0.001, places=7)
        expected_dlon = 100.0 / (M_PER_DEG_LAT * math.cos(math.radians(47.0)))
        _, lon = ref.fix(100.0, 0.0)
        self.assertAlmostEqual(lon - 8.0, expected_dlon, places=9)
        lat, lon = ref.fix(-100.0, -111.32)
        self.assertAlmostEqual(lat - 47.0, -0.001, places=7)
        self.assertAlmostEqual(lon - 8.0, -expected_dlon, places=9)

    def test_api_gps_block_and_validation(self) -> None:
        from fastapi.testclient import TestClient
        from dashboard.server import app

        client = TestClient(app)
        ok = client.post("/api/gps", json={"enabled": True, "ref_lat": 47.4, "ref_lon": 8.5})
        self.assertEqual(ok.status_code, 200)
        self.assertTrue(ok.json()["simulated"])
        gps = client.get("/api/status").json()["gps"]
        self.assertTrue(gps["enabled"])
        self.assertTrue(gps["simulated"])
        self.assertAlmostEqual(gps["ref_lat"], 47.4)
        self.assertIsNone(gps["lat"], "no telemetry yet -> no simulated fix")
        self.assertEqual(
            client.post("/api/gps", json={"enabled": True, "ref_lat": 95.0}).status_code, 422)
        client.post("/api/gps", json={"enabled": False, "ref_lat": 47.3769, "ref_lon": 8.5417})

    def test_origin_gating_semantics(self) -> None:
        # No origin by default: the dashboard must say "Origin not set", never
        # fabricate coordinates. Enabling GPS without an origin yields no fix;
        # with an origin the fix is computed from the last logged physics row.
        from dashboard.server import DashboardState, GpsSetup
        from telemetry.gps import GpsReference

        store = DashboardState()
        gps = store.snapshot(-1)["gps"]
        self.assertFalse(gps["origin_set"])
        self.assertIsNone(gps["ref_lat"])
        self.assertIsNone(gps["lat"])

        store.set_gps(GpsSetup(enabled=True))            # enabled, still no origin
        gps = store.snapshot(-1)["gps"]
        self.assertTrue(gps["enabled"])
        self.assertFalse(gps["origin_set"])
        self.assertIsNone(gps["lat"], "no origin -> no simulated coordinates")

        store.set_gps(GpsSetup(enabled=True, ref_lat=10.903489, ref_lon=76.899439))
        gps = store.snapshot(-1)["gps"]
        self.assertTrue(gps["origin_set"])
        self.assertIsNone(gps["lat"], "origin set but no telemetry yet -> no fix")
        self.assertEqual(gps["origin_world"], [0.0, 0.0])
        self.assertEqual(gps["drone_start_world"], [-4.0, 0.0],
                         "default scenario start is 4 m west of the origin")

        # The drone never starts at world (0,0): its fix must reflect the
        # scenario start offset from the origin.
        store._on_step({"timestamp": 1.25, "position_x": -4.0, "position_y": 0.0})
        gps = store.snapshot(-1)["gps"]
        elat, elon = GpsReference(10.903489, 76.899439).fix(-4.0, 0.0)
        self.assertAlmostEqual(gps["lat"], round(elat, 7), places=7)
        self.assertAlmostEqual(gps["lon"], round(elon, 7), places=7)
        self.assertEqual(gps["t"], 1.25, "update time is the sim time of the source row")

        store.set_gps(GpsSetup(enabled=False, ref_lat=10.903489, ref_lon=76.899439))
        gps = store.snapshot(-1)["gps"]
        self.assertIsNone(gps["lat"], "disabled -> no coordinates, never stale ones")


class TestTelemetryIntegrity(unittest.TestCase):
    """Rows are JSON-safe (native scalars only) and carry explanation fields."""

    _rows = []
    _summary = None
    _world = None

    @classmethod
    def setUpClass(cls) -> None:
        from scenarios import SCENARIOS
        from simulation.acmfds_mission import ACMFDSMissionRunner

        out = os.path.join(PROJECT_ROOT, "results", "_test_telemetry.csv")
        runner = ACMFDSMissionRunner(SCENARIOS["obstacle"], gui=False, seed=42, csv_path=out)
        runner.setup()
        cls._world = runner.describe_world()
        report = runner.run(max_duration=8.0, on_step=cls._rows.append)
        runner.close()
        cls._summary = report.summary

    def test_world_description_matches_spawned_world(self) -> None:
        w = self._world
        self.assertEqual(w["goal"], [4.5, 0.0, 2.0])
        self.assertEqual(w["bounds"], {"x": [-7.5, 7.5], "y": [-5.0, 5.0], "z": [0.0, 6.0]})
        static = w["obstacles"]["static"]
        self.assertTrue(static, "obstacle scenario must describe its static obstacles")
        import json as _json
        _json.dumps(w)   # must be JSON-safe for the dashboard socket
        for o in static:
            self.assertEqual(len(o["position"]), 3)
            self.assertEqual(len(o["half_extents"]), 3)
            self.assertTrue(all(v > 0 for v in o["half_extents"]))

    def test_goal_distance_column_is_true_geometry(self) -> None:
        gx, gy = self._world["goal"][0], self._world["goal"][1]
        self.assertTrue(self._rows)
        for row in self._rows:
            self.assertIn("goal_distance", row)
            self.assertGreaterEqual(row["goal_distance"], 0.0)
        last = self._rows[-1]
        expected = math.hypot(gx - last["position_x"], gy - last["position_y"])
        self.assertAlmostEqual(last["goal_distance"], expected, places=2,
                               msg="logged goal_distance must equal the actual X/Y distance to goal")

    def test_movement_action_logged_every_step(self) -> None:
        # The dashboard's direction/action display reads movement_action: the
        # directional pick when ACMFDS selects one, else the decision in force.
        # It must never be empty during flight (the old selected_direction
        # column went blank outside avoidance maneuvers).
        for row in self._rows:
            self.assertTrue(row["movement_action"],
                            "movement_action must be logged on every step")
            if row["selected_direction"]:
                self.assertEqual(row["movement_action"], row["selected_direction"])
            else:
                self.assertEqual(row["movement_action"], row["acmfds_decision"])

    def test_rows_are_json_safe(self) -> None:
        import json as _json

        self.assertGreater(len(self._rows), 500)
        for row in self._rows:
            for key, value in row.items():
                self.assertNotIsInstance(value, np.generic,
                                         f"numpy scalar leaked into logged field {key!r}")
            _json.dumps(row)
        self.assertIsNotNone(self._summary)

    def test_explanation_and_decision_columns(self) -> None:
        for row in self._rows:
            self.assertTrue(row["decision_reason"], "decision explanation must be logged")
            self.assertTrue(row["mission_phase"], "mission phase must be logged")
        dirs = {r["selected_direction"] for r in self._rows if r["selected_direction"]}
        self.assertTrue(dirs, "selected direction must be logged")
        contexts = {r["acmfds_context"] for r in self._rows}
        self.assertTrue(contexts - {"NORMAL"},
                        "obstacle scenario must leave the NORMAL context")
        # Every obstacle-avoidance row compares directional candidates;
        # detour rows name the selected direction, and unsafe alternatives
        # are rejected explicitly with their clearance values.
        obstacle = [r for r in self._rows
                    if r["acmfds_context"] in ("OBSTACLE_NEAR", "OBSTACLE_CRITICAL")]
        self.assertTrue(obstacle)
        for row in obstacle:
            self.assertIn("safe", row["candidates"])
            if row["acmfds_decision"] == "AVOID_OBSTACLE":
                self.assertTrue(row["selected_direction"])
        rejected = [r for r in self._rows if "rejected" in r["candidates"]]
        self.assertTrue(rejected, "at least one unsafe candidate must be rejected explicitly")


def _summary_row(i: int, decision: str, context: str) -> dict:
    """Minimal logged row with the fields compute_summary reads."""
    return {
        "timestamp": i * (1.0 / 120.0), "step": i,
        "mission_completed": 0, "collision": 0,
        "obstacle_distance": 5.0, "target_altitude": 2.0, "altitude": 2.0,
        "battery_percentage": 100.0, "wind_speed": 0.0, "velocity_z": 0.0,
        "acmfds_decision": decision, "acmfds_context": context,
    }


class TestClearanceMarginAndCounting(unittest.TestCase):
    """Safety-margin gating, critical-band thresholds, and event counting."""

    def setUp(self) -> None:
        self.acmfds = ACMFDS()

    def test_critical_band_matches_config_threshold(self) -> None:
        # 0.9 m sits between obstacle_critical (0.8) and obstacle_avoid
        # (1.5): classified OBSTACLE_NEAR, not CRITICAL, not EMERGENCY.
        d = self.acmfds.decide(make_state(
            obstacle_distance=0.9, clearance_front=0.9,
            clearance_left=5.0, clearance_right=5.0,
        ))
        self.assertEqual(d.context, FlightContext.OBSTACLE_NEAR)
        # Below obstacle_critical the context is always EMERGENCY.
        d = self.acmfds.decide(make_state(
            obstacle_distance=0.7, clearance_front=0.7,
            clearance_left=5.0, clearance_right=5.0,
        ))
        self.assertEqual(d.context, FlightContext.EMERGENCY)

    def test_side_detour_requires_safety_margin(self) -> None:
        cfg = ACMFDSConfig()
        gate = cfg.obstacle_critical + cfg.safety_margin  # 1.1 m by default
        d = self.acmfds.decide(make_state(
            obstacle_distance=1.2, clearance_front=1.2,
            clearance_left=0.9, clearance_right=2.5,
        ))
        self.assertEqual(d.decision, DecisionType.AVOID_OBSTACLE)
        by_dir = {c.direction: c for c in d.candidates}
        self.assertFalse(by_dir["LEFT"].safe, "0.9 m side is under the 1.1 m gate")
        self.assertLess(by_dir["LEFT"].clearance, gate)
        self.assertTrue(by_dir["RIGHT"].safe)
        self.assertEqual(d.selected_direction, "RIGHT")
        self.assertLess(d.command.lateral_speed, 0.0, "detour right = -y body")

    def test_avoid_backs_off_in_critical_band(self) -> None:
        # Corridor above the detour gate (critical + margin) but inside the
        # avoid band: creep forward while detouring.
        d = self.acmfds.decide(make_state(
            obstacle_distance=1.3, clearance_front=1.3,
            clearance_left=2.0, clearance_right=0.4,
        ))
        self.assertEqual(d.decision, DecisionType.AVOID_OBSTACLE)
        self.assertGreater(d.command.forward_speed, 0.0)
        # Corridor tightened under the detour gate (but above critical):
        # back away instead of squeezing through.
        d = self.acmfds.decide(make_state(
            obstacle_distance=0.9, clearance_front=0.9,
            clearance_left=2.0, clearance_right=0.4,
        ))
        self.assertEqual(d.decision, DecisionType.AVOID_OBSTACLE)
        self.assertLess(d.command.forward_speed, 0.0)

    def test_post_maneuver_cooldown_limits_speed(self) -> None:
        cfg = ACMFDSConfig()
        d = self.acmfds.decide(make_state(maneuver_cooldown_s=0.6))
        self.assertEqual(d.decision, DecisionType.MOVE_FORWARD)
        self.assertLessEqual(d.command.speed_limit, cfg.cautious_speed,
                             "cooldown must hold cautious speed, not cruise")
        self.assertIn("post-maneuver caution", d.reason)
        # Cooldown expired -> normal cruise speed is restored.
        d = self.acmfds.decide(make_state(maneuver_cooldown_s=0.0))
        self.assertEqual(d.command.speed_limit, cfg.cruise_speed)
        self.assertNotIn("post-maneuver caution", d.reason)

    def test_emergency_events_count_entries_not_steps(self) -> None:
        rows = (
            [_summary_row(i, "MOVE_FORWARD", "NORMAL") for i in range(2)]
            + [_summary_row(i, "EMERGENCY", "EMERGENCY") for i in range(2, 5)]
            + [_summary_row(i, "MOVE_FORWARD", "NORMAL") for i in range(5, 7)]
            + [_summary_row(i, "EMERGENCY", "EMERGENCY") for i in range(7, 10)]
        )
        s = compute_summary(rows, "timeout")
        self.assertEqual(s.emergency_events, 2, "two distinct emergency entries")
        self.assertEqual(s.emergency_steps, 6, "six emergency decision steps")

    def test_safe_landing_events_count_entries_not_steps(self) -> None:
        rows = (
            [_summary_row(i, "MOVE_FORWARD", "NORMAL") for i in range(2)]
            + [_summary_row(i, "SAFE_LANDING", "LOW_BATTERY") for i in range(2, 6)]
        )
        s = compute_summary(rows, "timeout")
        self.assertEqual(s.safe_landing_events, 1)
        self.assertEqual(s.safe_landing_steps, 4)

    def test_landed_flag_tracks_touchdown_outcome(self) -> None:
        rows = [_summary_row(0, "DESCEND", "LANDING"),
                _summary_row(1, "MISSION_COMPLETE", "MISSION_COMPLETE")]
        rows[-1]["altitude"] = 0.03
        rows[-1]["velocity_z"] = -0.05
        s = compute_summary(rows, "landed")
        self.assertTrue(s.landed)
        self.assertTrue(s.mission_completed)
        self.assertAlmostEqual(s.final_altitude, 0.03)
        self.assertAlmostEqual(s.final_vertical_speed, -0.05)
        s = compute_summary(rows, "collision")
        self.assertFalse(s.landed, "a collision run must not report a landing")
        self.assertFalse(s.mission_completed)


class TestPauseResumeAndBatteryTrace(unittest.TestCase):
    """One live flight with a mid-flight pause/resume cycle.

    Verifies against the real runner+control path used by the dashboard:
    * while paused: no new rows, sim time / battery / position frozen;
    * resume continues the SAME episode (step counter + dt continuity);
    * Stop is distinct from Pause (aborts with 'stopped by operator');
    * the logged battery drain matches the documented Wh model recomputed
      from the logged thrust trace (no calculation or transfer bug).
    """

    _rows = []
    _frozen = None
    _frozen_later = None
    _report = None
    _model = {}

    @classmethod
    def setUpClass(cls) -> None:
        import threading
        import time as _time
        from scenarios import SCENARIOS
        from simulation.acmfds_mission import ACMFDSMissionRunner

        class Ctrl:
            def __init__(self) -> None:
                self.pause_requested = False
                self.stop_requested = False
                self.moving_enabled = True

        ctrl = Ctrl()
        rows = cls._rows
        out = os.path.join(PROJECT_ROOT, "results", "_test_pause.csv")
        runner = ACMFDSMissionRunner(SCENARIOS["normal"], gui=False, seed=42, csv_path=out)
        runner.setup()                        # creates drone/obstacles/world
        cls._model = {
            "dt": runner.dt,
            "max_thrust": runner.drone.max_thrust,
            "base_w": runner.battery.config.base_draw_w,
            "draw_w": runner.battery.config.thrust_draw_w,
            "cap_wh": runner.battery.config.capacity_wh,
        }

        def on_step(row):
            rows.append(row)
            if len(rows) == 400:
                ctrl.pause_requested = True   # pause at a safe step boundary

        def drive():
            cls._report = runner.run(max_duration=20.0, on_step=on_step, control=ctrl)

        th = threading.Thread(target=drive, daemon=True)
        th.start()
        deadline = _time.time() + 60
        while len(rows) < 400 and _time.time() < deadline:
            _time.sleep(0.02)
        _time.sleep(0.3)                      # settle into the paused spin
        cls._frozen = dict(len=len(rows), t=rows[-1]["timestamp"],
                           batt=rows[-1]["battery_percentage"],
                           x=rows[-1]["position_x"], step=rows[-1]["step"],
                           ep=rows[-1]["episode"])
        _time.sleep(0.5)
        cls._frozen_later = dict(len=len(rows), t=rows[-1]["timestamp"],
                                 batt=rows[-1]["battery_percentage"],
                                 x=rows[-1]["position_x"])
        ctrl.pause_requested = False          # resume the same flight
        deadline = _time.time() + 60
        while len(rows) <= 400 and _time.time() < deadline:
            _time.sleep(0.02)
        deadline = _time.time() + 60
        while len(rows) < 900 and _time.time() < deadline:
            _time.sleep(0.02)
        ctrl.stop_requested = True            # Stop (distinct from Pause)
        th.join(timeout=60)
        runner.close()

    def test_pause_freezes_physics_time_and_battery(self) -> None:
        self.assertEqual(self._frozen["len"], 400, "flight must reach the pause point")
        self.assertEqual(
            self._frozen_later,
            {k: self._frozen[k] for k in self._frozen_later},
            "while paused: no new rows; sim time, battery, and position frozen",
        )

    def test_resume_continues_same_episode(self) -> None:
        self.assertGreater(len(self._rows), 400, "resume must continue the flight")
        nxt = self._rows[400]
        self.assertEqual(nxt["step"], self._frozen["step"] + 1,
                         "step counter continues from the paused step")
        self.assertAlmostEqual(nxt["timestamp"], self._frozen["t"] + self._model["dt"],
                               places=4, msg="sim time continues with the same dt")
        self.assertEqual(nxt["episode"], self._frozen["ep"], "same episode, no new mission")
        self.assertLess(abs(nxt["position_x"] - self._rows[399]["position_x"]), 0.1,
                        "position continues from the frozen pose (no reset/teleport)")

    def test_stop_is_distinct_from_pause(self) -> None:
        self.assertEqual(self._report.outcome, "aborted")
        self.assertEqual(self._report.termination_reason, "stopped by operator")

    def test_battery_drain_matches_documented_model(self) -> None:
        # Battery at step i consumes the thrust ENTERING step i; row i logs
        # the thrust after actuation, so row[i-1].thrust_n drove battery step i.
        # Logged consumption rows[0]-rows[-1] therefore equals the sum over
        # steps 1..n-1 of power(row[i-1].thrust_n)*dt.
        m = self._model
        rows = self._rows
        frac = lambda tn: max(0.0, min(1.0, tn / m["max_thrust"]))
        wh = sum(
            (m["base_w"] + m["draw_w"] * frac(rows[i - 1]["thrust_n"]) ** 1.5)
            * (m["dt"] / 3600.0)
            for i in range(1, len(rows))
        )
        expected_pp = 100.0 * wh / m["cap_wh"]
        logged_pp = rows[0]["battery_percentage"] - rows[-1]["battery_percentage"]
        self.assertAlmostEqual(
            logged_pp, expected_pp, delta=0.05,
            msg="logged battery drain must match the documented Wh model",
        )

    def test_pause_resume_guards(self) -> None:
        from dashboard.server import DashboardState, RunControl

        st = DashboardState()
        self.assertFalse(st.pause()["ok"], "pause with no active run must be rejected")
        self.assertFalse(st.resume()["ok"], "resume with nothing paused must be rejected")
        st.status = "running"
        st.control = RunControl()
        self.assertTrue(st.pause()["ok"])
        self.assertTrue(st.control.pause_requested)
        self.assertFalse(st.pause()["ok"], "pausing an already-paused run must be rejected")
        self.assertTrue(st.resume()["ok"])
        self.assertFalse(st.control.pause_requested)
        self.assertFalse(st.resume()["ok"], "resuming a running (not paused) run must be rejected")
        # Stop from paused aborts the flight — it is not a second pause.
        st.pause()
        st.stop()
        self.assertTrue(st.control.stop_requested)
        self.assertFalse(st.control.pause_requested)


def run_all_tests() -> bool:
    print("=" * 60)
    print("RUNNING DRONE SIMULATION TEST SUITE")
    print("=" * 60)
    suite = unittest.TestLoader().loadTestsFromTestCase(TestGymnasiumEnv)
    for case in (TestFlightState, TestACMFDSRules, TestBatteryModel, TestWindModel,
                 TestMissionSequencer, TestActuationPath, TestDataDrivenController,
                 TestRaySensorsVsKnownObstacle, TestVisualPartsFollowBody,
                 TestHeadlessMissionLogging, TestDefaultPathFreeOfRL,
                 TestWaypointNavigation, TestObstacleAvoidanceMLModel,
                 TestSpeedPlanner, TestMovingObstacleReproducibility,
                 TestDashboardAPI, TestSimulatedGps, TestTelemetryIntegrity,
                 TestClearanceMarginAndCounting, TestPauseResumeAndBatteryTrace):
        suite.addTests(unittest.TestLoader().loadTestsFromTestCase(case))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


if __name__ == "__main__":
    sys.exit(0 if run_all_tests() else 1)
