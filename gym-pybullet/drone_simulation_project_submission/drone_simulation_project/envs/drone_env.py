"""Gymnasium-compatible PyBullet 3D Drone Environment for Reinforcement Learning.

Provides:
- 6-DOF physical quadcopter dynamics with true gravity and forces/torques
- Continuous normalized action space: [thrust, roll, pitch, yaw]
- Rich continuous observation space: positions, velocities, attitudes, target vectors,
  and 8-direction virtual ray distance sensors
- Configurable obstacle layouts (static reference vs. randomized)
- Carefully calibrated reward function balancing progress, stability, and collision penalties
- Integration with ACMFDS decision system as a safety and telemetry layer
- Strict compliance with Gymnasium v1.0+ API and check_env validator
"""

import math
from typing import Optional, Dict, Any, Tuple, Sequence, List
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data

try:
    from ..simulation.drone import DroneModel
    from ..simulation.obstacles import ObstacleManager, DEFAULT_STATIC_OBSTACLES
    from ..decision.acmfds import ACMFDS
    from ..state.flight_state import FlightState
except (ImportError, ValueError):
    from simulation.drone import DroneModel
    from simulation.obstacles import ObstacleManager, DEFAULT_STATIC_OBSTACLES
    from decision.acmfds import ACMFDS
    from state.flight_state import FlightState


class DroneEnv(gym.Env):
    """Gymnasium Environment for 3D Quadcopter Navigation and Obstacle Avoidance.

    Observation Space (27 dimensions, Box):
        [0:3]   - Drone Position: x, y, z in meters
        [3:6]   - Drone Linear Velocity: vx, vy, vz in m/s
        [6:9]   - Drone Orientation (Euler): roll, pitch, yaw in radians
        [9:12]  - Drone Angular Velocity: wx, wy, wz in rad/s
        [12:15] - Target Position: target_x, target_y, target_z in meters
        [15:18] - Relative Target Vector: target_pos - drone_pos in meters
        [18]    - Euclidean Distance to Target in meters
        [19:27] - 8 Obstacle Ray Distances: normalized [0, 1] for
                  [Front, Back, Left, Right, Up, Down, Front-Left, Front-Right]

    Action Space (4 dimensions, Box [-1.0, 1.0]):
        [0] - Normalized Thrust:
              -1.0 = min thrust (0 N), 0.0 = hover thrust (9.81 N), +1.0 = max thrust (20 N)
        [1] - Roll Command: normalized torque [-1, 1]
        [2] - Pitch Command: normalized torque [-1, 1]
        [3] - Yaw Command: normalized torque [-1, 1]
    """

    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": 60,
    }

    def __init__(
        self,
        render_mode: Optional[str] = None,
        randomize_obstacles: bool = False,
        num_random_obstacles: int = 3,
        start_position: Sequence[float] = (-4.0, 0.0, 2.0),
        target_position: Sequence[float] = (4.5, 0.0, 2.0),
        max_episode_steps: int = 500,
        time_step: float = 1.0 / 120.0,
        substeps: int = 2,
    ) -> None:
        super().__init__()

        self.render_mode = render_mode
        self.randomize_obstacles = randomize_obstacles
        self.num_random_obstacles = num_random_obstacles
        self.default_start = np.array(start_position, dtype=np.float32)
        self.default_target = np.array(target_position, dtype=np.float32)
        self.max_episode_steps = max_episode_steps
        self.time_step = float(time_step)
        self.substeps = int(substeps)

        # Permitted flight boundary [min_x, max_x, min_y, max_y, min_z, max_z]
        self.bounds = np.array([-7.0, 7.0, -4.0, 4.0, 0.12, 5.0], dtype=np.float32)
        self.target_radius = 0.40  # Sphere of acceptance

        # Action space: [thrust, roll, pitch, yaw]
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(4,),
            dtype=np.float32,
        )

        # Observation space bounds (27 elements)
        obs_low = np.array(
            [
                -10.0, -10.0, 0.0,      # pos x, y, z
                -15.0, -15.0, -15.0,    # vel x, y, z
                -np.pi, -np.pi, -np.pi, # roll, pitch, yaw
                -20.0, -20.0, -20.0,    # ang vel x, y, z
                -10.0, -10.0, 0.0,      # target x, y, z
                -20.0, -20.0, -20.0,    # rel target vector
                0.0,                    # target distance
            ] + [0.0] * 8,              # 8 ray sensors
            dtype=np.float32,
        )

        obs_high = np.array(
            [
                10.0, 10.0, 10.0,       # pos x, y, z
                15.0, 15.0, 15.0,       # vel x, y, z
                np.pi, np.pi, np.pi,    # roll, pitch, yaw
                20.0, 20.0, 20.0,       # ang vel x, y, z
                10.0, 10.0, 10.0,       # target x, y, z
                20.0, 20.0, 20.0,       # rel target vector
                30.0,                   # target distance
            ] + [1.0] * 8,              # 8 ray sensors
            dtype=np.float32,
        )

        self.observation_space = spaces.Box(
            low=obs_low,
            high=obs_high,
            dtype=np.float32,
        )

        # Simulation handles
        self.client_id: int = -1
        self.plane_id: int = -1
        self.target_marker_id: int = -1
        self.drone: Optional[DroneModel] = None
        self.obstacle_manager: Optional[ObstacleManager] = None
        self.acmfds: ACMFDS = ACMFDS()

        # Episode state variables
        self.current_step = 0
        self.prev_distance_to_target = 0.0
        self.target_pos = self.default_target.copy()
        self.start_pos = self.default_start.copy()

        # Connect PyBullet server
        self._init_pybullet()

    def _init_pybullet(self) -> None:
        """Establish PyBullet connection and configure world."""
        is_gui = (self.render_mode == "human")
        conn_mode = p.GUI if is_gui else p.DIRECT
        self.client_id = p.connect(conn_mode)

        if self.client_id < 0:
            raise RuntimeError("Failed to start PyBullet instance in DroneEnv.")

        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)
        p.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        p.setTimeStep(self.time_step, physicsClientId=self.client_id)
        p.setPhysicsEngineParameter(
            numSolverIterations=10,
            fixedTimeStep=self.time_step,
            physicsClientId=self.client_id,
        )

        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)

        # Instantiate drone and obstacle managers
        self.drone = DroneModel(
            physics_client_id=self.client_id,
            enable_visual_parts=(self.render_mode == "human"),
        )
        self.drone.spawn(position=self.default_start)

        self.obstacle_manager = ObstacleManager(
            physics_client_id=self.client_id,
            randomize=self.randomize_obstacles,
            num_random_obstacles=self.num_random_obstacles,
        )
        self.obstacle_manager.setup_obstacles(
            start_pos=self.default_start,
            goal_pos=self.default_target,
        )

        # Create target visual sphere in GUI mode
        if self.render_mode == "human":
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.client_id)
            visual_shape = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=0.25,
                rgbaColor=[0.1, 0.9, 0.2, 0.85],
                physicsClientId=self.client_id,
            )
            self.target_marker_id = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=visual_shape,
                basePosition=list(self.default_target),
                physicsClientId=self.client_id,
            )
            self._update_camera()

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Reset environment to initial state."""
        super().reset(seed=seed)

        self.current_step = 0

        # Optional start perturbation
        if options and options.get("random_start", False):
            offset_x = float(self.np_random.uniform(-0.5, 0.5))
            offset_y = float(self.np_random.uniform(-0.5, 0.5))
            offset_z = float(self.np_random.uniform(-0.2, 0.2))
            self.start_pos = self.default_start + np.array([offset_x, offset_y, offset_z], dtype=np.float32)
        else:
            self.start_pos = self.default_start.copy()

        # Re-setup obstacles (randomized if enabled)
        if self.obstacle_manager:
            self.obstacle_manager.setup_obstacles(
                np_random=self.np_random,
                start_pos=self.start_pos,
                goal_pos=self.target_pos,
            )

        # Reset drone physical pose
        if self.drone:
            self.drone.reset_state(
                position=self.start_pos,
                orientation_euler=(0.0, 0.0, 0.0),
            )

        # Settle physics for a few steps
        for _ in range(5):
            p.stepSimulation(physicsClientId=self.client_id)

        # Compute initial observation & telemetry
        obs = self._get_observation()
        drone_pos = self.drone.get_position() if self.drone else self.start_pos
        self.prev_distance_to_target = float(np.linalg.norm(self.target_pos - drone_pos))

        sensor_dists = obs[19:27] * (self.drone.max_ray_distance if self.drone else 5.0)
        min_obstacle_dist = float(np.min(sensor_dists))

        decision = self.acmfds.decide(FlightState(
            altitude=float(drone_pos[2]),
            target_altitude=float(self.target_pos[2]),
            obstacle_distance=min_obstacle_dist,
            clearance_front=min_obstacle_dist,
            clearance_left=min_obstacle_dist,
            clearance_right=min_obstacle_dist,
            goal_distance=self.prev_distance_to_target,
        ))

        info = {
            "distance_to_target": self.prev_distance_to_target,
            "altitude": float(drone_pos[2]),
            "collision": False,
            "ground_contact": False,
            "out_of_bounds": False,
            "target_reached": False,
            "min_obstacle_distance": min_obstacle_dist,
            "acmfds_decision": decision.decision.value,
            "acmfds_context": decision.context.value,
            "acmfds_reason": decision.reason,
            "episode_step": 0,
        }

        if self.render_mode == "human":
            self.render()

        return obs, info

    def step(
        self,
        action: np.ndarray,
    ) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """Advance environment by one control step applying continuous action."""
        self.current_step += 1

        # Clip action to action space bounds
        act = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)

        # Apply action and advance physics by substeps
        if self.drone:
            for _ in range(self.substeps):
                self.drone.apply_action(act)
                p.stepSimulation(physicsClientId=self.client_id)

        # Retrieve drone state
        obs = self._get_observation()
        pos = obs[0:3]
        vel = obs[3:6]
        euler = obs[6:9]
        dist_to_target = float(obs[18])
        sensor_dists = obs[19:27] * (self.drone.max_ray_distance if self.drone else 5.0)
        min_obstacle_dist = float(np.min(sensor_dists))

        # Check collision with obstacles
        obstacle_collision = self._check_obstacle_collision()

        # Check ground contact
        ground_contact = self._check_ground_collision(pos[2])

        # Check permitted boundary
        out_of_bounds = self._check_out_of_bounds(pos)

        # Check target reached
        target_reached = bool(dist_to_target < self.target_radius)

        # Check if drone has critically flipped
        excessive_tilt = bool(abs(euler[0]) > 1.3 or abs(euler[1]) > 1.3)

        # ACMFDS decision
        acmfds_result = self.acmfds.decide(FlightState(
            altitude=float(pos[2]),
            target_altitude=float(self.target_pos[2]),
            obstacle_distance=min_obstacle_dist,
            clearance_front=min_obstacle_dist,
            clearance_left=min_obstacle_dist,
            clearance_right=min_obstacle_dist,
            velocity_x=float(vel[0]), velocity_y=float(vel[1]), velocity_z=float(vel[2]),
            roll=float(euler[0]), pitch=float(euler[1]), yaw=float(euler[2]),
            goal_distance=dist_to_target,
        ))

        # Reward Calculation
        reward = self._compute_reward(
            pos=pos,
            euler=euler,
            act=act,
            dist_to_target=dist_to_target,
            min_obstacle_dist=min_obstacle_dist,
            target_reached=target_reached,
            obstacle_collision=obstacle_collision,
            ground_contact=ground_contact,
            out_of_bounds=out_of_bounds,
            excessive_tilt=excessive_tilt,
        )

        # Termination (terminal state reached)
        terminated = bool(
            target_reached
            or obstacle_collision
            or ground_contact
            or out_of_bounds
            or excessive_tilt
        )

        # Truncation (time limit reached)
        truncated = bool(self.current_step >= self.max_episode_steps and not terminated)

        self.prev_distance_to_target = dist_to_target

        info = {
            "distance_to_target": dist_to_target,
            "altitude": float(pos[2]),
            "altitude_error": float(self.target_pos[2] - pos[2]),
            "collision": obstacle_collision,
            "ground_contact": ground_contact,
            "out_of_bounds": out_of_bounds,
            "target_reached": target_reached,
            "excessive_tilt": excessive_tilt,
            "min_obstacle_distance": min_obstacle_dist,
            "acmfds_decision": acmfds_result.decision.value,
            "acmfds_context": acmfds_result.context.value,
            "acmfds_reason": acmfds_result.reason,
            "episode_step": self.current_step,
            "episode_time": self.current_step * self.time_step * self.substeps,
        }

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, info

    def _get_observation(self) -> np.ndarray:
        """Construct normalized continuous observation array."""
        if self.drone is None:
            return np.zeros(27, dtype=np.float32)

        pos = self.drone.get_position()
        vel = self.drone.get_linear_velocity()
        euler = self.drone.get_orientation_euler()
        ang_vel = self.drone.get_angular_velocity()

        target_pos = self.target_pos
        rel_target = target_pos - pos
        dist_to_target = float(np.linalg.norm(rel_target))

        # Ray sensors (draw debug lines only when human rendering)
        draw_rays = (self.render_mode == "human")
        sensor_readings = self.drone.get_sensor_readings(draw_debug_rays=draw_rays)

        obs = np.concatenate([
            pos,                  # [0:3]
            vel,                  # [3:6]
            euler,                # [6:9]
            ang_vel,              # [9:12]
            target_pos,           # [12:15]
            rel_target,           # [15:18]
            [dist_to_target],     # [18]
            sensor_readings,      # [19:27]
        ]).astype(np.float32)

        return np.clip(obs, self.observation_space.low, self.observation_space.high)

    def _compute_reward(
        self,
        pos: np.ndarray,
        euler: np.ndarray,
        act: np.ndarray,
        dist_to_target: float,
        min_obstacle_dist: float,
        target_reached: bool,
        obstacle_collision: bool,
        ground_contact: bool,
        out_of_bounds: bool,
        excessive_tilt: bool,
    ) -> float:
        """Calculate shaped reinforcement learning reward."""
        # 1. Target progress reward (positive when moving toward target)
        progress = (self.prev_distance_to_target - dist_to_target)
        progress_reward = float(progress * 15.0)

        # 2. Target reached large bonus
        target_bonus = 100.0 if target_reached else 0.0

        # 3. Base alive bonus to encourage staying aloft within limits
        alive_bonus = 0.15

        # 4. Altitude error penalty
        alt_error = abs(pos[2] - self.target_pos[2])
        altitude_penalty = -0.75 * (alt_error ** 2)

        # 5. Excessive tilt penalty (maintain horizontal attitude)
        tilt_penalty = -0.4 * (euler[0] ** 2 + euler[1] ** 2)

        # 6. Control effort penalty (smooth actuation)
        effort_penalty = -0.01 * float(np.sum(act ** 2))

        # 7. Obstacle proximity penalty
        proximity_penalty = 0.0
        safe_clearance = 0.8
        if min_obstacle_dist < safe_clearance:
            proximity_penalty = -2.5 * (safe_clearance - min_obstacle_dist)

        # 8. Terminal penalties
        collision_penalty = -60.0 if obstacle_collision else 0.0
        ground_penalty = -50.0 if ground_contact else 0.0
        boundary_penalty = -50.0 if out_of_bounds else 0.0
        tilt_crash_penalty = -40.0 if excessive_tilt else 0.0

        # 9. Small time step penalty
        step_penalty = -0.05

        total_reward = (
            progress_reward
            + target_bonus
            + alive_bonus
            + altitude_penalty
            + tilt_penalty
            + effort_penalty
            + proximity_penalty
            + collision_penalty
            + ground_penalty
            + boundary_penalty
            + tilt_crash_penalty
            + step_penalty
        )

        return float(total_reward)

    def _check_obstacle_collision(self) -> bool:
        """Check if drone has contacted any obstacle body."""
        if not self.drone or not self.obstacle_manager:
            return False

        contact_points = p.getContactPoints(
            bodyA=self.drone.drone_id,
            physicsClientId=self.client_id,
        )
        for contact in contact_points:
            body_b = contact[2]
            if self.obstacle_manager.is_obstacle(body_b):
                return True
        return False

    def _check_ground_collision(self, altitude: float) -> bool:
        """Check if drone collided with ground plane."""
        if altitude <= self.bounds[4]:
            return True

        if self.drone and self.plane_id >= 0:
            contacts = p.getContactPoints(
                bodyA=self.drone.drone_id,
                bodyB=self.plane_id,
                physicsClientId=self.client_id,
            )
            return len(contacts) > 0
        return False

    def _check_out_of_bounds(self, pos: np.ndarray) -> bool:
        """Check if drone position left the permitted bounding box."""
        if pos[0] < self.bounds[0] or pos[0] > self.bounds[1]:
            return True
        if pos[1] < self.bounds[2] or pos[1] > self.bounds[3]:
            return True
        if pos[2] > self.bounds[5]:
            return True
        return False

    def render(self) -> Optional[np.ndarray]:
        """Render frame or update visual camera tracking."""
        if self.render_mode == "human":
            if self.drone:
                self.drone.update_visual_parts()
                pos = self.drone.get_position()
                self._update_camera(target=pos)
            return None
        elif self.render_mode == "rgb_array":
            # Offscreen RGB rendering
            view_matrix = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=[0, 0, 1.2],
                distance=7.0,
                yaw=45,
                pitch=-30,
                roll=0,
                upAxisIndex=2,
                physicsClientId=self.client_id,
            )
            proj_matrix = p.computeProjectionMatrixFOV(
                fov=60,
                aspect=1.0,
                nearVal=0.1,
                farVal=100.0,
                physicsClientId=self.client_id,
            )
            _, _, px, _, _ = p.getCameraImage(
                width=320,
                height=320,
                viewMatrix=view_matrix,
                projectionMatrix=proj_matrix,
                physicsClientId=self.client_id,
            )
            rgb_array = np.array(px, dtype=np.uint8).reshape((320, 320, 4))[:, :, :3]
            return rgb_array
        return None

    def _update_camera(self, target: Optional[Sequence[float]] = None) -> None:
        """Point PyBullet GUI camera to target position."""
        if self.render_mode != "human" or self.client_id < 0:
            return
        cam_target = list(target) if target is not None else [0.0, 0.0, 1.2]
        p.resetDebugVisualizerCamera(
            cameraDistance=6.5,
            cameraYaw=45,
            cameraPitch=-25,
            cameraTargetPosition=cam_target,
            physicsClientId=self.client_id,
        )

    def close(self) -> None:
        """Safely disconnect PyBullet and release all resources."""
        if self.drone:
            self.drone.cleanup()
            self.drone = None

        if self.obstacle_manager:
            self.obstacle_manager.clear_obstacles()
            self.obstacle_manager = None

        if self.client_id >= 0:
            try:
                p.disconnect(physicsClientId=self.client_id)
            except Exception:
                pass
            self.client_id = -1
