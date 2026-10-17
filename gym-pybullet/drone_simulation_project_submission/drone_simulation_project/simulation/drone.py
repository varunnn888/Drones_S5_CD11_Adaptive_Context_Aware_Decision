"""Physical PyBullet Quadcopter Drone Model.

Implements:
- 6-DOF rigid-body PyBullet dynamics with actual mass and inertia
- Gravity and body-relative thrust/torque generation
- Linear and angular aerodynamic drag
- Virtual distance ray-casting sensors (8 directions)
- Exact custom visual quadcopter model with arms, motors, and animated propellers
"""

import math
from typing import List, Tuple, Sequence, Optional, Dict, Any
import numpy as np
import pybullet as p


class DroneModel:
    """Rigid-body quadcopter simulated via PyBullet forces and torques."""

    def __init__(
        self,
        physics_client_id: int = 0,
        mass: float = 1.0,
        arm_length: float = 0.25,
        max_thrust: float = 20.0,
        min_thrust: float = 0.0,
        max_torque: float = 1.5,
        linear_drag: float = 0.25,
        angular_drag: float = 0.15,
        max_ray_distance: float = 5.0,
        enable_visual_parts: bool = True,
    ) -> None:
        self.client_id = physics_client_id
        self.mass = float(mass)
        self.gravity = 9.81
        self.hover_thrust = self.mass * self.gravity  # 9.81 N for 1.0 kg
        self.arm_length = float(arm_length)
        self.max_thrust = float(max_thrust)
        self.min_thrust = float(min_thrust)
        self.max_torque = float(max_torque)
        self.linear_drag = float(linear_drag)
        self.angular_drag = float(angular_drag)
        self.max_ray_distance = float(max_ray_distance)
        self.enable_visual_parts = enable_visual_parts

        self.drone_id: int = -1
        self.visual_parts: List[Dict[str, Any]] = []
        self.propeller_angle: float = 0.0
        self.debug_line_ids: List[int] = []

        # 8 Sensor ray directions in body frame (unit vectors)
        # +X is forward, +Y is left, +Z is upward
        diag = 1.0 / math.sqrt(2.0)
        self.ray_directions = np.array([
            [1.0, 0.0, 0.0],    # 0: Front
            [-1.0, 0.0, 0.0],   # 1: Back
            [0.0, 1.0, 0.0],    # 2: Left
            [0.0, -1.0, 0.0],   # 3: Right
            [0.0, 0.0, 1.0],    # 4: Up
            [0.0, 0.0, -1.0],   # 5: Down
            [diag, diag, 0.0],   # 6: Front-Left
            [diag, -diag, 0.0],  # 7: Front-Right
        ], dtype=np.float32)

    # Named ray indices (keep sensor geometry consistent across the project).
    RAY_FRONT = 0
    RAY_BACK = 1
    RAY_LEFT = 2
    RAY_RIGHT = 3
    RAY_UP = 4
    RAY_DOWN = 5
    RAY_FRONT_LEFT = 6
    RAY_FRONT_RIGHT = 7

    def get_forward_clearances(self, draw_debug_rays: bool = False) -> Dict[str, float]:
        """Forward-cone clearances in meters (heading-consistent sensing).

        Combines the front ray with the two forward diagonal rays so that
        obstacle detection matches the direction the drone is facing.

        Returns keys:
            front / left / right : clearance in meters (5.0 = nothing in range)
            up                   : overhead clearance in meters
            front_obstacle_top   : top height (m) of the body hit by the front
                                   ray, or None when the front is clear — used
                                   to evaluate climb-over feasibility.
        """
        rays = self.get_sensor_readings(draw_debug_rays=draw_debug_rays)
        front_body = self._last_ray_hits[self.RAY_FRONT] if hasattr(self, "_last_ray_hits") else -1
        top = None
        if front_body not in (-1, self.drone_id) and front_body not in (
            v["id"] for v in self.visual_parts
        ):
            try:
                aabb_min, aabb_max = p.getAABB(
                    front_body, -1, physicsClientId=self.client_id
                )
                top = float(aabb_max[2])
            except Exception:
                top = None
        return {
            "front": float(rays[self.RAY_FRONT]) * self.max_ray_distance,
            "left": float(rays[self.RAY_FRONT_LEFT]) * self.max_ray_distance,
            "right": float(rays[self.RAY_FRONT_RIGHT]) * self.max_ray_distance,
            "up": float(rays[self.RAY_UP]) * self.max_ray_distance,
            "front_obstacle_top": top,
        }

    def spawn(
        self,
        position: Sequence[float] = (-4.0, 0.0, 2.0),
        orientation_euler: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> int:
        """Create the physical multi-body drone and visual assemblies."""
        self.cleanup()

        initial_orientation = p.getQuaternionFromEuler(
            list(orientation_euler),
            physicsClientId=self.client_id,
        )

        # ----------------------------------------------------
        # Main Physical Body with Mass & Collision Geometry
        # ----------------------------------------------------
        collision_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.25, 0.25, 0.08],
            physicsClientId=self.client_id,
        )

        visual_id = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.35, 0.25, 0.10],
            rgbaColor=[0.0, 0.35, 1.0, 1.0],  # Vibrant Blue
            physicsClientId=self.client_id,
        )

        self.drone_id = p.createMultiBody(
            baseMass=self.mass,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            basePosition=list(position),
            baseOrientation=initial_orientation,
            physicsClientId=self.client_id,
        )

        # Configure physical contact parameters
        p.changeDynamics(
            self.drone_id,
            -1,
            linearDamping=0.04,
            angularDamping=0.04,
            restitution=0.1,
            lateralFriction=0.8,
            physicsClientId=self.client_id,
        )

        # ----------------------------------------------------
        # Visual Model Parts (Arms, Motors, Propellers)
        # ----------------------------------------------------
        if self.enable_visual_parts:
            self._create_visual_accessories(position, initial_orientation)

        self.propeller_angle = 0.0
        return self.drone_id

    def _create_visual_accessories(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
    ) -> None:
        """Construct the visual quadcopter: X arms, motor housings, spinning
        propellers, colored nose marker, navigation LEDs, and landing gear.

        All parts are massless visual bodies whose offsets from the body
        origin are transformed by the drone's orientation every frame in
        :meth:`update_visual_parts` (never simply added to world coordinates).
        """
        # Motor centers: X configuration, +X is the nose.
        motor_offsets = [
            [0.55, 0.55, 0.10],
            [0.55, -0.55, 0.10],
            [-0.55, 0.55, 0.10],
            [-0.55, -0.55, 0.10],
        ]

        # 1. Four arms: long thin boxes centered midway along each diagonal.
        arm_len = math.sqrt(0.55 ** 2 + 0.55 ** 2)  # 0.778 m
        for x, y, _ in motor_offsets:
            arm_yaw = math.atan2(y, x)
            arm_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[arm_len / 2.0 + 0.04, 0.045, 0.035],
                rgbaColor=[0.15, 0.15, 0.15, 1.0],  # carbon dark
                physicsClientId=self.client_id,
            )
            arm_id = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=arm_visual,
                basePosition=list(position),
                physicsClientId=self.client_id,
            )
            self.visual_parts.append({
                "id": arm_id,
                "offset": [x / 2.0, y / 2.0, 0.02],
                "type": "arm",
                "angle": arm_yaw,
            })

        # 2. Four motor housings (cylinders) at the arm ends.
        for offset in motor_offsets:
            motor_visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=0.12,
                length=0.14,
                rgbaColor=[0.25, 0.25, 0.28, 1.0],
                physicsClientId=self.client_id,
            )
            motor_id = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=motor_visual,
                basePosition=list(position),
                physicsClientId=self.client_id,
            )
            self.visual_parts.append({
                "id": motor_id,
                "offset": offset,
                "type": "motor",
                "angle": 0.0,
            })

        # 3. Four propellers: two crossed thin blades each (X shape).
        for i, offset in enumerate(motor_offsets):
            for k in (0, 1):
                prop_visual = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[0.30, 0.028, 0.012],
                    rgbaColor=[0.08, 0.08, 0.08, 0.95],
                    physicsClientId=self.client_id,
                )
                prop_id = p.createMultiBody(
                    baseMass=0,
                    baseVisualShapeIndex=prop_visual,
                    basePosition=[position[0], position[1], position[2] + 0.19],
                    physicsClientId=self.client_id,
                )
                self.visual_parts.append({
                    "id": prop_id,
                    "offset": [offset[0], offset[1], offset[2] + 0.09],
                    "type": "propeller",
                    "angle": 0.0,
                    "spin_offset": k * (math.pi / 2.0),
                })

        # 4. Nose marker: bright orange front block + camera eye (front = +X).
        nose_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.13, 0.11, 0.06],
            rgbaColor=[1.0, 0.45, 0.05, 1.0],
            physicsClientId=self.client_id,
        )
        nose_id = p.createMultiBody(
            baseMass=0, baseVisualShapeIndex=nose_visual,
            basePosition=list(position), physicsClientId=self.client_id,
        )
        self.visual_parts.append({
            "id": nose_id, "offset": [0.36, 0.0, 0.02],
            "type": "nose", "angle": 0.0,
        })
        cam_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.05, 0.05, 0.035],
            rgbaColor=[0.02, 0.02, 0.02, 1.0],
            physicsClientId=self.client_id,
        )
        cam_id = p.createMultiBody(
            baseMass=0, baseVisualShapeIndex=cam_visual,
            basePosition=list(position), physicsClientId=self.client_id,
        )
        self.visual_parts.append({
            "id": cam_id, "offset": [0.38, 0.0, -0.09],
            "type": "nose", "angle": 0.0,
        })

        # 5. Navigation LEDs: red (left/+Y), green (right/-Y) on front motors.
        for (lx, ly), color in (
            ((0.55, 0.55), (0.95, 0.05, 0.05, 1.0)),
            ((0.55, -0.55), (0.05, 0.9, 0.2, 1.0)),
        ):
            led_visual = p.createVisualShape(
                p.GEOM_SPHERE, radius=0.035,
                rgbaColor=list(color),
                physicsClientId=self.client_id,
            )
            led_id = p.createMultiBody(
                baseMass=0, baseVisualShapeIndex=led_visual,
                basePosition=list(position), physicsClientId=self.client_id,
            )
            self.visual_parts.append({
                "id": led_id, "offset": [lx, ly, 0.185],
                "type": "motor", "angle": 0.0,
            })

        # 6. Landing gear: two small skids.
        for gy in (-0.16, 0.16):
            gear_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.16, 0.025, 0.045],
                rgbaColor=[0.1, 0.1, 0.1, 1.0],
                physicsClientId=self.client_id,
            )
            gear_id = p.createMultiBody(
                baseMass=0, baseVisualShapeIndex=gear_visual,
                basePosition=list(position), physicsClientId=self.client_id,
            )
            self.visual_parts.append({
                "id": gear_id, "offset": [0.0, gy, -0.14],
                "type": "gear", "angle": 0.0,
            })

    def update_visual_parts(self) -> None:
        """Synchronize every visual part with the physical body.

        Component offsets are rotated by the full body orientation
        matrix (roll + pitch + yaw), then translated by the body position.
        """
        if not self.visual_parts or self.drone_id < 0:
            return

        pos, orn = p.getBasePositionAndOrientation(
            self.drone_id,
            physicsClientId=self.client_id,
        )
        euler = p.getEulerFromQuaternion(orn, physicsClientId=self.client_id)
        yaw = euler[2]
        rot_mat = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)

        self.propeller_angle += 0.45

        for part in self.visual_parts:
            offset = np.array(part["offset"])
            world_pos = (np.array(pos) + rot_mat.dot(offset)).tolist()

            if part["type"] == "arm":
                part_orn = p.getQuaternionFromEuler(
                    [euler[0], euler[1], yaw + part["angle"]],
                    physicsClientId=self.client_id,
                )
            elif part["type"] == "propeller":
                part_orn = p.getQuaternionFromEuler(
                    [euler[0], euler[1], yaw + self.propeller_angle + part.get("spin_offset", 0.0)],
                    physicsClientId=self.client_id,
                )
            else:
                part_orn = orn

            p.resetBasePositionAndOrientation(
                part["id"],
                world_pos,
                part_orn,
                physicsClientId=self.client_id,
            )

    def apply_action(self, action: Sequence[float]) -> Dict[str, float]:
        """Apply continuous control action to the physical drone body.

        Action format: [thrust, roll_command, pitch_command, yaw_command]
        All normalized to [-1.0, 1.0].

        Args:
            action: 4-element continuous control array

        Returns:
            Dictionary of applied physical values
        """
        thrust_cmd = float(np.clip(action[0], -1.0, 1.0))
        roll_cmd = float(np.clip(action[1], -1.0, 1.0))
        pitch_cmd = float(np.clip(action[2], -1.0, 1.0))
        yaw_cmd = float(np.clip(action[3], -1.0, 1.0))

        # Convert normalized thrust_cmd [-1, 1] into physical Newtons:
        # action=0.0 -> hover_thrust (9.81 N)
        # action=1.0 -> max_thrust (20.0 N)
        # action=-1.0 -> min_thrust (0.0 N)
        if thrust_cmd >= 0.0:
            thrust = self.hover_thrust + thrust_cmd * (self.max_thrust - self.hover_thrust)
        else:
            thrust = self.hover_thrust + thrust_cmd * (self.hover_thrust - self.min_thrust)
        thrust = float(np.clip(thrust, self.min_thrust, self.max_thrust))

        # Map commands to body torques (N*m)
        torque_roll = roll_cmd * self.max_torque
        torque_pitch = pitch_cmd * self.max_torque
        torque_yaw = yaw_cmd * (self.max_torque * 0.5)

        # Apply upward thrust in drone body frame (+Z body)
        # p.applyExternalForce with LINK_FRAME applies force in local frame!
        p.applyExternalForce(
            objectUniqueId=self.drone_id,
            linkIndex=-1,
            forceObj=[0.0, 0.0, thrust],
            posObj=[0.0, 0.0, 0.0],
            flags=p.LINK_FRAME,
            physicsClientId=self.client_id,
        )

        # Apply control torques in body frame
        p.applyExternalTorque(
            objectUniqueId=self.drone_id,
            linkIndex=-1,
            torqueObj=[torque_roll, torque_pitch, torque_yaw],
            flags=p.LINK_FRAME,
            physicsClientId=self.client_id,
        )

        # Apply aerodynamic drag opposing motion
        linear_vel, angular_vel = p.getBaseVelocity(
            self.drone_id,
            physicsClientId=self.client_id,
        )
        base_pos, _ = p.getBasePositionAndOrientation(
            self.drone_id, physicsClientId=self.client_id
        )
        drag_force = [-self.linear_drag * v for v in linear_vel]
        drag_torque = [-self.angular_drag * w for w in angular_vel]

        # WORLD_FRAME posObj is a world-coordinate application point: [0,0,0]
        # is the world origin, not the drone. Apply drag at the body COM so
        # it does not create a spurious lever-arm torque.
        p.applyExternalForce(
            objectUniqueId=self.drone_id,
            linkIndex=-1,
            forceObj=drag_force,
            posObj=list(base_pos),
            flags=p.WORLD_FRAME,
            physicsClientId=self.client_id,
        )
        p.applyExternalTorque(
            objectUniqueId=self.drone_id,
            linkIndex=-1,
            torqueObj=drag_torque,
            flags=p.WORLD_FRAME,
            physicsClientId=self.client_id,
        )

        return {
            "thrust": thrust,
            "torque_roll": torque_roll,
            "torque_pitch": torque_pitch,
            "torque_yaw": torque_yaw,
        }

    def get_sensor_readings(self, draw_debug_rays: bool = False) -> np.ndarray:
        """Perform virtual distance sensing using PyBullet rayTestBatch.

        Casts 8 rays in body-relative directions:
        [Front, Back, Left, Right, Up, Down, Front-Left, Front-Right]

        Returns:
            np.ndarray of shape (8,) with normalized distances in [0.0, 1.0]
            (1.0 means no obstacle detected within max_ray_distance)
        """
        pos, orn = p.getBasePositionAndOrientation(
            self.drone_id,
            physicsClientId=self.client_id,
        )
        rot_mat = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)

        ray_from_positions = []
        ray_to_positions = []

        for dir_vec in self.ray_directions:
            world_dir = rot_mat.dot(dir_vec)
            start_pos = [
                pos[0] + world_dir[0] * 0.35,
                pos[1] + world_dir[1] * 0.35,
                pos[2] + world_dir[2] * 0.35,
            ]
            end_pos = [
                pos[0] + world_dir[0] * self.max_ray_distance,
                pos[1] + world_dir[1] * self.max_ray_distance,
                pos[2] + world_dir[2] * self.max_ray_distance,
            ]
            ray_from_positions.append(start_pos)
            ray_to_positions.append(end_pos)

        results = p.rayTestBatch(
            ray_from_positions,
            ray_to_positions,
            physicsClientId=self.client_id,
        )

        distances = np.ones(len(self.ray_directions), dtype=np.float32)
        # Remember which body each ray hit (for clearance queries).
        self._last_ray_hits: list = [int(h[0]) for h in results]

        # Debug lines are permanent items replaced in place (lifeTime=0 +
        # replaceItemUniqueId). Remove/recreate with a finite lifeTime
        # races expiry and triggers PyBullet's "User debug draw failed"
        # warnings; in-place replacement on permanent items cannot.
        if draw_debug_rays and len(self.debug_line_ids) != len(self.ray_directions):
            for line_id in self.debug_line_ids:
                try:
                    p.removeUserDebugItem(line_id, physicsClientId=self.client_id)
                except Exception:
                    pass
            self.debug_line_ids = [
                p.addUserDebugLine(
                    [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], lineWidth=1.5,
                    physicsClientId=self.client_id,
                )
                for _ in self.ray_directions
            ]
        elif not draw_debug_rays and self.debug_line_ids:
            for line_id in self.debug_line_ids:
                try:
                    p.removeUserDebugItem(line_id, physicsClientId=self.client_id)
                except Exception:
                    pass
            self.debug_line_ids.clear()

        for i, hit in enumerate(results):
            hit_object_id = hit[0]
            hit_fraction = hit[2]

            # Ignore hits on self or visual attachments
            visual_ids = [v["id"] for v in self.visual_parts]
            if hit_object_id != -1 and hit_object_id != self.drone_id and hit_object_id not in visual_ids:
                distances[i] = float(np.clip(hit_fraction, 0.0, 1.0))
                color = [1.0, 0.2, 0.2]  # Red if obstacle hit
            else:
                distances[i] = 1.0
                color = [0.2, 1.0, 0.2]  # Green if clear

            if draw_debug_rays:
                hit_pos = hit[3] if distances[i] < 1.0 else ray_to_positions[i]
                p.addUserDebugLine(
                    ray_from_positions[i],
                    hit_pos,
                    lineColorRGB=color,
                    lineWidth=1.5,
                    replaceItemUniqueId=self.debug_line_ids[i],
                    physicsClientId=self.client_id,
                )

        return distances

    def get_position(self) -> np.ndarray:
        """Return [x, y, z] position as numpy array."""
        pos, _ = p.getBasePositionAndOrientation(self.drone_id, physicsClientId=self.client_id)
        return np.array(pos, dtype=np.float32)

    def get_orientation_euler(self) -> np.ndarray:
        """Return [roll, pitch, yaw] orientation in radians."""
        _, orn = p.getBasePositionAndOrientation(self.drone_id, physicsClientId=self.client_id)
        euler = p.getEulerFromQuaternion(orn, physicsClientId=self.client_id)
        return np.array(euler, dtype=np.float32)

    def get_linear_velocity(self) -> np.ndarray:
        """Return [vx, vy, vz] velocity as numpy array."""
        vel, _ = p.getBaseVelocity(self.drone_id, physicsClientId=self.client_id)
        return np.array(vel, dtype=np.float32)

    def get_angular_velocity(self) -> np.ndarray:
        """Return [wx, wy, wz] angular velocity as numpy array."""
        _, omega = p.getBaseVelocity(self.drone_id, physicsClientId=self.client_id)
        return np.array(omega, dtype=np.float32)

    def reset_state(
        self,
        position: Sequence[float] = (-4.0, 0.0, 2.0),
        orientation_euler: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        """Reset drone state to given position and zero velocities."""
        orn_quat = p.getQuaternionFromEuler(list(orientation_euler), physicsClientId=self.client_id)
        p.resetBasePositionAndOrientation(
            self.drone_id,
            list(position),
            orn_quat,
            physicsClientId=self.client_id,
        )
        p.resetBaseVelocity(
            self.drone_id,
            linearVelocity=[0.0, 0.0, 0.0],
            angularVelocity=[0.0, 0.0, 0.0],
            physicsClientId=self.client_id,
        )
        if self.enable_visual_parts:
            self.update_visual_parts()

    def cleanup(self) -> None:
        """Clean up all physics and debug bodies."""
        if self.drone_id >= 0:
            try:
                p.removeBody(self.drone_id, physicsClientId=self.client_id)
            except Exception:
                pass
            self.drone_id = -1

        for part in self.visual_parts:
            try:
                p.removeBody(part["id"], physicsClientId=self.client_id)
            except Exception:
                pass
        self.visual_parts.clear()

        for line_id in self.debug_line_ids:
            try:
                p.removeUserDebugItem(line_id, physicsClientId=self.client_id)
            except Exception:
                pass
        self.debug_line_ids.clear()
