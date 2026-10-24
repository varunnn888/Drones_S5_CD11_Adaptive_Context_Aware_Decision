"""PyBullet physics environment wrapper and standalone visual simulation.

Provides:
- Robust PyBullet simulation server lifecycle management (GUI and DIRECT headless)
- Fixed physics timestep configuration
- 3D camera controls (overview and drone-following modes)
- Target and trajectory visual markers in GUI
- Standalone runnable demonstration with Data-Driven Controller navigation
"""

import math
import os
import sys
import time
from typing import Optional, Sequence, List

import pybullet as p
import pybullet_data
import numpy as np

# Ensure project root is importable when run as a plain script.
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

try:
    from .drone import DroneModel
    from .obstacles import ObstacleManager
except (ImportError, ValueError):
    from simulation.drone import DroneModel
    from simulation.obstacles import ObstacleManager


class PyBulletSimulation:
    """Manages the PyBullet physics server and scene geometry."""

    def __init__(
        self,
        gui: bool = False,
        time_step: float = 1.0 / 120.0,
        gravity: float = -9.81,
    ) -> None:
        self.gui = gui
        self.time_step = float(time_step)
        self.gravity = float(gravity)

        self.client_id: int = -1
        self.plane_id: int = -1
        self.target_visual_id: int = -1
        self.trajectory_line_ids: List[int] = []

    def connect(self) -> int:
        """Connect to PyBullet server (GUI or DIRECT headless)."""
        if self.client_id >= 0:
            return self.client_id

        connection_mode = p.GUI if self.gui else p.DIRECT
        self.client_id = p.connect(connection_mode)

        if self.client_id < 0:
            raise RuntimeError("Failed to connect to PyBullet physics server.")

        # PyBullet default assets (plane.urdf etc.)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)

        # Simulation physics configuration
        p.setGravity(0, 0, self.gravity, physicsClientId=self.client_id)
        p.setTimeStep(self.time_step, physicsClientId=self.client_id)
        p.setPhysicsEngineParameter(
            numSolverIterations=10,
            fixedTimeStep=self.time_step,
            physicsClientId=self.client_id,
        )

        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)

        # Configure GUI settings
        if self.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.client_id)
            self._draw_ground_grid()
            self.set_camera(distance=7.5, yaw=45, pitch=-25, target=(0.0, 0.0, 1.2))

        return self.client_id

    def _draw_ground_grid(self, extent: float = 8.0, step: float = 1.0) -> None:
        """Draw subtle reference grid lines on the ground plane."""
        color_major = [0.55, 0.62, 0.68]
        color_minor = [0.75, 0.8, 0.85]
        n = int(extent / step)
        for i in range(-n, n + 1):
            major = (i % 2 == 0)
            color = color_major if major else color_minor
            width = 1.2 if major else 0.6
            p.addUserDebugLine(
                [i * step, -extent, 0.005], [i * step, extent, 0.005],
                lineColorRGB=color, lineWidth=width,
                physicsClientId=self.client_id,
            )
            p.addUserDebugLine(
                [-extent, i * step, 0.005], [extent, i * step, 0.005],
                lineColorRGB=color, lineWidth=width,
                physicsClientId=self.client_id,
            )

    def set_camera(
        self,
        distance: float = 7.5,
        yaw: float = 45.0,
        pitch: float = -25.0,
        target: Sequence[float] = (0.0, 0.0, 1.2),
    ) -> None:
        """Update third-person camera pose."""
        if not self.gui or self.client_id < 0:
            return
        p.resetDebugVisualizerCamera(
            cameraDistance=float(distance),
            cameraYaw=float(yaw),
            cameraPitch=float(pitch),
            cameraTargetPosition=list(target),
            physicsClientId=self.client_id,
        )

    def set_target_marker(self, target_pos: Sequence[float]) -> None:
        """Create or update a visual marker sphere at the target location."""
        if not self.gui or self.client_id < 0:
            return

        if self.target_visual_id < 0:
            visual_shape = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=0.25,
                rgbaColor=[0.1, 0.9, 0.2, 0.85],  # Translucent green target
                physicsClientId=self.client_id,
            )
            self.target_visual_id = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=visual_shape,
                basePosition=list(target_pos),
                physicsClientId=self.client_id,
            )
        else:
            p.resetBasePositionAndOrientation(
                self.target_visual_id,
                list(target_pos),
                [0, 0, 0, 1],
                physicsClientId=self.client_id,
            )

    def draw_trajectory(self, waypoints: Sequence[Sequence[float]], color: Sequence[float] = (0.2, 0.8, 1.0)) -> None:
        """Draw debug lines showing the planned waypoint route."""
        if not self.gui or self.client_id < 0:
            return

        for line_id in self.trajectory_line_ids:
            try:
                p.removeUserDebugItem(line_id, physicsClientId=self.client_id)
            except Exception:
                pass
        self.trajectory_line_ids.clear()

        for i in range(len(waypoints) - 1):
            p1 = list(waypoints[i])
            p2 = list(waypoints[i + 1])
            lid = p.addUserDebugLine(
                p1,
                p2,
                lineColorRGB=list(color),
                lineWidth=2.0,
                physicsClientId=self.client_id,
            )
            self.trajectory_line_ids.append(lid)

    def step(self, num_substeps: int = 1) -> None:
        """Advance the physics simulation by num_substeps."""
        if self.client_id < 0:
            return
        for _ in range(num_substeps):
            p.stepSimulation(physicsClientId=self.client_id)

    def close(self) -> None:
        """Disconnect and clean up physics server."""
        if self.client_id >= 0:
            try:
                p.disconnect(physicsClientId=self.client_id)
            except Exception:
                pass
            self.client_id = -1
            self.plane_id = -1
            self.target_visual_id = -1
            self.trajectory_line_ids.clear()


def run_simulation(gui: bool = True, scenario_name: str = "normal") -> None:
    """Standalone visual demonstration of the ACMFDS-driven mission.

    The previous version of this demo spawned the drone mid-air and flew it
    with a PD guidance loop; it has been replaced by the full ACMFDS
    architecture (takeoff -> hover -> cruise -> land) in
    ``simulation.acmfds_mission``.
    """
    try:
        from .acmfds_mission import run_acmfds_mission
        from ..scenarios import get_scenario
    except (ImportError, ValueError):
        from simulation.acmfds_mission import run_acmfds_mission
        from scenarios import get_scenario

    scenario = get_scenario(scenario_name)
    report = run_acmfds_mission(scenario, gui=gui, realtime=True)
    print(f"\nSimulation finished: {report.outcome} ({report.termination_reason})")


if __name__ == "__main__":
    run_simulation(gui=True)
