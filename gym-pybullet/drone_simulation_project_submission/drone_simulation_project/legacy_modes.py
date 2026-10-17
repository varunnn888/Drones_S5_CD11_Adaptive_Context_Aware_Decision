"""Legacy optional execution modes (clearly labelled, non-default).

Manual direct control is kept as a demonstration aid only. It bypasses
ACMFDS entirely and is not part of the autonomous architecture.
"""

import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import pybullet as p

from simulation.pybullet_sim import PyBulletSimulation
from simulation.drone import DroneModel
from simulation.obstacles import ObstacleManager


def run_mode_1_manual(gui: bool = True) -> None:
    """Manual/Direct Control via keyboard inputs in PyBullet GUI."""
    print("\n" + "=" * 60)
    print("LEGACY MODE 1: MANUAL DIRECT CONTROL (no ACMFDS)")
    print("=" * 60)
    print("Controls:")
    print("  Up/Down Arrow : Pitch forward / backward")
    print("  Left/Right    : Roll left / right")
    print("  W / S         : Increase / decrease thrust")
    print("  A / D         : Yaw left / right")
    print("  Space         : Hover thrust")
    print("  Q             : Quit")
    print("=" * 60 + "\n")

    sim = PyBulletSimulation(gui=gui, time_step=1.0 / 120.0)
    sim.connect()

    drone = DroneModel(physics_client_id=sim.client_id, enable_visual_parts=gui)
    drone.spawn(position=(-4.0, 0.0, 0.12))
    obstacles = ObstacleManager(physics_client_id=sim.client_id, randomize=False)
    obstacles.setup_obstacles()

    thrust_cmd, roll_cmd, pitch_cmd, yaw_cmd = 0.0, 0.0, 0.0, 0.0
    dt = sim.time_step

    try:
        while p.isConnected(physicsClientId=sim.client_id):
            keys = p.getKeyboardEvents(physicsClientId=sim.client_id)
            if ord("q") in keys and keys[ord("q")] & p.KEY_WAS_TRIGGERED:
                break
            if ord("w") in keys:
                thrust_cmd = min(1.0, thrust_cmd + 0.02)
            elif ord("s") in keys:
                thrust_cmd = max(-1.0, thrust_cmd - 0.02)
            elif ord(" ") in keys:
                thrust_cmd = 0.0
            pitch_cmd = 0.4 if p.B3G_UP_ARROW in keys else (-0.4 if p.B3G_DOWN_ARROW in keys else 0.0)
            roll_cmd = -0.4 if p.B3G_LEFT_ARROW in keys else (0.4 if p.B3G_RIGHT_ARROW in keys else 0.0)
            yaw_cmd = -0.4 if ord("a") in keys else (0.4 if ord("d") in keys else 0.0)

            drone.apply_action([thrust_cmd, roll_cmd, pitch_cmd, yaw_cmd])
            sim.step()
            if gui:
                drone.update_visual_parts()
                sim.set_camera(distance=6.0, yaw=45, pitch=-25, target=drone.get_position())
                time.sleep(dt * 0.8)
    except KeyboardInterrupt:
        pass
    finally:
        sim.close()
        print("Manual mode finished.")
