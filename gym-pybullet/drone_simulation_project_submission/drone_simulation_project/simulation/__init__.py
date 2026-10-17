"""PyBullet simulation components.

Imports are lazy (PEP 562) so that pure-Python modules such as
``simulation.mission``, ``simulation.battery`` and ``simulation.wind`` can be
used without pybullet installed.
"""

__all__ = [
    "ObstacleManager",
    "ObstacleConfig",
    "DroneModel",
    "PyBulletSimulation",
    "BatteryModel",
    "BatteryConfig",
    "WindModel",
    "WindConfig",
    "MissionSequencer",
    "MissionPhase",
    "MissionConfig",
]


def __getattr__(name):
    if name in ("ObstacleManager", "ObstacleConfig"):
        from .obstacles import ObstacleManager, ObstacleConfig

        return {"ObstacleManager": ObstacleManager, "ObstacleConfig": ObstacleConfig}[name]
    if name == "DroneModel":
        from .drone import DroneModel

        return DroneModel
    if name == "PyBulletSimulation":
        from .pybullet_sim import PyBulletSimulation

        return PyBulletSimulation
    if name in ("BatteryModel", "BatteryConfig"):
        from .battery import BatteryModel, BatteryConfig

        return {"BatteryModel": BatteryModel, "BatteryConfig": BatteryConfig}[name]
    if name in ("WindModel", "WindConfig"):
        from .wind import WindModel, WindConfig

        return {"WindModel": WindModel, "WindConfig": WindConfig}[name]
    if name in ("MissionSequencer", "MissionPhase", "MissionConfig"):
        from .mission import MissionSequencer, MissionPhase, MissionConfig

        return {
            "MissionSequencer": MissionSequencer,
            "MissionPhase": MissionPhase,
            "MissionConfig": MissionConfig,
        }[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
