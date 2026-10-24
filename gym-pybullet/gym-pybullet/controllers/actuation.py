"""Actuation layer: converts ACMFDS flight commands into low-level commands.

Architecture position::

    ACMFDS Decision -> FlightCommand -> ActuationLayer -> normalized action
        -> DroneModel.apply_action -> PyBullet forces/torques

Control method — sliding-mode control (SMC), a nonlinear robust control law:

* Altitude channel: first-order sliding surface s = ė + λe on altitude
  error with boundary-layer saturation (no switching chatter).
* Horizontal channel: velocity error -> desired tilt direction/magnitude
  (small-angle thrust vectoring), then attitude SMC tracks the reference
  tilt on roll and pitch.
* Yaw channel: attitude SMC on the wrapped heading error.

This layer deliberately contains **no PID structure and no integral term**.
SMC was chosen because it is a published, genuinely non-PID nonlinear
control method that is robust to the model uncertainty of a 1 kg
quadcopter (mass error, drag, gusts) without hiding feedback gains inside a
neural network. Remaining limitation, stated honestly: like every feedback
controller, SMC closes a loop around state error — the project's claim is
"no PID controller", not "no feedback".

Output convention matches ``DroneModel.apply_action``:
    [thrust_norm, roll_cmd, pitch_cmd, yaw_cmd], all in [-1, 1]
    thrust_norm = 0 -> hover thrust; ±1 -> max/min thrust.
"""

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from state.flight_state import FlightState
from decision.acmfds import FlightCommand
from simulation.wind import AIR_DENSITY, DRAG_COEFFICIENT, EXPOSED_AREA


@dataclass
class ActuationConfig:
    """Gains and physical limits for the SMC actuation layer (SI units)."""

    mass: float = 1.0
    gravity: float = 9.81
    min_thrust: float = 0.0
    max_thrust: float = 20.0
    max_torque: float = 1.5
    # Aerodynamic drag coefficient (N·s/m) — synced from DroneModel at setup.
    # Model-based feedforward: without it a proportional velocity law settles
    # at v = v_cmd * k / (k + c) (measured droop 1.2 -> 1.03 m/s).
    linear_drag: float = 0.25
    # Rigid-body inertia about body X, Y, Z (kg m^2). Default estimated
    # from the collision box (0.5 x 0.5 x 0.16 m, 1 kg); the runner may
    # override with p.getDynamicsInfo values.
    inertia: Tuple[float, float, float] = (0.023, 0.023, 0.042)

    # Altitude SMC
    lambda_z: float = 2.2       # sliding surface slope, 1/s
    k_z: float = 3.0            # reaching gain, m/s^2
    phi_z: float = 0.25         # boundary layer, m/s
    thrust_slew_n_per_s: float = 90.0

    # Velocity -> tilt mapping. Acceleration demand = tilt_gain * velocity
    # error plus a wind feedforward trim (inverse statics, no integrator);
    # tilt reference = atan(|a_des| / g), capped. tilt_gain is sized so the
    # avoidance/emergency braking demand actually reaches the tilt cap
    # (a = g * tan(max_tilt) ~ 3.7 m/s^2) and can stop cruise speed within
    # the detection-to-critical clearance gap.
    tilt_gain: float = 3.2      # 1/s on velocity error
    max_tilt: float = 0.36      # rad reference cap (~21 deg)

    # Landing protection: below this altitude while descending to a near-zero
    # target, horizontal motion targets are zeroed and the drone is leveled,
    # so touchdown happens vertically (no scoping, no tip-over).
    landing_cutoff_altitude: float = 0.45

    # Attitude SMC (roll / pitch)
    lambda_att: float = 6.0     # 1/s
    k_att: float = 40.0         # rad/s^2
    phi_att: float = 0.30       # rad/s boundary layer

    # Yaw SMC
    k_yaw: float = 25.0         # rad/s^2
    phi_yaw: float = 0.20       # rad/s boundary layer


def _sat(x: float) -> float:
    """Unit saturation (signum with boundary layer applied by caller)."""
    return max(-1.0, min(1.0, x))


class ActuationLayer:
    """SMC-based command-to-action conversion."""

    def __init__(self, config: Optional[ActuationConfig] = None) -> None:
        self.config = config or ActuationConfig()
        self._prev_thrust: float = self.hover_thrust

    @property
    def hover_thrust(self) -> float:
        return self.config.mass * self.config.gravity

    def reset(self) -> None:
        self._prev_thrust = self.hover_thrust

    # ------------------------------------------------------------------
    def compute_action(
        self, state: FlightState, command: FlightCommand, dt: float
    ) -> np.ndarray:
        """Produce a normalized action for ``DroneModel.apply_action``.

        Args:
            state: validated flight state (attitude in radians)
            command: ACMFDS flight command
            dt: control timestep, seconds

        Returns:
            np.ndarray [thrust_norm, roll_cmd, pitch_cmd, yaw_cmd] in [-1, 1]
        """
        cfg = self.config
        thrust = self._altitude_control(state, command, dt)
        roll_cmd, pitch_cmd = self._horizontal_control(state, command)
        yaw_cmd = self._yaw_control(state, command)
        return np.array(
            [
                self._normalize_thrust(thrust),
                float(np.clip(roll_cmd, -1.0, 1.0)),
                float(np.clip(pitch_cmd, -1.0, 1.0)),
                float(np.clip(yaw_cmd, -1.0, 1.0)),
            ],
            dtype=np.float32,
        )

    # ------------------------------------------------------------------
    def _altitude_control(self, state: FlightState, command: FlightCommand, dt: float) -> float:
        cfg = self.config
        e = command.target_altitude - state.altitude       # m
        e_dot = -state.velocity_z                          # m/s
        s = e_dot + cfg.lambda_z * e                       # sliding surface
        accel = cfg.lambda_z * e_dot + cfg.k_z * _sat(s / cfg.phi_z)
        thrust = cfg.mass * (cfg.gravity + accel)
        # Slew-rate limit avoids step discontinuities in the applied force.
        max_delta = cfg.thrust_slew_n_per_s * max(dt, 1e-4)
        thrust = min(thrust, self._prev_thrust + max_delta)
        thrust = max(thrust, self._prev_thrust - max_delta)
        thrust = float(np.clip(thrust, cfg.min_thrust, cfg.max_thrust))
        self._prev_thrust = thrust
        return thrust

    def _horizontal_control(self, state: FlightState, command: FlightCommand) -> Tuple[float, float]:
        cfg = self.config
        # Landing protection: level the drone and demand zero horizontal
        # velocity during the final descent — touchdown must be vertical.
        if (
            state.altitude <= cfg.landing_cutoff_altitude
            and command.target_altitude <= cfg.landing_cutoff_altitude
            and state.velocity_z < 0.05
        ):
            return (
                self._attitude_smc(0, 0.0, state.roll, state.roll_rate),
                self._attitude_smc(1, 0.0, state.pitch, state.pitch_rate),
            )

        # Desired horizontal velocity in the world frame (body command rotated by yaw).
        psi = state.yaw
        fwd = min(command.forward_speed, command.speed_limit)
        lat = min(command.lateral_speed, command.speed_limit)
        v_des_x = math.cos(psi) * fwd - math.sin(psi) * lat
        v_des_y = math.sin(psi) * fwd + math.cos(psi) * lat

        # Acceleration demand = proportional velocity error + model-based
        # feedforward (drag compensation, wind trim). Both are instantaneous
        # inverse-statics terms — no integral, no PID structure.
        a_des_x = cfg.tilt_gain * (v_des_x - state.velocity_x) + cfg.linear_drag * v_des_x
        a_des_y = cfg.tilt_gain * (v_des_y - state.velocity_y) + cfg.linear_drag * v_des_y
        if state.wind_speed > 1e-6:
            wind_v = np.asarray(state.wind_vector, dtype=np.float64)
            drone_v = np.array([state.velocity_x, state.velocity_y, state.velocity_z])
            v_rel = wind_v - drone_v
            speed = float(np.linalg.norm(v_rel))
            if speed > 1e-6:
                f_wind = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * EXPOSED_AREA * speed * speed
                a_wind = f_wind / cfg.mass
                a_des_x -= a_wind * float(v_rel[0] / speed)
                a_des_y -= a_wind * float(v_rel[1] / speed)

        a_mag = math.hypot(a_des_x, a_des_y)
        if a_mag < 1e-6:
            return 0.0, 0.0

        alpha = min(math.atan(a_mag / cfg.gravity), cfg.max_tilt)
        beta = math.atan2(a_des_y, a_des_x)                  # world direction
        rel = beta - psi
        # Small-angle thrust vectoring: world tilt (θ, -φ) = α·(cos rel, sin rel)
        pitch_des = alpha * math.cos(rel)
        roll_des = -alpha * math.sin(rel)

        roll_cmd = self._attitude_smc(0, roll_des, state.roll, state.roll_rate)
        pitch_cmd = self._attitude_smc(1, pitch_des, state.pitch, state.pitch_rate)
        return roll_cmd, pitch_cmd

    def _yaw_control(self, state: FlightState, command: FlightCommand) -> float:
        cfg = self.config
        if command.heading is None:
            return 0.0
        err = command.heading - state.yaw
        err = math.atan2(math.sin(err), math.cos(err))  # wrap to [-π, π]
        s = -state.yaw_rate + 3.0 * err                 # sliding surface
        torque = cfg.inertia[2] * (3.0 * (-state.yaw_rate) + cfg.k_yaw * _sat(s / cfg.phi_yaw))
        return float(np.clip(torque / cfg.max_torque, -1.0, 1.0))

    def _attitude_smc(
        self, axis: int, angle_des: float, angle: float, rate: float, rate_des: float = 0.0
    ) -> float:
        """First-order SMC on angle error; returns normalized torque."""
        cfg = self.config
        e = angle_des - angle
        e_dot = rate_des - rate
        s = e_dot + cfg.lambda_att * e
        torque = cfg.inertia[axis] * (cfg.lambda_att * e_dot + cfg.k_att * _sat(s / cfg.phi_att))
        return float(np.clip(torque / cfg.max_torque, -1.0, 1.0))

    # ------------------------------------------------------------------
    def _normalize_thrust(self, thrust_n: float) -> float:
        cfg = self.config
        hover = self.hover_thrust
        if thrust_n >= hover:
            norm = (thrust_n - hover) / (cfg.max_thrust - hover)
        else:
            norm = (thrust_n - hover) / (hover - cfg.min_thrust)
        return float(np.clip(norm, -1.0, 1.0))
