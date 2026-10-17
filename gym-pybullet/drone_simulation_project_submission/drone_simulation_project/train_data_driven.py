"""Train the optional data-driven feedforward thrust predictor.

PID-audit note: an earlier revision generated training targets with a PID
law (tracking/damping/integral gains). That violated the project's no-PID
requirement and has been removed.

Data generation now uses pure inverse dynamics:
    1. Sample minimum-jerk altitude reference profiles z_ref(t) between
       random start/end altitudes (position, velocity, and acceleration are
       analytic — no feedback controller is involved in the targets).
    2. Target thrust: T*(t) = m (g + z̈_ref) + c · ż_ref  (feedforward only).
    3. The 1-D plant is flown with the project's sliding-mode actuation
       layer so feature distributions match real closed-loop flight.

The result is a learned inverse-dynamics model whose runtime path and
training targets contain no PID structure.
"""

import os
import sys
import argparse
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from controllers.data_driven_controller import DataDrivenAltitudeController
from controllers.actuation import ActuationLayer
from decision.acmfds import FlightCommand
from state.flight_state import FlightState


def minimum_jerk(t: float, T: float, z0: float, z1: float) -> tuple[float, float, float]:
    """Position, velocity, acceleration of a minimum-jerk profile at time t."""
    if t >= T:
        return z1, 0.0, 0.0
    s = max(0.0, t / T)
    ds = 1.0 / T
    pos = z0 + (z1 - z0) * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5)
    vel = (z1 - z0) * (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) * ds
    acc = (z1 - z0) * (60 * s - 180 * s ** 2 + 120 * s ** 3) * ds * ds
    return pos, vel, acc


def generate_flight_training_data(
    num_episodes: int = 250,
    steps_per_episode: int = 480,
    dt: float = 1.0 / 120.0,
    seed: int = 42,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate state/thrust samples under 1-D physics.

    Targets are feedforward inverse dynamics of minimum-jerk references —
    no PID or feedback gains appear anywhere in the data.
    """
    rng = np.random.RandomState(seed)
    mass = 1.0
    gravity = 9.81
    linear_drag = 0.25

    X_list = []
    y_list = []

    for _ in range(num_episodes):
        # Chain 1-3 minimum-jerk segments per episode. Chained segments create
        # reference steps, so the feature distribution covers realistic
        # transient tracking errors (not just near-zero errors), while targets
        # remain pure feedforward inverse dynamics throughout.
        segments: list[tuple[float, float]] = []
        z_start = float(rng.uniform(0.3, 3.5))
        z_prev = z_start
        for _s in range(int(rng.randint(1, 4))):
            z_next = float(np.clip(z_prev + rng.uniform(-2.2, 2.2), 0.3, 4.5))
            dur = float(rng.uniform(1.2, 4.0))
            segments.append((dur, z_next))
            z_prev = z_next
        # Trailing hold: the reference freezes at the final altitude while the
        # plant settles (vz decays, error/integral still nonzero). This corner
        # — small error + lingering integral + near-zero velocity — is exactly
        # where an undertrained feedforward model overpredicts climb thrust.
        segments.append((float(rng.uniform(0.8, 2.0)), z_prev))

        actuation = ActuationLayer()
        collector = DataDrivenAltitudeController()  # for feature extraction only
        z, vz = z_start, 0.0

        for step in range(steps_per_episode):
            t = step * dt
            # Locate the active segment.
            t_acc = t
            z_seg0 = z_start
            z_ref, vz_ref, az_ref = z, 0.0, 0.0
            for dur, z_end in segments:
                if t_acc < dur:
                    z_ref, vz_ref, az_ref = minimum_jerk(t_acc, dur, z_seg0, z_end)
                    break
                t_acc -= dur
                z_seg0 = z_end
            else:
                z_ref, vz_ref, az_ref = segments[-1][1], 0.0, 0.0

            # Plant control (SMC, non-PID) — shapes realistic feature states.
            st = FlightState(altitude=z, velocity_z=vz, target_altitude=z_ref)
            actuation.compute_action(st, FlightCommand(target_altitude=z_ref), dt)
            thrust = actuation._prev_thrust

            # Feature vector (inputs only) — includes reference acceleration
            # so the feedforward target is identifiable.
            feats = collector.extract_features(z_ref, z, dt, vz, target_accel=az_ref)

            # Feedforward target thrust from inverse dynamics (no feedback).
            target_thrust = mass * (gravity + az_ref) + linear_drag * vz_ref
            noise = rng.normal(0.0, 0.05)
            target_thrust = float(np.clip(target_thrust + noise, 0.0, 20.0))

            X_list.append(feats)
            y_list.append(target_thrust)

            # Advance 1-D physics.
            az = (thrust - mass * gravity - linear_drag * vz) / mass
            vz += az * dt
            z += vz * dt
            if z < 0.05:
                z = 0.05
                vz = max(0.0, vz)

    X = np.array(X_list, dtype=np.float32)
    y = np.array(y_list, dtype=np.float32)
    return X, y


def train_data_driven_controller(
    output_path: str = "models/data_driven_controller.pkl",
    model_type: str = "mlp",
    num_episodes: int = 250,
) -> DataDrivenAltitudeController:
    """Train, evaluate, and save the feedforward thrust predictor."""
    print("=" * 60)
    print("TRAINING DATA-DRIVEN FEEDFORWARD THRUST PREDICTOR")
    print("=" * 60)
    print(f"Model architecture   : {model_type.upper()}")
    print(f"Target law           : inverse dynamics (minimum-jerk refs, no PID)")
    print(f"Generating samples   : {num_episodes} trajectories...")

    X, y = generate_flight_training_data(num_episodes=num_episodes)
    print(f"Dataset generated    : {X.shape[0]:,} samples across {X.shape[1]} features.")

    controller = DataDrivenAltitudeController()
    print("Fitting model to inverse-dynamics dataset...")
    metrics = controller.train_on_data(X, y, model_type=model_type)

    print("\nValidation Metrics:")
    print(f"  MSE  : {metrics['val_mse']:.4f}")
    print(f"  MAE  : {metrics['val_mae']:.4f} N")
    print(f"  R^2  : {metrics['val_r2']:.4f}")

    full_output_path = os.path.join(PROJECT_ROOT, output_path)
    controller.save(full_output_path)
    print(f"Model saved to       : {full_output_path}")
    print("=" * 60 + "\n")
    return controller


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train data-driven feedforward thrust predictor")
    parser.add_argument("--output", type=str, default="models/data_driven_controller.pkl")
    parser.add_argument("--model-type", type=str, choices=["mlp", "gradient_boost", "ridge"], default="mlp")
    parser.add_argument("--episodes", type=int, default=250)
    args = parser.parse_args()

    train_data_driven_controller(
        output_path=args.output,
        model_type=args.model_type,
        num_episodes=args.episodes,
    )
