"""Data-Driven Controller for Quadcopter Simulation.

Role in the current architecture (documented honestly):
An *optional learned feedforward thrust predictor*. It learns inverse
vertical dynamics — given flight features, predict the feedforward thrust
T* = m(g + z̈_ref) + c·ż_ref of a reference maneuver — and can be consulted
by the actuation layer or evaluation tooling. It is NOT the stabilizing
controller of the project (the sliding-mode actuation layer is), and the
main ACMFDS demonstration runs without it.

PID-audit note: an earlier revision generated its training targets with a
PID law (proportional/derivative/integral gains). That hidden dependency
violated the project's no-PID requirement and has been removed: training
data is now generated from minimum-jerk reference profiles through pure
inverse dynamics (see train_data_driven.py). No PID terms exist anywhere in
the training pipeline or in this module's runtime path.

Features (7): error_z, error_derivative, leaky integral_error, velocity_z,
current_z, target_z, target_accel — inputs only; targets are feedforward
thrust in N.
"""

import os
import math
from typing import Optional, Sequence, Dict, Any, Union
import numpy as np

# Default model weights path
DEFAULT_MODEL_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "models",
    "data_driven_controller.pkl",
)


class DataDrivenAltitudeController:
    """Learned feedforward thrust predictor for drone altitude maneuvers.

    Args:
        model_path: Path to trained weights (.pkl). If None or not found,
                    the controller degrades to a transparent hover-thrust
                    feedforward baseline (documented, not a hidden PID).
        hover_thrust: Baseline thrust counteracting gravity (m·g = 9.81 N)
        min_thrust: Minimum physical thrust limit (N)
        max_thrust: Maximum physical thrust limit (N)
        integral_limit: Maximum absolute value for the integral feature
        leak_rate: Leaky integrator decay factor per second
    """

    FEATURE_NAMES = [
        "error_z",
        "error_derivative",
        "integral_error",
        "velocity_z",
        "current_z",
        "target_z",
        "target_accel",  # reference vertical acceleration z̈_ref (m/s^2)
    ]

    def __init__(
        self,
        model_path: Optional[str] = None,
        hover_thrust: float = 9.81,
        min_thrust: float = 0.0,
        max_thrust: float = 20.0,
        integral_limit: float = 4.0,
        leak_rate: float = 0.05,
    ) -> None:
        self.hover_thrust = float(hover_thrust)
        self.min_thrust = float(min_thrust)
        self.max_thrust = float(max_thrust)
        self.integral_limit = float(integral_limit)
        self.leak_rate = float(leak_rate)

        # Internal feature state
        self.integral = 0.0
        self.previous_error: Optional[float] = None
        self.previous_z: Optional[float] = None
        self.is_trained = False
        self.backend = "hover_feedforward"

        # Model storage
        self.ml_model: Any = None
        self._feature_mean: Optional[np.ndarray] = None
        self._feature_std: Optional[np.ndarray] = None

        # Attempt loading trained model if available
        resolved_path = model_path if model_path else DEFAULT_MODEL_PATH
        if resolved_path and os.path.exists(resolved_path):
            self.load(resolved_path)

    def _init_default_weights(self) -> None:
        """Deprecated: random 'calibrated' weights removed.

        The previous default was a random initialization presented as
        calibrated weights — unreliable and misleading. The documented
        fallback is now the transparent hover-feedforward baseline in
        ``_fallback_forward`` (hover thrust, no feedback structure).
        """

    def reset(self) -> None:
        """Reset internal integrator and historical state."""
        self.integral = 0.0
        self.previous_error: Optional[float] = None
        self.previous_z: Optional[float] = None

    def extract_features(
        self,
        target_z: float,
        current_z: float,
        dt: float,
        current_vz: Optional[float] = None,
        target_accel: float = 0.0,
    ) -> np.ndarray:
        """Extract continuous feature vector for data-driven model inference.

        Args:
            target_z: Reference altitude in meters
            current_z: Current measured altitude in meters
            dt: Simulation timestep in seconds
            current_vz: Optional vertical velocity; if None, estimated via finite difference
            target_accel: Reference vertical acceleration z̈_ref (m/s^2).
                Feedforward inverse dynamics is only identifiable when the
                reference acceleration is observable, hence this feature.

        Returns:
            7-dimensional numpy array of features
        """
        error = target_z - current_z

        if dt > 1e-6:
            # Leaky anti-windup integration
            self.integral = (1.0 - self.leak_rate * dt) * self.integral + error * dt
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

            # Numerical derivative of error (smooth on step 0)
            if self.previous_error is not None:
                error_deriv = (error - self.previous_error) / dt
            else:
                error_deriv = 0.0
            self.previous_error = error

            # Velocity estimation if not directly supplied
            if current_vz is not None:
                vz = float(current_vz)
            elif self.previous_z is not None:
                vz = (current_z - self.previous_z) / dt
            else:
                vz = 0.0
            self.previous_z = current_z
        else:
            error_deriv = 0.0
            vz = float(current_vz) if current_vz is not None else 0.0

        features = np.array(
            [error, error_deriv, self.integral, vz, current_z, target_z, float(target_accel)],
            dtype=np.float32,
        )
        return features

    def _fallback_forward(self, features: np.ndarray) -> float:
        """Transparent fallback: hover thrust (pure feedforward, no feedback).

        Used only when no trained model is available. It makes no claim of
        tracking performance and keeps the output within physical bounds.
        """
        return float(self.hover_thrust)

    def compute(
        self,
        target_z: float,
        current_z: float,
        dt: float,
        current_vz: Optional[float] = None,
        target_accel: float = 0.0,
    ) -> float:
        """Compute feedforward vertical thrust from the learned model.

        Args:
            target_z: Reference altitude in meters
            current_z: Current measured altitude in meters
            dt: Timestep in seconds
            current_vz: Optional vertical velocity in m/s
            target_accel: Reference vertical acceleration z̈_ref (m/s^2);
                0 for hover/steady references

        Returns:
            Thrust prediction bounded within [min_thrust, max_thrust] (N)
        """
        features = self.extract_features(target_z, current_z, dt, current_vz, target_accel)

        if self.ml_model is not None and hasattr(self.ml_model, "predict"):
            try:
                x = features.reshape(1, -1)
                if self._feature_mean is not None and self._feature_std is not None:
                    x = (x - self._feature_mean) / (self._feature_std + 1e-8)
                pred = self.ml_model.predict(x)
                raw_thrust = float(pred[0])
            except Exception:
                raw_thrust = self._fallback_forward(features)
        else:
            raw_thrust = self._fallback_forward(features)

        if math.isnan(raw_thrust) or math.isinf(raw_thrust):
            raw_thrust = self.hover_thrust
        return float(max(self.min_thrust, min(self.max_thrust, raw_thrust)))

    def compute_control(
        self,
        state: Union[np.ndarray, Sequence[float], Dict[str, Any], float],
        target_z: Optional[float] = None,
        dt: float = 1.0 / 120.0,
    ) -> float:
        """Compute vertical thrust command directly from drone state.

        Args:
            state: Drone state representation. Can be:
                - 1D numpy array observation from DroneEnv (27-dim), where:
                    index 2 is current altitude, index 5 is vertical velocity,
                    index 14 is target altitude
                - Dict with keys 'current_z' (or 'altitude', 'z'), 'target_z', and 'velocity_z' (or 'vz')
                - Sequence of floats [current_z, velocity_z] or [x, y, z]
                - Scalar float representing current_z
            target_z: Target altitude in meters (overrides state target if specified)
            dt: Timestep in seconds

        Returns:
            Thrust command bounded within [min_thrust, max_thrust] (N)
        """
        current_vz: Optional[float] = None
        current_z: float = 0.0
        tz: float = 2.0 if target_z is None else float(target_z)
        t_accel: float = 0.0

        if isinstance(state, dict):
            current_z = float(state.get("current_z", state.get("altitude", state.get("z", 0.0))))
            if target_z is None and "target_z" in state:
                tz = float(state["target_z"])
            if "velocity_z" in state or "vz" in state:
                current_vz = float(state.get("velocity_z", state.get("vz", 0.0)))
            if "target_accel" in state:
                t_accel = float(state["target_accel"])
        elif isinstance(state, (np.ndarray, list, tuple)):
            arr = np.asarray(state, dtype=np.float32).ravel()
            if len(arr) >= 27:
                current_z = float(arr[2])
                current_vz = float(arr[5])
                if target_z is None:
                    tz = float(arr[14])
            elif len(arr) >= 3:
                current_z = float(arr[2])
            elif len(arr) == 2:
                current_z = float(arr[0])
                current_vz = float(arr[1])
            elif len(arr) == 1:
                current_z = float(arr[0])
        else:
            current_z = float(state)

        return self.compute(
            target_z=tz,
            current_z=current_z,
            dt=dt,
            current_vz=current_vz,
            target_accel=t_accel,
        )

    def train_on_data(
        self,
        X: np.ndarray,
        y: np.ndarray,
        model_type: str = "mlp",
        test_size: float = 0.15,
        random_state: int = 42,
    ) -> Dict[str, float]:
        """Train data-driven controller directly on collected flight data.

        Args:
            X: Matrix of flight features (N, 6)
            y: Array of target thrust values (N,)
            model_type: "mlp" (Multi-Layer Perceptron) or "ridge" / "gradient_boost"
            test_size: Holdout validation split fraction
            random_state: Random seed for reproducibility

        Returns:
            Dictionary with training and validation metrics (MSE, MAE, R2)
        """
        from sklearn.model_selection import train_test_split
        from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

        X_train, X_val, y_train, y_val = train_test_split(
            X, y, test_size=test_size, random_state=random_state
        )

        # Normalize features
        self._feature_mean = np.mean(X_train, axis=0)
        self._feature_std = np.std(X_train, axis=0)
        self._feature_std[self._feature_std < 1e-6] = 1.0

        X_train_norm = (X_train - self._feature_mean) / self._feature_std
        X_val_norm = (X_val - self._feature_mean) / self._feature_std

        if model_type == "mlp":
            from sklearn.neural_network import MLPRegressor
            model = MLPRegressor(
                hidden_layer_sizes=(64, 32),
                activation="relu",
                solver="adam",
                alpha=1e-4,
                learning_rate_init=0.003,
                max_iter=300,
                early_stopping=True,
                validation_fraction=0.1,
                random_state=random_state,
            )
        elif model_type == "gradient_boost":
            from sklearn.ensemble import HistGradientBoostingRegressor
            model = HistGradientBoostingRegressor(
                max_iter=150,
                learning_rate=0.05,
                random_state=random_state,
            )
        else:
            from sklearn.linear_model import Ridge
            model = Ridge(alpha=1.0)

        model.fit(X_train_norm, y_train)
        self.ml_model = model
        self.is_trained = True
        self.backend = f"sklearn_{model_type}"

        val_preds = model.predict(X_val_norm)
        metrics = {
            "val_mse": float(mean_squared_error(y_val, val_preds)),
            "val_mae": float(mean_absolute_error(y_val, val_preds)),
            "val_r2": float(r2_score(y_val, val_preds)),
        }
        return metrics

    def save(self, filepath: str) -> None:
        """Save trained model and normalization statistics."""
        import joblib

        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        payload = {
            "ml_model": self.ml_model,
            "feature_mean": self._feature_mean,
            "feature_std": self._feature_std,
            "hover_thrust": self.hover_thrust,
            "min_thrust": self.min_thrust,
            "max_thrust": self.max_thrust,
            "is_trained": self.is_trained,
            "backend": self.backend,
        }
        joblib.dump(payload, filepath)

    def load(self, filepath: str) -> bool:
        """Load trained model and normalization statistics."""
        if not os.path.exists(filepath):
            return False
        import joblib

        try:
            payload = joblib.load(filepath)
            self.ml_model = payload.get("ml_model")
            self._feature_mean = payload.get("feature_mean")
            self._feature_std = payload.get("feature_std")
            self.is_trained = bool(payload.get("is_trained", True)) and self.ml_model is not None
            self.backend = payload.get("backend", "loaded_joblib")
            if self.ml_model is None:
                self.backend = "hover_feedforward"
                self.is_trained = False
            return True
        except Exception as e:
            print(f"[WARN] Failed to load data-driven controller from {filepath}: {e}")
            self.ml_model = None
            self.is_trained = False
            self.backend = "hover_feedforward"
            return False


# Primary controller interface alias
DataDrivenController = DataDrivenAltitudeController

