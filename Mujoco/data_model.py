"""Lightweight data-driven risk estimation and adaptive objective weights."""
from __future__ import annotations

import numpy as np
from sklearn.ensemble import RandomForestRegressor


class DataDrivenRiskModel:
    """Demo model trained on reproducible synthetic risk examples.

    For research use, replace the synthetic training data with labelled flight
    logs or a validated obstacle-risk dataset.
    """

    def __init__(self, seed: int = 42) -> None:
        rng = np.random.default_rng(seed)
        n = 2500
        nearest = rng.uniform(0.15, 5.0, n)
        speed = rng.uniform(0.0, 2.6, n)
        battery = rng.uniform(15.0, 100.0, n)
        tracking = rng.uniform(0.0, 1.0, n)
        features = np.column_stack((nearest, speed, battery, tracking))
        target = (
            0.62 * np.exp(-nearest / 1.15)
            + 0.16 * np.clip(speed / 2.6, 0.0, 1.0)
            + 0.12 * (1.0 - battery / 100.0)
            + 0.10 * tracking
        )
        labels = np.clip(target + rng.normal(0.0, 0.025, n), 0.0, 1.0)
        self.model = RandomForestRegressor(
            n_estimators=80,
            max_depth=8,
            random_state=seed,
            n_jobs=-1,
        )
        self.model.fit(features, labels)

    def predict(
        self,
        nearest_obstacle: float,
        speed: float,
        battery: float,
        tracking_error: float,
    ) -> float:
        features = np.array([[nearest_obstacle, speed, battery, tracking_error]], dtype=float)
        return float(np.clip(self.model.predict(features)[0], 0.0, 1.0))


def adaptive_weights(
    risk: float,
    battery: float,
    distance: float,
    speed: float,
    remaining_time: float,
) -> dict[str, float]:
    """Return normalized Safety/Energy/Time/Tracking/Mission weights."""
    raw = np.array(
        [
            0.18 + 0.48 * np.clip(risk, 0.0, 1.0),
            0.12 + 0.30 * (1.0 - np.clip(battery / 100.0, 0.0, 1.0)),
            0.07 + 0.18 * np.clip(distance / 18.0, 0.0, 1.0),
            0.14 + 0.18 * np.clip(speed / 2.4, 0.0, 1.0),
            0.14 + 0.22 * np.clip(remaining_time / 150.0, 0.0, 1.0),
        ],
        dtype=float,
    )
    names = ("safety", "energy", "time", "tracking", "mission")
    raw /= max(float(raw.sum()), 1e-9)
    return {name: float(value) for name, value in zip(names, raw)}
