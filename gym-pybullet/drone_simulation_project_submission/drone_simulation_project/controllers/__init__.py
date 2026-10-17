"""Controllers package.

- ActuationLayer: sliding-mode (non-PID) command-to-actuation conversion used
  by the ACMFDS mission pipeline.
- DataDrivenAltitudeController / DataDrivenController: optional learned
  feedforward thrust predictor (trained on PID-free inverse dynamics).
"""

from .actuation import ActuationLayer, ActuationConfig
from .data_driven_controller import (
    DataDrivenAltitudeController,
    DataDrivenController,
)

__all__ = [
    "ActuationLayer",
    "ActuationConfig",
    "DataDrivenAltitudeController",
    "DataDrivenController",
]
