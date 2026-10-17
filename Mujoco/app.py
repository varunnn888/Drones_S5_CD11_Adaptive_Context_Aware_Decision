"""Launch the ACMFDS autonomous quadrotor dashboard."""
from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from dashboard import Dashboard
from simulation import QuadrotorSimulation


def main() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    simulation = QuadrotorSimulation()
    window = Dashboard(simulation)
    window.showMaximized()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
