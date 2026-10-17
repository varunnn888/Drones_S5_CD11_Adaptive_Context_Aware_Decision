"""Reference-inspired dark ACMFDS dashboard built with PySide6 and MuJoCo."""
from __future__ import annotations

import math
import sys

import mujoco
import numpy as np
from PySide6.QtCore import Qt, QRect, QTimer
from PySide6.QtGui import QColor, QBrush, QFont, QImage, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from config import START
from planner import MAXX, MAXY, MINX, MINY


COLORS = {
    "background": "#020d17",
    "panel": "#041826",
    "panel_inner": "#061b2a",
    "border": "#08729b",
    "border_soft": "#153d54",
    "cyan": "#32caff",
    "blue": "#2497ff",
    "green": "#15ed83",
    "orange": "#ff9e32",
    "red": "#ff494e",
    "text": "#d9efff",
    "muted": "#8db7cc",
    "grid": "#18394d",
}


class Panel(QFrame):
    """Small two-column metric card with optional inline progress bars."""

    def __init__(self, title: str, rows: list[tuple[str, str]], progress_keys: set[str] | None = None):
        super().__init__()
        self.setObjectName("panel")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
        self.values: dict[str, QLabel] = {}
        self.bars: dict[str, QProgressBar] = {}
        progress_keys = progress_keys or set()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 5)
        layout.setSpacing(2)

        title_label = QLabel(title.upper())
        title_label.setObjectName("panelTitle")
        title_label.setMinimumHeight(18)
        layout.addWidget(title_label)

        for key, value in rows:
            row = QHBoxLayout()
            row.setSpacing(5)
            left = QLabel(key)
            left.setObjectName("metricName")
            left.setMinimumWidth(108)
            right = QLabel(value)
            right.setObjectName("metricValue")
            right.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            right.setTextInteractionFlags(Qt.TextSelectableByMouse)
            row.addWidget(left, 0)
            row.addWidget(right, 1)
            layout.addLayout(row)
            self.values[key] = right

            if key in progress_keys:
                bar = QProgressBar()
                bar.setObjectName("metricBar")
                bar.setRange(0, 100)
                bar.setValue(0)
                bar.setTextVisible(False)
                bar.setFixedHeight(11)
                layout.addWidget(bar)
                self.bars[key] = bar

    def set(self, key: str, value: object) -> None:
        if key in self.values:
            self.values[key].setText(str(value))

    def set_progress(self, key: str, value: float) -> None:
        if key in self.bars:
            self.bars[key].setValue(int(max(0.0, min(100.0, value))))


class WeightPanel(QFrame):
    """Inline adaptive-weight rows matching the reference screenshot."""

    WEIGHT_COLORS = {
        "Safety": "#2497ff",
        "Energy": "#4ec8a0",
        "Time": "#ff9c47",
        "Tracking": "#8270f8",
        "Mission": "#ffc640",
    }

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("panel")
        self.setMinimumHeight(145)
        self._bars: dict[str, QProgressBar] = {}
        self._values: dict[str, QLabel] = {}
        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 5, 8, 6)
        outer.setSpacing(3)
        title = QLabel("ADAPTIVE WEIGHTS (DATA-DRIVEN)")
        title.setObjectName("panelTitle")
        outer.addWidget(title)
        for key in ("Safety", "Energy", "Time", "Tracking", "Mission"):
            row = QHBoxLayout()
            row.setSpacing(6)
            name = QLabel(key)
            name.setMinimumWidth(64)
            name.setObjectName("metricName")
            value = QLabel("0.20")
            value.setMinimumWidth(42)
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value.setObjectName("metricValue")
            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setValue(20)
            bar.setTextVisible(False)
            bar.setFixedHeight(14)
            bar.setObjectName("weightBar")
            bar.setStyleSheet(
                "QProgressBar {background:#122d40;border:0;border-radius:3px;}"
                f"QProgressBar::chunk {{background:{self.WEIGHT_COLORS[key]};border-radius:3px;}}"
            )
            row.addWidget(name)
            row.addWidget(value)
            row.addWidget(bar, 1)
            outer.addLayout(row)
            self._values[key] = value
            self._bars[key] = bar

    def set_weights(self, weights: dict[str, float]) -> None:
        for key in self._bars:
            value = float(weights.get(key.lower(), 0.0))
            self._values[key].setText(f"{value:.2f}")
            self._bars[key].setValue(int(round(value * 100)))


class ChartFrame(QFrame):
    def __init__(self, title: str, child: QWidget, min_height: int = 180) -> None:
        super().__init__()
        self.setObjectName("panel")
        self.setMinimumHeight(min_height)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 7, 8, 7)
        layout.setSpacing(3)
        if title.strip():
            label = QLabel(title.upper())
            label.setObjectName("panelTitle")
            layout.addWidget(label)
        layout.addWidget(child, 1)


class PlotWidget(QWidget):
    """Time-series plot with explicit axes, tick values, units and margins."""

    def __init__(
        self,
        title: str,
        y_label: str,
        y_min: float,
        y_max: float,
        y_step: float,
        line_color: str,
    ) -> None:
        super().__init__()
        self.title = title
        self.y_label = y_label
        self.y_min = y_min
        self.y_max = y_max
        self.y_step = y_step
        self.line_color = QColor(line_color)
        self.times: list[float] = []
        self.values: list[float] = []
        self.setMinimumHeight(145)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_data(self, times: list[float], values: list[float]) -> None:
        self.times = list(times)[-610:]
        self.values = list(values)[-610:]
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt API
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#031522"))

        width = self.width()
        height = self.height()
        left, right, top, bottom = 50, 12, 20, 30
        plot = QRect(left, top, max(10, width - left - right), max(10, height - top - bottom))

        # Six evenly spaced horizontal levels, each with a readable value.
        y_count = int(round((self.y_max - self.y_min) / self.y_step))
        font = QFont("Arial", 8)
        painter.setFont(font)
        for i in range(y_count + 1):
            value = self.y_min + i * self.y_step
            y = plot.bottom() - int(round((value - self.y_min) / (self.y_max - self.y_min) * plot.height()))
            painter.setPen(QPen(QColor(COLORS["grid"]), 1))
            painter.drawLine(plot.left(), y, plot.right(), y)
            painter.setPen(QColor("#9abacc"))
            text = f"{value:g}"
            painter.drawText(QRect(16, y - 7, left - 21, 15), Qt.AlignRight | Qt.AlignVCenter, text)

        # Expand the time axis in steps as the simulation runs. This keeps the
        # first few seconds readable instead of pinning all points to the left.
        latest_time = max(self.times) if self.times else 0.0
        x_max = max(20.0, min(120.0, math.ceil(max(latest_time, 1.0) / 20.0) * 20.0))
        x_step = 5.0 if x_max <= 20.0 else (10.0 if x_max <= 60.0 else 20.0)
        x_value = 0.0
        while x_value <= x_max + 1e-6:
            x = plot.left() + int(round(x_value / x_max * plot.width()))
            painter.setPen(QPen(QColor(COLORS["grid"]), 1))
            painter.drawLine(x, plot.top(), x, plot.bottom())
            painter.setPen(QColor("#9abacc"))
            painter.drawText(QRect(x - 18, plot.bottom() + 3, 36, 13), Qt.AlignHCenter | Qt.AlignTop, f"{x_value:g}")
            x_value += x_step

        painter.setPen(QPen(QColor("#588298"), 1))
        painter.drawLine(plot.left(), plot.top(), plot.left(), plot.bottom())
        painter.drawLine(plot.left(), plot.bottom(), plot.right(), plot.bottom())

        # Labels are placed outside the graph area so they never overlap ticks.
        painter.setPen(QColor("#d7efff"))
        painter.setFont(QFont("Arial", 9, QFont.DemiBold))
        painter.drawText(QRect(left, 0, plot.width(), 13), Qt.AlignHCenter | Qt.AlignVCenter, self.title)
        painter.setFont(QFont("Arial", 8))
        painter.drawText(QRect(plot.left(), height - 14, plot.width(), 12), Qt.AlignHCenter | Qt.AlignVCenter, "Time (s)")
        painter.save()
        painter.translate(7, plot.center().y())
        painter.rotate(-90)
        painter.drawText(QRect(int(-plot.height() / 2), -7, plot.height(), 14), Qt.AlignHCenter | Qt.AlignVCenter, self.y_label)
        painter.restore()

        # Keep points in range; this makes labels and the plotted trace consistent.
        if len(self.times) >= 2 and len(self.values) >= 2:
            points = []
            for t, value in zip(self.times, self.values):
                px = plot.left() + np.clip(float(t) / x_max, 0.0, 1.0) * plot.width()
                py = plot.bottom() - np.clip(
                    (float(value) - self.y_min) / (self.y_max - self.y_min), 0.0, 1.0
                ) * plot.height()
                points.append((int(round(px)), int(round(py))))
            painter.setClipRect(plot)
            painter.setPen(QPen(self.line_color, 2.1, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            for a, b in zip(points, points[1:]):
                painter.drawLine(a[0], a[1], b[0], b[1])
            painter.setClipping(False)
            last_value = self.values[-1]
            painter.setPen(QColor("#e3f7ff"))
            painter.setFont(QFont("Arial", 8, QFont.Bold))
            painter.drawText(QRect(plot.right() - 60, plot.top() + 2, 57, 14), Qt.AlignRight, f"{last_value:.2f}")
        painter.end()


class TopView(QWidget):
    """XY map showing the safe route, obstacles, start, target and current drone."""

    def __init__(self, sim) -> None:
        super().__init__()
        self.sim = sim
        self.setMinimumHeight(220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setToolTip(
            "Drag RED static obstacles, ORANGE dynamic obstacles, or the GREEN goal in this TOP VIEW map. "
            "Use the customization controls below for precise position and motion settings."
        )
        self.setCursor(Qt.ArrowCursor)
        self._drag_obstacle_index: int | None = None
        self._drag_dynamic_obstacle_index: int | None = None
        self._drag_mode: str | None = None
        self._drag_changed = False
        self.on_obstacle_moved = None
        self.on_dynamic_obstacle_moved = None
        self.on_destination_moved = None

    def _plot_rect(self) -> QRect:
        return QRect(54, 16, max(20, self.width() - 71), max(20, self.height() - 52))

    def world_to_screen(self, x: float, y: float) -> tuple[int, int]:
        plot = self._plot_rect()
        px = plot.left() + (float(x) - MINX) / (MAXX - MINX) * plot.width()
        py = plot.bottom() - (float(y) - MINY) / (MAXY - MINY) * plot.height()
        return int(round(px)), int(round(py))

    def screen_to_world(self, px: float, py: float) -> tuple[float, float]:
        plot = self._plot_rect()
        x = MINX + (float(px) - plot.left()) / max(1, plot.width()) * (MAXX - MINX)
        y = MINY + (plot.bottom() - float(py)) / max(1, plot.height()) * (MAXY - MINY)
        return (
            float(np.clip(x, MINX, MAXX)),
            float(np.clip(y, MINY, MAXY)),
        )

    def _static_obstacle_at(self, px: float, py: float) -> int | None:
        if not getattr(self.sim, "static_enabled", True):
            return None
        sx = self._plot_rect().width() / (MAXX - MINX)
        sy = self._plot_rect().height() / (MAXY - MINY)
        candidates = []
        for index, (ox, oy, hx, hy, _height) in enumerate(
            getattr(self.sim, "static_obstacles", [])
        ):
            cx, cy = self.world_to_screen(ox, oy)
            radius = max(10, int(round(max(hx, hy) * max(sx, sy))) + 6)
            distance_sq = (float(px) - cx) ** 2 + (float(py) - cy) ** 2
            if distance_sq <= radius * radius:
                candidates.append((distance_sq, index))
        return min(candidates)[1] if candidates else None

    def _dynamic_obstacle_at(self, px: float, py: float) -> int | None:
        if not getattr(self.sim, "dynamic_enabled", True):
            return None
        sx = self._plot_rect().width() / (MAXX - MINX)
        sy = self._plot_rect().height() / (MAXY - MINY)
        candidates = []
        for index, (ox, oy, hx, hy, _height) in enumerate(
            getattr(self.sim, "dynamic_obstacles", [])
        ):
            cx, cy = self.world_to_screen(ox, oy)
            radius = max(12, int(round((max(hx, hy) + 0.18) * max(sx, sy))) + 5)
            distance_sq = (float(px) - cx) ** 2 + (float(py) - cy) ** 2
            if distance_sq <= radius * radius:
                candidates.append((distance_sq, index))
        return min(candidates)[1] if candidates else None

    def _target_at(self, px: float, py: float) -> bool:
        tx, ty = self.world_to_screen(*self.sim.destination[:2])
        return (float(px) - tx) ** 2 + (float(py) - ty) ** 2 <= 15 ** 2

    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt API
        if event.button() == Qt.LeftButton:
            point = event.position().toPoint()
            if self._target_at(point.x(), point.y()):
                self._drag_mode = "target"
                self._drag_changed = False
                self.sim.obstacle_editing = True
                self.setCursor(Qt.ClosedHandCursor)
                event.accept()
                return
            dynamic_index = self._dynamic_obstacle_at(point.x(), point.y())
            if dynamic_index is not None:
                self._drag_dynamic_obstacle_index = dynamic_index
                self._drag_mode = "dynamic_obstacle"
                self._drag_changed = False
                self.sim.obstacle_editing = True
                self.setCursor(Qt.ClosedHandCursor)
                event.accept()
                return
            obstacle_index = self._static_obstacle_at(point.x(), point.y())
            if obstacle_index is not None:
                self._drag_obstacle_index = obstacle_index
                self._drag_mode = "obstacle"
                self._drag_changed = False
                self.sim.obstacle_editing = True
                self.setCursor(Qt.ClosedHandCursor)
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802 - Qt API
        point = event.position().toPoint()
        if self._drag_mode == "target" and (event.buttons() & Qt.LeftButton):
            x, y = self.screen_to_world(point.x(), point.y())
            changed = self.sim.move_destination(x, y, replan=False)
            self._drag_changed = self._drag_changed or changed
            self.update()
            event.accept()
            return
        if self._drag_obstacle_index is not None and (event.buttons() & Qt.LeftButton):
            x, y = self.screen_to_world(point.x(), point.y())
            changed = self.sim.move_static_obstacle(
                self._drag_obstacle_index, x, y, replan=False
            )
            self._drag_changed = self._drag_changed or changed
            self.update()
            event.accept()
            return

        if self._drag_dynamic_obstacle_index is not None and (event.buttons() & Qt.LeftButton):
            x, y = self.screen_to_world(point.x(), point.y())
            changed = self.sim.move_dynamic_obstacle_to_position(
                self._drag_dynamic_obstacle_index, x, y, replan=False
            )
            self._drag_changed = self._drag_changed or changed
            self.update()
            event.accept()
            return

        hovering = (
            self._target_at(point.x(), point.y())
            or self._static_obstacle_at(point.x(), point.y()) is not None
            or self._dynamic_obstacle_at(point.x(), point.y()) is not None
        )
        self.setCursor(Qt.OpenHandCursor if hovering else Qt.ArrowCursor)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802 - Qt API
        if event.button() == Qt.LeftButton and self._drag_mode == "target":
            changed = self._drag_changed
            self._drag_mode = None
            self._drag_changed = False
            self.sim.obstacle_editing = False
            self.setCursor(Qt.ArrowCursor)
            if changed:
                self.sim.replan(reason="DESTINATION REPOSITIONED")
                if callable(self.on_destination_moved):
                    self.on_destination_moved()
            self.update()
            event.accept()
            return
        if event.button() == Qt.LeftButton and self._drag_dynamic_obstacle_index is not None:
            changed = self._drag_changed
            index = self._drag_dynamic_obstacle_index
            self._drag_dynamic_obstacle_index = None
            self._drag_mode = None
            self._drag_changed = False
            self.sim.obstacle_editing = False
            self.setCursor(Qt.ArrowCursor)
            if changed:
                self.sim.replan(reason="DYNAMIC OBSTACLE REPOSITIONED")
                if callable(self.on_dynamic_obstacle_moved):
                    self.on_dynamic_obstacle_moved(index)
            self.update()
            event.accept()
            return
        if event.button() == Qt.LeftButton and self._drag_obstacle_index is not None:
            changed = self._drag_changed
            self._drag_obstacle_index = None
            self._drag_mode = None
            self._drag_changed = False
            self.sim.obstacle_editing = False
            self.setCursor(Qt.ArrowCursor)
            if changed:
                self.sim.replan(reason="STATIC OBSTACLE REPOSITIONED")
                if callable(self.on_obstacle_moved):
                    self.on_obstacle_moved()
            self.update()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt API
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QColor("#031522"))
        plot = self._plot_rect()

        # Grid and tick labels adapt to the expanded world bounds.
        def ticks_for(low: float, high: float) -> list[int]:
            span = high - low
            step = 4 if span >= 14 else (2 if span >= 8 else 1)
            first = int(math.ceil(low / step) * step)
            return list(range(first, int(math.floor(high / step) * step) + 1, step))

        x_ticks = ticks_for(MINX, MAXX)
        y_ticks = ticks_for(MINY, MAXY)
        for value in x_ticks:
            x, _ = self.world_to_screen(value, 0)
            p.setPen(QPen(QColor("#123247"), 1))
            p.drawLine(x, plot.top(), x, plot.bottom())
            p.setPen(QColor("#abcada"))
            p.setFont(QFont("Arial", 8))
            p.drawText(QRect(x - 14, plot.bottom() + 3, 28, 13), Qt.AlignHCenter | Qt.AlignTop, str(value))
        for value in y_ticks:
            _, y = self.world_to_screen(0, value)
            p.setPen(QPen(QColor("#123247"), 1))
            p.drawLine(plot.left(), y, plot.right(), y)
            p.setPen(QColor("#abcada"))
            p.setFont(QFont("Arial", 8))
            p.drawText(QRect(17, y - 7, 31, 14), Qt.AlignRight | Qt.AlignVCenter, str(value))

        p.setPen(QPen(QColor("#4b7790"), 1))
        p.drawRect(plot)

        # Decorative city blocks in the XY map.
        sx = plot.width() / (MAXX - MINX)
        sy = plot.height() / (MAXY - MINY)
        for bx, by, hx, hy, height in getattr(self.sim, "buildings", []):
            left, top = self.world_to_screen(bx - hx, by + hy)
            right, bottom = self.world_to_screen(bx + hx, by - hy)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(65, 88, 111, 170)))
            p.drawRect(QRect(min(left, right), min(top, bottom), max(1, abs(right - left)), max(1, abs(bottom - top))))

        # Draw the full sweep of each moving hazard first, so motion is obvious
        # even in a paused screenshot. Orange dashed lines show the travel range.
        if getattr(self.sim, "dynamic_enabled", True):
            p.setPen(QPen(QColor("#9e5721"), 2, Qt.DashLine))
            for x1, y1, x2, y2 in getattr(self.sim, "dynamic_obstacle_tracks", []):
                a = self.world_to_screen(x1, y1)
                b = self.world_to_screen(x2, y2)
                p.drawLine(a[0], a[1], b[0], b[1])
                p.setBrush(QBrush(QColor("#ffb34e")))
                p.setPen(QPen(QColor("#ffd27f"), 1))
                p.drawEllipse(a[0] - 3, a[1] - 3, 6, 6)
                p.drawEllipse(b[0] - 3, b[1] - 3, 6, 6)

        # Static obstacles are red; dynamic hazards are larger orange/yellow
        # markers labelled D1/D2. These update continuously with the mocap bodies.
        if getattr(self.sim, "static_enabled", True):
            for index, (ox, oy, hx, hy, height) in enumerate(getattr(self.sim, "static_obstacles", []), start=1):
                cx, cy = self.world_to_screen(ox, oy)
                radius_x = max(7, int(round(max(hx, hy) * sx)))
                radius_y = max(7, int(round(max(hx, hy) * sy)))
                p.setPen(QPen(QColor("#ffd6d8"), 1.2))
                p.setBrush(QBrush(QColor(237, 65, 73, 235)))
                p.drawEllipse(cx - radius_x, cy - radius_y, 2 * radius_x, 2 * radius_y)
                p.setPen(QColor("#ffffff"))
                p.setFont(QFont("Arial", 8, QFont.Bold))
                p.drawText(QRect(cx - radius_x, cy - 6, 2 * radius_x, 12), Qt.AlignCenter, f"S{index}")

        if getattr(self.sim, "dynamic_enabled", True):
            for index, (ox, oy, hx, hy, height) in enumerate(getattr(self.sim, "dynamic_obstacles", []), start=1):
                cx, cy = self.world_to_screen(ox, oy)
                radius_x = max(9, int(round((max(hx, hy) + 0.18) * sx)))
                radius_y = max(9, int(round((max(hx, hy) + 0.18) * sy)))
                # Glow / clearance halo.
                p.setPen(QPen(QColor(255, 177, 70, 180), 1.5))
                p.setBrush(QBrush(QColor(255, 150, 47, 42)))
                p.drawEllipse(cx - radius_x, cy - radius_y, 2 * radius_x, 2 * radius_y)
                # Solid orange body marker.
                body_rx = max(7, int(round(max(hx, hy) * sx)))
                body_ry = max(7, int(round(max(hx, hy) * sy)))
                p.setBrush(QBrush(QColor("#ff861c")))
                p.setPen(QPen(QColor("#fff0ae"), 2))
                p.drawEllipse(cx - body_rx, cy - body_ry, 2 * body_rx, 2 * body_ry)
                p.setPen(QColor("#351b00"))
                p.setFont(QFont("Arial", 8, QFont.Bold))
                p.drawText(QRect(cx - body_rx, cy - 6, 2 * body_rx, 12), Qt.AlignCenter, f"D{index}")

        # Show the currently selected route, not always the best-ranked route.
        ranking = getattr(self.sim, "path_ranking", [])
        if ranking:
            selected = next(
                (item for item in ranking if int(item.get("path_id", -1)) == int(getattr(self.sim, "selected_path_id", -1))),
                ranking[0],
            )
            best = selected.get("path", [])
            if len(best) >= 2:
                p.setPen(QPen(QColor("#4b788f"), 1.3, Qt.DashLine))
                coords = [self.world_to_screen(x, y) for x, y in best]
                for a, b in zip(coords, coords[1:]):
                    p.drawLine(a[0], a[1], b[0], b[1])

        active = [tuple(self.sim.pos[:2])]
        active.extend(tuple(point[:2]) for point in self.sim.waypoints[self.sim.wp_index :])
        if len(active) >= 2:
            p.setPen(QPen(QColor("#60e4ff"), 2.2, Qt.DashLine, Qt.RoundCap))
            coords = [self.world_to_screen(x, y) for x, y in active]
            for a, b in zip(coords, coords[1:]):
                p.drawLine(a[0], a[1], b[0], b[1])

        # Start and target markers.
        start_x, start_y = self.world_to_screen(START[0], START[1])
        target_x, target_y = self.world_to_screen(*self.sim.destination[:2])
        p.setPen(QPen(QColor("#d5edff"), 1))
        p.setBrush(QBrush(QColor("#087df5")))
        p.drawEllipse(start_x - 7, start_y - 7, 14, 14)
        p.setBrush(QBrush(QColor("#08eb7c")))
        p.setPen(QPen(QColor("#a3ffd3"), 1))
        p.drawEllipse(target_x - 8, target_y - 8, 16, 16)
        drone_x, drone_y = self.world_to_screen(self.sim.pos[0], self.sim.pos[1])
        p.setBrush(QBrush(QColor("#42d5ff")))
        p.setPen(QPen(QColor("#d7f8ff"), 1.3))
        p.drawEllipse(drone_x - 5, drone_y - 5, 10, 10)

        p.setPen(QColor("#d7efff"))
        p.setFont(QFont("Arial", 8))
        p.drawText(QRect(plot.left(), self.height() - 16, plot.width(), 12), Qt.AlignHCenter, "X (m)")
        p.save()
        p.translate(7, plot.center().y())
        p.rotate(-90)
        p.drawText(QRect(int(-plot.height() / 2), -6, plot.height(), 12), Qt.AlignHCenter, "Y (m)")
        p.restore()
        p.end()


class MjView(QFrame):
    """Interactive MuJoCo viewport with 360-degree camera orbit and zoom."""

    def __init__(self, sim) -> None:
        super().__init__()
        self.sim = sim
        self.setObjectName("viewport")
        self.setMinimumSize(470, 360)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.renderer = None
        self.renderer_error = ""
        try:
            # Keep requests below MuJoCo's default offscreen framebuffer dimensions.
            self.renderer = mujoco.Renderer(sim.model, height=480, width=640)
        except Exception as exc:
            self.renderer_error = str(exc)
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self._home_azimuth = 138.0
        self._home_elevation = -32.0
        self._home_distance = 25.5
        self._last_mouse_pos = None
        self.camera.lookat[:] = [6.0, 5.0, 0.75]
        self.camera.distance = self._home_distance
        self.camera.azimuth = self._home_azimuth
        self.camera.elevation = self._home_elevation
        self.orbit_timer = QTimer(self)
        self.orbit_timer.setInterval(40)
        self.orbit_timer.timeout.connect(self._advance_orbit)

    def set_auto_orbit(self, enabled: bool) -> None:
        """Start/stop continuous azimuth rotation across the complete 360 degrees."""
        if enabled:
            self.orbit_timer.start()
        else:
            self.orbit_timer.stop()
        self.update()

    def is_auto_orbiting(self) -> bool:
        return self.orbit_timer.isActive()

    def _advance_orbit(self) -> None:
        self.camera.azimuth = (float(self.camera.azimuth) + 1.5) % 360.0
        self.update()

    def set_camera_preset(self, preset: str) -> None:
        """Set one of the named camera viewpoints while keeping the arena centered."""
        self.camera.lookat[:] = [6.0, 5.0, 0.75]
        if preset == "top":
            self.camera.azimuth = 90.0
            self.camera.elevation = -85.0
            self.camera.distance = 27.5
        elif preset == "side":
            self.camera.azimuth = 90.0
            self.camera.elevation = -12.0
            self.camera.distance = 25.5
        else:
            self.camera.azimuth = self._home_azimuth
            self.camera.elevation = self._home_elevation
            self.camera.distance = self._home_distance
        self.update()

    def mousePressEvent(self, event) -> None:  # noqa: N802 - Qt API
        if event.button() == Qt.LeftButton:
            self._last_mouse_pos = event.position().toPoint()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802 - Qt API
        if self._last_mouse_pos is not None and (event.buttons() & Qt.LeftButton):
            current = event.position().toPoint()
            delta = current - self._last_mouse_pos
            self._last_mouse_pos = current
            self.camera.azimuth = (float(self.camera.azimuth) + float(delta.x()) * 0.65) % 360.0
            self.camera.elevation = float(np.clip(self.camera.elevation - float(delta.y()) * 0.42, -88.0, -5.0))
            self.update()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802 - Qt API
        if event.button() == Qt.LeftButton:
            self._last_mouse_pos = None
            self.setCursor(Qt.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event) -> None:  # noqa: N802 - Qt API
        steps = float(event.angleDelta().y()) / 120.0
        self.camera.distance = float(np.clip(self.camera.distance * math.exp(-0.12 * steps), 7.5, 48.0))
        self.update()
        event.accept()

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt API
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        try:
            if self.renderer is None:
                raise RuntimeError(self.renderer_error or "MuJoCo renderer could not be initialized")
            self.renderer.update_scene(self.sim.data, camera=self.camera)
            rgb = self.renderer.render()
            h, w, _ = rgb.shape
            image = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()
            image = image.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            painter.fillRect(self.rect(), QColor("#031522"))
            painter.drawImage((self.width() - image.width()) // 2, (self.height() - image.height()) // 2, image)

            # Camera control legend layered over the actual MuJoCo render.
            painter.setPen(QPen(QColor("#2c6987"), 1))
            painter.setBrush(QBrush(QColor(3, 22, 36, 225)))
            painter.drawRoundedRect(QRect(12, 12, 204, 122), 7, 7)
            painter.setFont(QFont("Arial", 9, QFont.DemiBold))
            painter.setPen(QColor("#49d2ff"))
            painter.drawText(22, 29, "3D FLIGHT VIEW | 360° CAMERA")
            painter.setFont(QFont("Arial", 8))
            for index, text in enumerate(("Drag left mouse = orbit", "Scroll = zoom in/out", "AUTO 360° = continuous rotation")):
                painter.setPen(QColor("#e4f5ff"))
                painter.drawText(22, 49 + index * 16, text)
            painter.setPen(QColor("#bdddea"))
            painter.drawText(22, 111, f"Azimuth {self.camera.azimuth % 360:.0f}°  |  Elevation {self.camera.elevation:.0f}°")
        except Exception as exc:  # Keep the rest of the UI usable if GL fails.
            painter.fillRect(self.rect(), QColor("#031522"))
            painter.setPen(QColor("#ffcc73"))
            painter.drawText(20, 32, "3D renderer error")
            painter.setPen(QColor("#d9efff"))
            painter.drawText(QRect(20, 43, max(200, self.width() - 40), 110), Qt.TextWordWrap, str(exc)[:300])
        painter.end()

    def close_renderer(self) -> None:
        self.orbit_timer.stop()
        try:
            if self.renderer is not None:
                self.renderer.close()
        except Exception:
            pass


class Dashboard(QWidget):
    def __init__(self, sim) -> None:
        super().__init__()
        self.sim = sim
        self.setObjectName("dashboardRoot")
        self.setWindowTitle("ACMFDS | Autonomous Quadrotor Simulation")
        self.resize(1600, 1000)
        self.setMinimumSize(1180, 760)
        self.setStyleSheet("""
            QWidget { background: transparent; color: #d9efff; font-family: Arial; font-size: 12px; }
            #dashboardRoot { background: #020d17; }
            #panel, #viewport { background: #041826; border: 1px solid #08729b; border-radius: 7px; }
            #routeOptionsBar { background: #041826; border: 1px solid #08729b; border-radius: 6px; }
            QComboBox { background: #061b2a; color: #d9efff; border: 1px solid #1f6684; border-radius: 4px; padding: 4px 7px; }
            QComboBox QAbstractItemView { background: #061b2a; color: #d9efff; selection-background-color: #0b5573; }
            QCheckBox { color: #d9efff; spacing: 5px; }
            QCheckBox::indicator { width: 14px; height: 14px; }
            QCheckBox::indicator:unchecked { background: #061b2a; border: 1px solid #33718a; border-radius: 3px; }
            QCheckBox::indicator:checked { background: #18c889; border: 1px solid #18c889; border-radius: 3px; }
            #panelTitle { color: #45caff; font-size: 13px; font-weight: bold; padding-bottom: 4px;
                          border-bottom: 1px solid #153d54; }
            #metricName { color: #d3e9f5; }
            #metricValue { color: #f2f9ff; }
            QProgressBar#metricBar { background: #142e41; border: 0; border-radius: 3px; }
            QProgressBar#metricBar::chunk { background: #21c889; border-radius: 3px; }
            QPushButton { background: #06263a; border: 1px solid #08729b; border-radius: 5px;
                          color: #d9efff; padding: 6px 12px; font-weight: bold; }
            QPushButton:hover { background: #0b3952; }
        """)

        root = QVBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 7)
        root.setSpacing(8)

        # Header bar.
        header = QHBoxLayout()
        header.setSpacing(12)
        title_box = QVBoxLayout()
        title_box.setSpacing(1)
        title = QLabel("ACMFDS  |  AUTONOMOUS QUADROTOR SIMULATION")
        title.setStyleSheet("font-size:23px;font-weight:800;color:#dceeff;letter-spacing:0.5px;")
        subtitle = QLabel("Data-Driven Decision Making   |   Obstacle Avoidance   |   Safe & Efficient Navigation")
        subtitle.setStyleSheet("font-size:14px;color:#48c9ff;")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box, 1)

        self.live_badge = QLabel("●  REAL-TIME MuJoCo PHYSICS")
        self.live_badge.setAlignment(Qt.AlignCenter)
        self.live_badge.setStyleSheet("color:#11f18a;border:1px solid #087d60;border-radius:7px;padding:8px 12px;font-weight:bold;")
        header.addWidget(self.live_badge)
        self.clock = QLabel("Sim Time: 0.0 s")
        self.clock.setAlignment(Qt.AlignCenter)
        self.clock.setStyleSheet("color:#d7efff;border:1px solid #075d80;border-radius:6px;padding:8px 12px;")
        header.addWidget(self.clock)
        root.addLayout(header)
        line = QFrame()
        line.setFixedHeight(1)
        line.setStyleSheet("background:#08729b;border:0;")
        root.addWidget(line)

        # Main split: the left telemetry column spans both upper and graph areas.
        main = QHBoxLayout()
        main.setSpacing(9)
        left = QVBoxLayout()
        left.setSpacing(8)
        left.setContentsMargins(0, 0, 0, 0)

        self.status = Panel(
            "Simulation Status",
            [("Time", "0.0 s"), ("Decision", "FORWARD"), ("Context", "CLEAR"), ("Mode", "NAVIGATING"), ("Simulation", "RUNNING")],
        )
        self.state = Panel(
            "Quadrotor State",
            [
                ("Position", "[0.00, 0.00, 2.00] m"),
                ("Velocity", "[0.00, 0.00, 0.00] m/s"),
                ("Speed", "0.00 m/s"),
                ("Altitude", "2.00 m"),
            ],
        )
        self.mission = Panel(
            "Mission Progress",
            [("Target", str(tuple(round(v, 2) for v in self.sim.destination)) + " m"), ("Distance", "0.00 m"), ("Progress", "0.0 %"), ("Waypoint", "1 / 1")],
            progress_keys={"Progress"},
        )
        self.safety = Panel(
            "Safety & Risk",
            [("Nearest obstacle", "0.00 m"), ("Risk level", "LOW"), ("Obstacle risk", "0.00"), ("Status", "SAFE")],
        )
        self.energy = Panel("Energy", [("Battery", "100.0 %")], progress_keys={"Battery"})
        self.controls = Panel(
            "Control Outputs",
            [("Force X", "0.00 N"), ("Force Y", "0.00 N"), ("Thrust Z", "11.77 N"), ("Control magnitude", "11.77 N")],
        )
        for panel in (self.status, self.state, self.mission, self.safety, self.energy, self.controls):
            left.addWidget(panel)
        left.addStretch(1)
        left_widget = QWidget()
        left_widget.setLayout(left)
        left_widget.setFixedWidth(316)
        main.addWidget(left_widget)

        # Main right-side content: 3D flight view above three readable graphs.
        content = QVBoxLayout()
        content.setSpacing(8)
        top_row = QHBoxLayout()
        top_row.setSpacing(9)
        self.view = MjView(self.sim)
        view_stack = QVBoxLayout()
        view_stack.setContentsMargins(0, 0, 0, 0)
        view_stack.setSpacing(6)
        camera_bar = QHBoxLayout()
        camera_bar.setSpacing(6)
        camera_help = QLabel("DRAG TO ORBIT · SCROLL TO ZOOM")
        camera_help.setStyleSheet("color:#8db7cc;font-size:10px;")
        camera_bar.addWidget(camera_help, 1)
        self.orbit_button = QPushButton("AUTO 360° ORBIT")
        self.orbit_button.setToolTip("Continuously rotate the camera through a full 360-degree orbit.")
        self.orbit_button.clicked.connect(self.toggle_camera_orbit)
        camera_bar.addWidget(self.orbit_button)
        home_camera_button = QPushButton("OVERVIEW")
        home_camera_button.clicked.connect(lambda: self.set_camera_preset("home"))
        camera_bar.addWidget(home_camera_button)
        top_camera_button = QPushButton("TOP")
        top_camera_button.clicked.connect(lambda: self.set_camera_preset("top"))
        camera_bar.addWidget(top_camera_button)
        side_camera_button = QPushButton("SIDE")
        side_camera_button.clicked.connect(lambda: self.set_camera_preset("side"))
        camera_bar.addWidget(side_camera_button)
        view_stack.addLayout(camera_bar)
        view_stack.addWidget(self.view, 1)
        view_widget = QWidget()
        view_widget.setLayout(view_stack)
        top_row.addWidget(view_widget, 2.15)

        # Keep the map in its own full-height card. Previously the map, adaptive
        # weights, and objective card were stacked into a short sidebar; on a
        # normal laptop display their minimum heights exceeded the available
        # space and Qt laid the cards over one another. The map now gets the
        # entire sidebar height, with adaptive weights moved to the graph row.
        sidebar = QVBoxLayout()
        sidebar.setSpacing(0)
        self.top_view = TopView(self.sim)
        self.top_view.on_obstacle_moved = self.static_obstacle_moved
        self.top_view.on_dynamic_obstacle_moved = self.dynamic_obstacle_moved
        self.top_view.on_destination_moved = self.destination_moved
        sidebar.addWidget(
            ChartFrame("TOP VIEW · DRAG RED/ORANGE OBSTACLES / GREEN GOAL", self.top_view, min_height=300),
            1,
        )
        self.weight_panel = WeightPanel()
        top_row.addLayout(sidebar, 1)
        content.addLayout(top_row, 6)

        # Interactive path-profile and obstacle controls.
        self.route_options_bar = QFrame()
        self.route_options_bar.setObjectName("routeOptionsBar")
        options_layout = QHBoxLayout(self.route_options_bar)
        options_layout.setContentsMargins(9, 5, 9, 5)
        options_layout.setSpacing(8)
        route_label = QLabel("PATH PROFILE")
        route_label.setObjectName("metricName")
        options_layout.addWidget(route_label)
        self.route_combo = QComboBox()
        self.route_combo.setMinimumWidth(220)
        self.route_combo.setMaximumWidth(310)
        self.route_combo.setToolTip("Choose a candidate route profile. Applying it replans from the current drone position.")
        options_layout.addWidget(self.route_combo)
        self.apply_route_button = QPushButton("USE ROUTE")
        self.apply_route_button.clicked.connect(self.apply_selected_route)
        options_layout.addWidget(self.apply_route_button)
        self.static_toggle = QCheckBox("Static obstacles")
        self.static_toggle.setChecked(bool(getattr(self.sim, "static_enabled", True)))
        self.dynamic_toggle = QCheckBox("Dynamic obstacles")
        self.dynamic_toggle.setChecked(bool(getattr(self.sim, "dynamic_enabled", True)))
        options_layout.addWidget(self.static_toggle)
        options_layout.addWidget(self.dynamic_toggle)
        self.replan_button = QPushButton("REPLAN")
        self.replan_button.clicked.connect(self.manual_replan)
        options_layout.addWidget(self.replan_button)
        options_layout.addStretch(1)
        self.static_toggle.toggled.connect(self.obstacle_options_changed)
        self.dynamic_toggle.toggled.connect(self.obstacle_options_changed)
        self._route_options_signature = None
        self.refresh_route_options(force=True)
        content.addWidget(self.route_options_bar)

        # Obstacle customization is split across two clear rows. The previous
        # single-line layout tried to fit every static/dynamic control into one
        # horizontal strip, so labels and the APPLY button were clipped on laptop
        # displays. The first row is for type, obstacle, position and apply; the
        # second row is for dynamic patrol settings plus feedback.
        obstacle_editor = QFrame()
        obstacle_editor.setObjectName("routeOptionsBar")
        obstacle_editor_layout = QVBoxLayout(obstacle_editor)
        obstacle_editor_layout.setContentsMargins(10, 7, 10, 7)
        obstacle_editor_layout.setSpacing(5)

        obstacle_position_row = QHBoxLayout()
        obstacle_position_row.setSpacing(8)
        obstacle_editor_label = QLabel("CUSTOMIZE OBSTACLE")
        obstacle_editor_label.setObjectName("metricName")
        obstacle_editor_label.setMinimumWidth(150)
        obstacle_position_row.addWidget(obstacle_editor_label)

        self.obstacle_type_combo = QComboBox()
        self.obstacle_type_combo.addItem("Static", "static")
        self.obstacle_type_combo.addItem("Dynamic", "dynamic")
        self.obstacle_type_combo.setMinimumWidth(102)
        self.obstacle_type_combo.setToolTip("Choose static red hazards or dynamic orange moving hazards.")
        obstacle_position_row.addWidget(self.obstacle_type_combo)

        self.obstacle_index_combo = QComboBox()
        self.obstacle_index_combo.setMinimumWidth(126)
        self.obstacle_index_combo.setToolTip("Select the obstacle to customize.")
        obstacle_position_row.addWidget(self.obstacle_index_combo)

        x_label = QLabel("X")
        x_label.setObjectName("metricName")
        obstacle_position_row.addWidget(x_label)
        self.obstacle_x_spin = QDoubleSpinBox()
        self.obstacle_x_spin.setRange(MINX, MAXX)
        self.obstacle_x_spin.setDecimals(2)
        self.obstacle_x_spin.setSingleStep(0.25)
        self.obstacle_x_spin.setSuffix(" m")
        self.obstacle_x_spin.setMinimumWidth(112)
        self.obstacle_x_spin.setToolTip("X coordinate in metres. For dynamic hazards this is the centre of its patrol track.")
        obstacle_position_row.addWidget(self.obstacle_x_spin)

        y_label = QLabel("Y")
        y_label.setObjectName("metricName")
        obstacle_position_row.addWidget(y_label)
        self.obstacle_y_spin = QDoubleSpinBox()
        self.obstacle_y_spin.setRange(MINY, MAXY)
        self.obstacle_y_spin.setDecimals(2)
        self.obstacle_y_spin.setSingleStep(0.25)
        self.obstacle_y_spin.setSuffix(" m")
        self.obstacle_y_spin.setMinimumWidth(112)
        self.obstacle_y_spin.setToolTip("Y coordinate in metres. For dynamic hazards this is the centre of its patrol track.")
        obstacle_position_row.addWidget(self.obstacle_y_spin)

        self.apply_obstacle_button = QPushButton("APPLY CUSTOM")
        self.apply_obstacle_button.setMinimumWidth(132)
        self.apply_obstacle_button.setToolTip("Apply obstacle position and (for dynamic obstacles) patrol settings, then replan the route.")
        self.apply_obstacle_button.clicked.connect(self.apply_obstacle_customization)
        obstacle_position_row.addWidget(self.apply_obstacle_button)
        obstacle_position_row.addStretch(1)
        obstacle_editor_layout.addLayout(obstacle_position_row)

        obstacle_motion_row = QHBoxLayout()
        obstacle_motion_row.setSpacing(8)
        motion_title = QLabel("DYNAMIC MOTION")
        motion_title.setObjectName("metricName")
        motion_title.setMinimumWidth(150)
        obstacle_motion_row.addWidget(motion_title)

        self.motion_axis_label = QLabel("Axis")
        self.motion_axis_label.setObjectName("metricName")
        obstacle_motion_row.addWidget(self.motion_axis_label)
        self.motion_axis_combo = QComboBox()
        self.motion_axis_combo.addItem("X sweep", "x")
        self.motion_axis_combo.addItem("Y sweep", "y")
        self.motion_axis_combo.setMinimumWidth(100)
        self.motion_axis_combo.setToolTip("Direction of the dynamic obstacle's patrol.")
        obstacle_motion_row.addWidget(self.motion_axis_combo)

        self.motion_range_label = QLabel("Range")
        self.motion_range_label.setObjectName("metricName")
        obstacle_motion_row.addWidget(self.motion_range_label)
        self.motion_range_spin = QDoubleSpinBox()
        self.motion_range_spin.setRange(0.0, 4.0)
        self.motion_range_spin.setDecimals(1)
        self.motion_range_spin.setSingleStep(0.25)
        self.motion_range_spin.setSuffix(" m")
        self.motion_range_spin.setMinimumWidth(108)
        self.motion_range_spin.setToolTip("Dynamic obstacle moves this far to either side of its patrol centre.")
        obstacle_motion_row.addWidget(self.motion_range_spin)

        self.motion_rate_label = QLabel("Rate")
        self.motion_rate_label.setObjectName("metricName")
        obstacle_motion_row.addWidget(self.motion_rate_label)
        self.motion_rate_spin = QDoubleSpinBox()
        self.motion_rate_spin.setRange(0.05, 1.50)
        self.motion_rate_spin.setDecimals(2)
        self.motion_rate_spin.setSingleStep(0.05)
        self.motion_rate_spin.setSuffix(" rad/s")
        self.motion_rate_spin.setMinimumWidth(118)
        self.motion_rate_spin.setToolTip("Angular rate of the dynamic obstacle's sinusoidal patrol.")
        obstacle_motion_row.addWidget(self.motion_rate_spin)

        self.obstacle_edit_feedback = QLabel("Edit either obstacle type here or drag red/orange markers in Top View.")
        self.obstacle_edit_feedback.setObjectName("metricName")
        self.obstacle_edit_feedback.setWordWrap(True)
        self.obstacle_edit_feedback.setMinimumWidth(250)
        obstacle_motion_row.addWidget(self.obstacle_edit_feedback, 1)
        obstacle_editor_layout.addLayout(obstacle_motion_row)

        self.obstacle_type_combo.currentIndexChanged.connect(self.obstacle_type_changed)
        self.obstacle_index_combo.currentIndexChanged.connect(self.load_obstacle_controls)
        self.motion_axis_combo.currentIndexChanged.connect(self._sync_dynamic_track_range)
        content.addWidget(obstacle_editor)
        self.rebuild_obstacle_index_options()
        self.load_obstacle_controls()

        # Live environment controls: these values change simulated flight physics
        # and decisions, not merely the dashboard labels.
        condition_editor = QFrame()
        condition_editor.setObjectName("routeOptionsBar")
        condition_layout = QHBoxLayout(condition_editor)
        condition_layout.setContentsMargins(9, 5, 9, 5)
        condition_layout.setSpacing(7)
        condition_title = QLabel("FLIGHT CONDITIONS")
        condition_title.setObjectName("metricName")
        condition_layout.addWidget(condition_title)

        condition_layout.addWidget(QLabel("Wind"))
        self.wind_speed_spin = QDoubleSpinBox()
        self.wind_speed_spin.setRange(0.0, 12.0)
        self.wind_speed_spin.setDecimals(1)
        self.wind_speed_spin.setSingleStep(0.5)
        self.wind_speed_spin.setSuffix(" m/s")
        self.wind_speed_spin.setValue(float(self.sim.wind_speed_mps))
        self.wind_speed_spin.setMinimumWidth(92)
        self.wind_speed_spin.setToolTip("Simulated wind speed. Higher wind adds a force disturbance, raises risk and energy use, and reduces commanded speed.")
        condition_layout.addWidget(self.wind_speed_spin)

        condition_layout.addWidget(QLabel("Direction"))
        self.wind_direction_spin = QDoubleSpinBox()
        self.wind_direction_spin.setRange(0.0, 359.0)
        self.wind_direction_spin.setDecimals(0)
        self.wind_direction_spin.setSingleStep(15.0)
        self.wind_direction_spin.setSuffix("°")
        self.wind_direction_spin.setValue(float(self.sim.wind_direction_deg))
        self.wind_direction_spin.setMinimumWidth(76)
        self.wind_direction_spin.setToolTip("Wind force direction in the world frame: 0°=+X/east, 90°=+Y/north.")
        condition_layout.addWidget(self.wind_direction_spin)

        condition_layout.addWidget(QLabel("Battery"))
        self.battery_condition_spin = QDoubleSpinBox()
        self.battery_condition_spin.setRange(0.0, 100.0)
        self.battery_condition_spin.setDecimals(0)
        self.battery_condition_spin.setSingleStep(5.0)
        self.battery_condition_spin.setSuffix(" %")
        self.battery_condition_spin.setValue(float(self.sim.battery))
        self.battery_condition_spin.setMinimumWidth(80)
        self.battery_condition_spin.setToolTip("Set the simulated current battery charge. The return-home reserve rises from 25% to 43% as wind increases; at 8% it makes an emergency landing.")
        condition_layout.addWidget(self.battery_condition_spin)

        self.apply_conditions_button = QPushButton("APPLY CONDITIONS")
        self.apply_conditions_button.setToolTip("Apply live wind/battery conditions and update flight control, risk weights and route planning.")
        self.apply_conditions_button.clicked.connect(self.apply_flight_conditions)
        condition_layout.addWidget(self.apply_conditions_button)
        self.condition_feedback = QLabel(getattr(self.sim, "condition_message", "Calm wind · full battery"))
        self.condition_feedback.setObjectName("metricName")
        self.condition_feedback.setWordWrap(True)
        condition_layout.addWidget(self.condition_feedback, 1)
        content.addWidget(condition_editor)

        graph_row = QHBoxLayout()
        graph_row.setSpacing(9)
        self.distance_plot = PlotWidget("Distance to Target (m)", "Distance (m)", 0.0, 20.0, 4.0, "#23caff")
        self.battery_plot = PlotWidget("Battery Level (%)", "Battery (%)", 0.0, 100.0, 20.0, "#19ef66")
        self.speed_plot = PlotWidget("Speed (m/s)", "Speed (m/s)", 0.0, 3.0, 0.5, "#ff9e32")
        graph_row.addWidget(ChartFrame("", self.distance_plot, min_height=165), 1)
        graph_row.addWidget(ChartFrame("", self.battery_plot, min_height=165), 1)
        graph_row.addWidget(ChartFrame("", self.speed_plot, min_height=165), 1)
        graph_row.addWidget(ChartFrame("", self.weight_panel, min_height=165), 1)
        content.addLayout(graph_row, 2.4)
        main.addLayout(content, 1)
        root.addLayout(main, 1)

        footer = QHBoxLayout()
        footer.setSpacing(8)
        footer_label = QLabel("ACMFDS - Autonomous Quadrotor Simulation   |   Data-Driven Models for Safe and Efficient Navigation")
        footer_label.setStyleSheet("color:#b2d5e8;font-size:11px;")
        footer.addWidget(footer_label, 1)
        self.gps_note = QLabel("GPS SIM: waiting for telemetry")
        self.gps_note.setStyleSheet("color:#75b7d4;font-size:10px;")
        footer.addWidget(self.gps_note)
        self.restart_button = QPushButton("RESTART MISSION ↻")
        self.restart_button.setToolTip("After reaching the destination, press this to return to the blue start marker and run the mission again.")
        self.restart_button.clicked.connect(self.reset_simulation)
        footer.addWidget(self.restart_button)
        tech = QLabel("Built with MuJoCo  |  PySide6  |  Python")
        tech.setStyleSheet("color:#b2d5e8;font-size:11px;")
        footer.addWidget(tech)
        root.addLayout(footer)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(16)
        self.update_display()

    def toggle_camera_orbit(self) -> None:
        enabled = not self.view.is_auto_orbiting()
        self.view.set_auto_orbit(enabled)
        self.orbit_button.setText("STOP 360° ORBIT" if enabled else "AUTO 360° ORBIT")

    def set_camera_preset(self, preset: str) -> None:
        self.view.set_camera_preset(preset)
        if self.view.is_auto_orbiting():
            # Keep orbiting from the newly chosen preset without a second click.
            self.orbit_button.setText("STOP 360° ORBIT")
        else:
            self.orbit_button.setText("AUTO 360° ORBIT")

    def reset_simulation(self) -> None:
        """Restart the mission at the blue START marker."""
        self.sim.reset()
        self.restart_button.setText("RESTART MISSION ↻")
        self.battery_condition_spin.setValue(float(self.sim.battery))
        self.condition_feedback.setText(getattr(self.sim, "condition_message", "Conditions applied"))
        self.refresh_route_options(force=True)
        self.update_display()

    def refresh_route_options(self, force: bool = False) -> None:
        ranking = list(getattr(self.sim, "path_ranking", []))
        signature = tuple(
            (int(item.get("path_id", -1)), str(item.get("profile", "Route")), round(float(item.get("score", 0.0)), 3))
            for item in ranking
        )
        if not force and signature == self._route_options_signature:
            return
        current_id = int(getattr(self.sim, "selected_path_id", 1))
        self.route_combo.blockSignals(True)
        self.route_combo.clear()
        selected_index = 0
        for index, item in enumerate(ranking):
            path_id = int(item.get("path_id", index + 1))
            label = (
                f"#{item.get('rank', index + 1)} {item.get('profile', 'ROUTE')}  |  "
                f"{float(item.get('length', 0.0)):.1f} m  |  score {float(item.get('score', 0.0)):.2f}"
            )
            self.route_combo.addItem(label, path_id)
            if path_id == current_id:
                selected_index = index
        if self.route_combo.count():
            self.route_combo.setCurrentIndex(selected_index)
        self.route_combo.blockSignals(False)
        self._route_options_signature = signature

    def apply_selected_route(self) -> None:
        path_id = self.route_combo.currentData()
        if path_id is None:
            return
        if self.sim.select_route_by_id(int(path_id)):
            self.update_display()
            self.refresh_route_options(force=True)

    def manual_replan(self) -> None:
        self.sim.replan(reason="MANUAL REPLAN")
        self.refresh_route_options(force=True)
        self.update_display()

    def apply_flight_conditions(self) -> None:
        """Apply current wind and battery controls to the live simulation."""
        message = self.sim.set_flight_conditions(
            self.wind_speed_spin.value(),
            self.wind_direction_spin.value(),
            self.battery_condition_spin.value(),
        )
        self.condition_feedback.setText(message)
        self.refresh_route_options(force=True)
        self.update_display()

    def rebuild_obstacle_index_options(self) -> None:
        """Populate the obstacle selector for the selected static/dynamic type."""
        if not hasattr(self, "obstacle_index_combo"):
            return
        kind = self.obstacle_type_combo.currentData()
        old_index = self.obstacle_index_combo.currentIndex()
        self.obstacle_index_combo.blockSignals(True)
        self.obstacle_index_combo.clear()
        if kind == "dynamic":
            count = len(self.sim.dynamic_obstacle_specs)
            for index in range(count):
                self.obstacle_index_combo.addItem(f"Dynamic D{index + 1}", index)
        else:
            count = len(self.sim.static_obstacles)
            for index in range(count):
                self.obstacle_index_combo.addItem(f"Static S{index + 1}", index)
        if count:
            self.obstacle_index_combo.setCurrentIndex(max(0, min(old_index, count - 1)))
        self.obstacle_index_combo.blockSignals(False)

    def obstacle_type_changed(self, _index: int | None = None) -> None:
        self.rebuild_obstacle_index_options()
        self.load_obstacle_controls()

    def _sync_dynamic_track_range(self, _index: int | None = None) -> None:
        """Keep a track's configured amplitude within the available arena span."""
        is_dynamic = self.obstacle_type_combo.currentData() == "dynamic"
        for widget in (
            self.motion_axis_label, self.motion_axis_combo,
            self.motion_range_label, self.motion_range_spin,
            self.motion_rate_label, self.motion_rate_spin,
        ):
            widget.setEnabled(is_dynamic)

    def load_obstacle_controls(self, _index: int | None = None) -> None:
        """Load position and dynamic motion settings for the selected obstacle."""
        if not hasattr(self, "obstacle_index_combo") or self.obstacle_index_combo.count() == 0:
            return
        kind = self.obstacle_type_combo.currentData()
        index = int(self.obstacle_index_combo.currentData())
        self._sync_dynamic_track_range()
        if kind == "dynamic":
            if not 0 <= index < len(self.sim.dynamic_obstacle_specs):
                return
            x, y, _hx, _hy, _height, axis, amplitude, rate, _phase = self.sim.dynamic_obstacle_specs[index]
            for widget in (self.obstacle_x_spin, self.obstacle_y_spin, self.motion_range_spin, self.motion_rate_spin):
                widget.blockSignals(True)
            self.obstacle_x_spin.setValue(float(x))
            self.obstacle_y_spin.setValue(float(y))
            self.motion_axis_combo.setCurrentIndex(0 if axis == "x" else 1)
            self.motion_range_spin.setValue(float(amplitude))
            self.motion_rate_spin.setValue(float(rate))
            for widget in (self.obstacle_x_spin, self.obstacle_y_spin, self.motion_range_spin, self.motion_rate_spin):
                widget.blockSignals(False)
            self.obstacle_edit_feedback.setText(
                f"D{index + 1}: track centre ({x:.2f}, {y:.2f}) m · {axis.upper()} sweep · range ±{amplitude:.2f} m · {rate:.2f} rad/s"
            )
        else:
            if not 0 <= index < len(self.sim.static_obstacles):
                return
            x, y, _hx, _hy, _height = self.sim.static_obstacles[index]
            self.obstacle_x_spin.blockSignals(True)
            self.obstacle_y_spin.blockSignals(True)
            self.obstacle_x_spin.setValue(float(x))
            self.obstacle_y_spin.setValue(float(y))
            self.obstacle_x_spin.blockSignals(False)
            self.obstacle_y_spin.blockSignals(False)
            self.obstacle_edit_feedback.setText(f"S{index + 1}: current position ({x:.2f}, {y:.2f}) m")

    def apply_obstacle_customization(self) -> None:
        """Apply a static position edit or a dynamic position + patrol edit."""
        kind = self.obstacle_type_combo.currentData()
        index = int(self.obstacle_index_combo.currentData())
        x, y = self.obstacle_x_spin.value(), self.obstacle_y_spin.value()
        if kind == "static":
            if not 0 <= index < len(self.sim.static_obstacles):
                self.obstacle_edit_feedback.setText("Select a static obstacle first.")
                return
            moved = self.sim.move_static_obstacle(index, x, y, replan=True)
            if moved:
                actual_x, actual_y = self.sim.static_obstacles[index][:2]
                self.obstacle_edit_feedback.setText(
                    f"S{index + 1} moved to ({actual_x:.2f}, {actual_y:.2f}) m; route replanned."
                )
            else:
                self.obstacle_edit_feedback.setText(
                    "Position unchanged or unsafe. Choose a free point away from the drone and goal."
                )
        else:
            if not 0 <= index < len(self.sim.dynamic_obstacle_specs):
                self.obstacle_edit_feedback.setText("Select a dynamic obstacle first.")
                return
            axis = self.motion_axis_combo.currentData()
            amplitude = self.motion_range_spin.value()
            rate = self.motion_rate_spin.value()
            changed = self.sim.configure_dynamic_obstacle(
                index, x, y, axis=axis, amplitude=amplitude,
                angular_rate=rate, replan=True,
            )
            if changed:
                sx, sy, _hx, _hy, _height, axis, amplitude, rate, _phase = self.sim.dynamic_obstacle_specs[index]
                self.obstacle_edit_feedback.setText(
                    f"D{index + 1}: track centre ({sx:.2f}, {sy:.2f}) m · {axis.upper()} sweep ±{amplitude:.2f} m at {rate:.2f} rad/s; route replanned."
                )
            else:
                self.obstacle_edit_feedback.setText(
                    "Track unchanged or unsafe. Move the track away from the drone/goal and keep the full sweep inside the arena."
                )
        feedback = self.obstacle_edit_feedback.text()
        self.refresh_route_options(force=True)
        self.top_view.update()
        self.update_display()
        self.load_obstacle_controls()
        self.obstacle_edit_feedback.setText(feedback)

    def static_obstacle_moved(self) -> None:
        """Refresh controls and route data after a red obstacle is dragged."""
        index = int(getattr(self.sim, "last_moved_static_index", 0))
        self.obstacle_type_combo.setCurrentIndex(0)
        if 0 <= index < self.obstacle_index_combo.count():
            self.obstacle_index_combo.setCurrentIndex(index)
        self.load_obstacle_controls()
        self.obstacle_edit_feedback.setText(f"S{index + 1} moved; route replanned.")
        self.refresh_route_options(force=True)
        self.top_view.update()
        self.update_display()

    def dynamic_obstacle_moved(self, index: int) -> None:
        """Refresh controls and route data after an orange obstacle is dragged."""
        self.obstacle_type_combo.setCurrentIndex(1)
        if 0 <= int(index) < self.obstacle_index_combo.count():
            self.obstacle_index_combo.setCurrentIndex(int(index))
        self.load_obstacle_controls()
        self.obstacle_edit_feedback.setText(f"D{int(index) + 1} repositioned; patrol preserved and route replanned.")
        self.refresh_route_options(force=True)
        self.top_view.update()
        self.update_display()

    def destination_moved(self) -> None:
        """Refresh the destination display and route options after dragging the goal."""
        self.mission.set("Target", str(tuple(round(v, 2) for v in self.sim.destination)) + " m")
        self.refresh_route_options(force=True)
        self.top_view.update()
        self.update_display()

    def obstacle_options_changed(self, _checked: bool = False) -> None:
        self.sim.set_obstacle_modes(
            static_enabled=self.static_toggle.isChecked(),
            dynamic_enabled=self.dynamic_toggle.isChecked(),
        )
        self.refresh_route_options(force=True)
        self.update_display()

    def tick(self) -> None:
        if self.sim.running:
            self.sim.step()
        self.update_display()

    def update_display(self) -> None:
        p = np.asarray(self.sim.pos, dtype=float)
        v = np.asarray(self.sim.vel, dtype=float)
        distance = float(self.sim.distance_to_target)
        progress = float(self.sim.mission_progress)
        nearest = float(self.sim.nearest_obstacle)
        risk = float(self.sim.risk)
        waypoint_count = len(self.sim.waypoints)

        self.clock.setText(f"Sim Time: {self.sim.sim_time:.1f} s")
        self.status.set("Time", f"{self.sim.sim_time:.1f} s")
        self.status.set("Decision", getattr(self.sim, "decision", "FORWARD"))
        self.status.set("Context", self.sim.context)
        self.status.set("Mode", self.sim.phase)
        self.status.set("Simulation", self.sim.status)

        self.state.set("Position", f"[{p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}] m")
        self.state.set("Velocity", f"[{v[0]:.2f}, {v[1]:.2f}, {v[2]:.2f}] m/s")
        self.state.set("Speed", f"{float(np.linalg.norm(v)):.2f} m/s")
        self.state.set("Altitude", f"{p[2]:.2f} m")
        self.gps_note.setText(
            f"GPS SIM  {self.sim.gps_latitude:.6f}°, {self.sim.gps_longitude:.6f}°  |  Alt {self.sim.gps_altitude:.2f} m"
        )

        self.mission.set("Target", str(tuple(round(v, 2) for v in self.sim.destination)) + " m")
        self.mission.set("Distance", f"{distance:.2f} m")
        self.mission.set("Progress", f"{progress:.1f} %")
        self.mission.set_progress("Progress", progress)
        self.mission.set("Waypoint", f"{min(self.sim.wp_index + 1, waypoint_count)} / {waypoint_count}")

        if risk >= 0.65:
            risk_level, safety_status = "HIGH", "CAUTION"
        elif risk >= 0.35:
            risk_level, safety_status = "MEDIUM", "MONITOR"
        else:
            risk_level, safety_status = "LOW", "SAFE"
        self.safety.set("Nearest obstacle", f"{nearest:.2f} m")
        self.safety.set("Risk level", risk_level)
        self.safety.set("Obstacle risk", f"{risk:.2f}")
        self.safety.set("Status", safety_status if self.sim.status == "RUNNING" else self.sim.status)

        self.energy.set("Battery", f"{self.sim.battery:.1f} %")
        self.energy.set_progress("Battery", self.sim.battery)
        self.controls.set("Force X", f"{self.sim.force_x:.2f} N")
        self.controls.set("Force Y", f"{self.sim.force_y:.2f} N")
        self.controls.set("Thrust Z", f"{self.sim.thrust_z:.2f} N")
        self.controls.set("Control magnitude", f"{self.sim.control_magnitude:.2f} N")

        self.weight_panel.set_weights(self.sim.weights)
        if self.sim.status == "MISSION COMPLETE":
            self.restart_button.setText("START AGAIN ↻")
        elif self.sim.status == "LOW BATTERY":
            self.restart_button.setText("RESTART MISSION ↻")
        else:
            self.restart_button.setText("RESTART MISSION ↻")
        self.apply_route_button.setEnabled(bool(self.sim.path_ranking) and self.sim.running)
        self.replan_button.setEnabled(self.sim.running)
        self.refresh_route_options()

        self.distance_plot.set_data(self.sim.history_time, self.sim.distance_history)
        self.battery_plot.set_data(self.sim.history_time, self.sim.battery_history)
        self.speed_plot.set_data(self.sim.history_time, self.sim.speed_history)
        if hasattr(self, "condition_feedback"):
            wind_speed = float(self.sim.wind_speed_mps)
            wind_label = "CALM" if wind_speed < 2.0 else ("BREEZY" if wind_speed < 6.0 else ("STRONG WIND" if wind_speed < 9.0 else "SEVERE WIND"))
            self.condition_feedback.setText(
                f"{wind_label} · {wind_speed:.1f} m/s @ {self.sim.wind_direction_deg:.0f}° · "
                f"battery {self.sim.battery:.1f}% · {self.sim.decision}"
            )
        self.view.update()
        self.top_view.update()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API
        self.timer.stop()
        self.view.close_renderer()
        event.accept()


def run(sim) -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    window = Dashboard(sim)
    window.showMaximized()
    return app.exec()
