"""Per-step flight logging and evaluation metric summaries.

The logger records the *actual* state, ACMFDS output, and commanded action
every control step — nothing is synthesized after the fact. Metrics are
computed from these records, never invented.
"""

import csv
import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from state.flight_state import FlightState
from decision.acmfds import Decision, FlightCommand

CSV_COLUMNS = [
    "timestamp", "episode", "step", "mission_phase",
    "position_x", "position_y", "altitude", "target_altitude",
    "velocity_x", "velocity_y", "velocity_z", "horizontal_speed",
    "roll", "pitch", "yaw", "goal_distance",
    "obstacle_distance", "clearance_front", "clearance_left", "clearance_right",
    "moving_obstacles",
    "battery_percentage",
    "wind_speed", "wind_direction_x", "wind_direction_y", "wind_direction_z",
    "acmfds_context", "acmfds_decision", "decision_reason",
    "selected_direction", "movement_action", "candidates",
    "command_forward_speed", "command_lateral_speed", "command_target_altitude",
    "thrust_n",
    "collision", "mission_completed",
]


def _round_native(value: Any, digits: int) -> float:
    """round() that always returns a native Python float.

    Sensor values (ray distances, velocities, poses) arrive as numpy scalars;
    numpy types survive round() and then break JSON serialization and pydantic
    responses, so every logged number is converted at the source.
    """
    return round(float(value), digits)


class FlightLogger:
    """Writes one CSV row per control step."""

    def __init__(self, filepath: str, episode: int = 1) -> None:
        self.filepath = filepath
        self.episode = episode
        self._rows: List[Dict[str, Any]] = []
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)

    def log(
        self,
        timestamp: float,
        step: int,
        state: FlightState,
        decision: Decision,
        command: FlightCommand,
        thrust_n: float,
        collision: bool,
        mission_completed: bool,
        mission_phase: str = "",
        moving_obstacle_positions: Optional[List[Any]] = None,
    ) -> Dict[str, Any]:
        moving_str = ""
        if moving_obstacle_positions:
            moving_str = ";".join(
                f"{float(p[0]):.3f},{float(p[1]):.3f},{float(p[2]):.3f}"
                for p in moving_obstacle_positions
            )
        row: Dict[str, Any] = {
            "timestamp": _round_native(timestamp, 4),
            "episode": self.episode,
            "step": step,
            "mission_phase": mission_phase,
            "position_x": _round_native(state.position_x, 4),
            "position_y": _round_native(state.position_y, 4),
            "altitude": _round_native(state.altitude, 4),
            "target_altitude": _round_native(state.target_altitude, 4),
            "velocity_x": _round_native(state.velocity_x, 4),
            "velocity_y": _round_native(state.velocity_y, 4),
            "velocity_z": _round_native(state.velocity_z, 4),
            "horizontal_speed": _round_native(state.horizontal_speed, 4),
            "roll": _round_native(state.roll, 4),
            "pitch": _round_native(state.pitch, 4),
            "yaw": _round_native(state.yaw, 4),
            "goal_distance": _round_native(state.goal_distance, 3),
            "obstacle_distance": _round_native(state.obstacle_distance, 4),
            "clearance_front": _round_native(state.clearance_front, 4),
            "clearance_left": _round_native(state.clearance_left, 4),
            "clearance_right": _round_native(state.clearance_right, 4),
            "moving_obstacles": moving_str,
            "battery_percentage": _round_native(state.battery_percentage, 2),
            "wind_speed": _round_native(state.wind_speed, 3),
            "wind_direction_x": _round_native(state.wind_direction_x, 3),
            "wind_direction_y": _round_native(state.wind_direction_y, 3),
            "wind_direction_z": _round_native(state.wind_direction_z, 3),
            "acmfds_context": decision.context.value,
            "acmfds_decision": decision.decision.value,
            "decision_reason": decision.reason,
            "selected_direction": decision.selected_direction or "",
            # The action actually in force: the directional pick when ACMFDS
            # selects one (LEFT/RIGHT/CLIMB), else the decision itself
            # (MOVE_FORWARD / HOLD / SAFE_LANDING / ...). Never empty during
            # flight — this is what dashboards should display as the
            # direction/action. Same fallback the mission runner uses.
            "movement_action": decision.selected_direction or decision.decision.value,
            "candidates": decision.candidates_summary(),
            "command_forward_speed": _round_native(command.forward_speed, 3),
            "command_lateral_speed": _round_native(command.lateral_speed, 3),
            "command_target_altitude": _round_native(command.target_altitude, 3),
            "thrust_n": _round_native(thrust_n, 3),
            "collision": int(collision),
            "mission_completed": int(mission_completed),
        }
        self._rows.append(row)
        return row

    @property
    def rows(self) -> List[Dict[str, Any]]:
        return self._rows

    def flush(self) -> str:
        """Write all rows to disk and return the path."""
        with open(self.filepath, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            writer.writerows(self._rows)
        return self.filepath


@dataclass
class EpisodeSummary:
    """Computed metrics for one episode/run."""

    outcome: str = ""                # landed | collision | timeout | emergency | battery_empty
    mission_completed: bool = False
    landed: bool = False             # touchdown verified (outcome == "landed")
    duration_s: float = 0.0
    steps: int = 0
    min_obstacle_distance: float = math.inf
    mean_altitude_error: float = 0.0
    max_altitude_error: float = 0.0
    battery_consumed_percent: float = 0.0
    battery_end_percent: float = 100.0
    final_altitude: float = 0.0
    final_vertical_speed: float = 0.0
    collision_count: int = 0
    emergency_events: int = 0        # distinct emergency episodes (entries)
    emergency_steps: int = 0         # control steps spent in EMERGENCY decisions
    safe_landing_events: int = 0     # distinct SAFE_LANDING episodes (entries)
    safe_landing_steps: int = 0      # control steps with SAFE_LANDING decisions
    wind_mean: float = 0.0
    wind_max: float = 0.0
    decisions: Dict[str, int] = field(default_factory=dict)
    contexts: Dict[str, int] = field(default_factory=dict)


def compute_summary(rows: List[Dict[str, Any]], outcome: str) -> EpisodeSummary:
    """Derive summary metrics from logged rows (all real, nothing invented).

    Emergency / safe-landing metrics are counted two ways and reported
    separately: ``*_events`` counts distinct episodes (a transition into the
    decision after a different decision), ``*_steps`` counts the per-step
    decision frequency. ``landed`` is true only when the run ended with a
    sequencer-confirmed touchdown (outcome ``"landed"``).
    """
    summary = EpisodeSummary(outcome=outcome)
    if not rows:
        return summary

    summary.steps = len(rows)
    summary.duration_s = rows[-1]["timestamp"] - rows[0]["timestamp"]
    # The final logged row precedes the mission-finished flip, so derive
    # completion from the outcome as well.
    summary.mission_completed = bool(rows[-1]["mission_completed"]) or outcome == "landed"
    summary.landed = outcome == "landed"
    summary.collision_count = sum(int(r["collision"]) for r in rows)
    summary.min_obstacle_distance = min(
        r["obstacle_distance"] for r in rows if r["obstacle_distance"] is not None
    )
    alt_errors = [abs(r["target_altitude"] - r["altitude"]) for r in rows]
    summary.mean_altitude_error = sum(alt_errors) / len(alt_errors)
    summary.max_altitude_error = max(alt_errors)
    summary.battery_end_percent = rows[-1]["battery_percentage"]
    summary.battery_consumed_percent = rows[0]["battery_percentage"] - summary.battery_end_percent
    summary.final_altitude = rows[-1]["altitude"]
    summary.final_vertical_speed = rows[-1]["velocity_z"]
    wind = [r["wind_speed"] for r in rows]
    summary.wind_mean = sum(wind) / len(wind)
    summary.wind_max = max(wind)

    prev_decision = None
    for r in rows:
        d = r["acmfds_decision"]
        summary.decisions[d] = summary.decisions.get(d, 0) + 1
        c = r["acmfds_context"]
        summary.contexts[c] = summary.contexts.get(c, 0) + 1
        if d == "EMERGENCY":
            summary.emergency_steps += 1
            if prev_decision != "EMERGENCY":
                summary.emergency_events += 1
        if d == "SAFE_LANDING":
            summary.safe_landing_steps += 1
            if prev_decision != "SAFE_LANDING":
                summary.safe_landing_events += 1
        prev_decision = d
    return summary


def format_summary(summary: EpisodeSummary) -> str:
    """Human-readable summary block."""
    landed_txt = (
        f"YES (final alt {summary.final_altitude:.2f} m, "
        f"vz {summary.final_vertical_speed:+.2f} m/s)"
        if summary.landed else "NO"
    )
    lines = [
        "=" * 60,
        "EPISODE SUMMARY",
        "=" * 60,
        f"Outcome                 : {summary.outcome}"
        + (" (touchdown confirmed)" if summary.landed else ""),
        f"Landed                  : {landed_txt}",
        f"Mission completed       : {summary.mission_completed}",
        f"Duration / steps        : {summary.duration_s:6.2f}s / {summary.steps}",
        f"Min obstacle distance   : {summary.min_obstacle_distance:6.3f} m",
        f"Altitude error (mean)   : {summary.mean_altitude_error:6.3f} m",
        f"Altitude error (max)    : {summary.max_altitude_error:6.3f} m",
        f"Battery consumed        : {summary.battery_consumed_percent:6.2f} %",
        f"Battery end             : {summary.battery_end_percent:6.2f} %",
        f"Collisions              : {summary.collision_count}",
        f"Emergency entries/steps : {summary.emergency_events} / {summary.emergency_steps}",
        f"Safe-land entries/steps : {summary.safe_landing_events} / {summary.safe_landing_steps}",
        f"Wind mean / max         : {summary.wind_mean:5.2f} / {summary.wind_max:5.2f} m/s",
        "Decision distribution   : "
        + ", ".join(f"{k}={v}" for k, v in sorted(summary.decisions.items())),
        "=" * 60,
    ]
    return "\n".join(lines)
