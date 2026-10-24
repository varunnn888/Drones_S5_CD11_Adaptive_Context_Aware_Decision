"""ACMFDS — Adaptive Context-Aware Multi-Objective Flight Decision System.

Naming note (documented discrepancy): the original README and code expanded
ACMFDS as "Adaptive Collision Mitigation and Flight Decision System". The
project title standardizes on **Adaptive Context-Aware Multi-Objective
Flight Decision System**; that expansion is used consistently from this
revision onward.

ACMFDS is the central, transparent decision layer of the architecture::

    PHYSICS SIMULATION -> LIVE FLIGHT STATE -> ACMFDS DECISION
        -> FLIGHT COMMAND -> ACTUATION -> QUADCOPTER DYNAMICS

Design properties:
* Rule-based context classifier + explicit objective weights. No black-box
  RL policy, no PID controller.
* Consumes the full validated :class:`~state.flight_state.FlightState`.
* Produces an explainable :class:`Decision` (context, decision, reason,
  active constraints, objective weights, and a recommended
  :class:`FlightCommand`) — never just a string.
* Priority handling for conflicting factors is explicit and documented:

    =========================================   ======================================
    Condition                                   Behavior
    =========================================   ======================================
    Normal, obstacle far, adequate battery      Continue mission (MOVE_FORWARD / HOLD)
    Obstacle close ahead                        AVOID_OBSTACLE toward the clearer side
    Battery low                                 ENERGY_CONSERVATION; SAFE_LANDING if
                                                battery is critical
    Strong wind                                 WIND_RESPONSE (reduced speed)
    Obstacle close and battery low              Collision safety wins; feasible
                                                lateral maneuver at low speed
    Critical collision risk / extreme tilt /    EMERGENCY: freeze horizontal motion,
    battery <= emergency level                  hold/regain safe altitude
    =========================================   ======================================
"""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional

from state.flight_state import FlightState


class FlightContext(str, Enum):
    """Dominant operational context (highest-priority active condition)."""

    NORMAL = "NORMAL"
    OBSTACLE_NEAR = "OBSTACLE_NEAR"
    OBSTACLE_CRITICAL = "OBSTACLE_CRITICAL"
    LOW_BATTERY = "LOW_BATTERY"
    STRONG_WIND = "STRONG_WIND"
    LANDING = "LANDING"
    EMERGENCY = "EMERGENCY"
    MISSION_COMPLETE = "MISSION_COMPLETE"


class DecisionType(str, Enum):
    """Discrete flight decisions ACMFDS can issue."""

    TAKEOFF = "TAKEOFF"
    CLIMB = "CLIMB"
    DESCEND = "DESCEND"
    HOLD = "HOLD"
    MOVE_FORWARD = "MOVE_FORWARD"
    AVOID_OBSTACLE = "AVOID_OBSTACLE"
    WIND_RESPONSE = "WIND_RESPONSE"
    ENERGY_CONSERVATION = "ENERGY_CONSERVATION"
    SAFE_LANDING = "SAFE_LANDING"
    EMERGENCY = "EMERGENCY"
    MISSION_COMPLETE = "MISSION_COMPLETE"


class Objective(str, Enum):
    """The four central optimization objectives."""

    SAFETY = "SAFETY"
    ENERGY_EFFICIENCY = "ENERGY_EFFICIENCY"
    OBSTACLE_CLEARANCE = "OBSTACLE_CLEARANCE"
    MISSION_PROGRESS = "MISSION_PROGRESS"


@dataclass
class FlightCommand:
    """Executable-level command recommended by ACMFDS.

    This is the explicit bridge between a *decision* and the actuation
    layer — a decision such as ``MOVE_FORWARD`` is never a motor RPM.

    Units: speeds in m/s (body frame: +X forward, +Y left), altitudes in m,
    heading in radians (world frame, CCW from +X).
    """

    target_altitude: float = 2.0
    forward_speed: float = 0.0      # m/s along body +X
    lateral_speed: float = 0.0      # m/s along body +Y
    heading: Optional[float] = None  # desired yaw, rad; None = keep current
    speed_limit: float = 1.2        # hard cap for the actuation layer, m/s


@dataclass
class AvoidanceCandidate:
    """One evaluated response to a detected obstacle (LEFT/RIGHT/FORWARD/CLIMB).

    ``clearance`` is the metric the candidate is judged on:
    forward-corridor / side clearance in meters, or vertical margin (m) for
    CLIMB. ``safe`` is the accept/reject verdict; ``detail`` is the explicit
    human-readable reason shown in the HUD, dashboard, and logs.
    """

    direction: str          # FORWARD | LEFT | RIGHT | CLIMB
    clearance: float        # m (corridor / side clearance, or climb margin)
    safe: bool
    detail: str


@dataclass
class Decision:
    """Explainable ACMFDS output."""

    context: FlightContext
    decision: DecisionType
    reason: str
    constraints: List[str] = field(default_factory=list)
    weights: Dict[str, float] = field(default_factory=dict)
    command: FlightCommand = field(default_factory=FlightCommand)
    candidates: List[AvoidanceCandidate] = field(default_factory=list)
    selected_direction: Optional[str] = None

    def candidates_summary(self) -> str:
        """Compact one-line summary of all evaluated candidates (for logs)."""
        return " | ".join(
            f"{c.direction}:{c.clearance:.2f}m:{'safe' if c.safe else 'rejected'}"
            for c in self.candidates
        )


@dataclass
class ACMFDSConfig:
    """All ACMFDS thresholds in one place (SI units)."""

    altitude_tolerance: float = 0.15      # m
    obstacle_near: float = 2.0            # m: begin caution
    obstacle_avoid: float = 1.5           # m: active avoidance
    obstacle_critical: float = 0.8        # m: critical collision risk
    battery_low: float = 30.0             # %
    battery_critical: float = 15.0        # %
    battery_emergency: float = 5.0        # %
    wind_strong: float = 4.0              # m/s
    wind_severe: float = 7.0              # m/s
    goal_reached_radius: float = 0.4      # m
    min_safe_altitude: float = 0.25       # m above ground (excl. landing)
    max_safe_altitude: float = 4.5        # m (ceiling for climb-over)
    max_safe_tilt: float = 0.55           # rad (~31 deg) before emergency
    cruise_speed: float = 1.2             # m/s normal forward speed
    cautious_speed: float = 0.6           # m/s in caution contexts
    conserve_speed: float = 0.5           # m/s under energy conservation
    climb_rate: float = 0.9               # m/s commanded climb
    descend_rate: float = -0.7            # m/s commanded descent
    climb_margin: float = 0.5             # m: clearance flown above a detected obstacle top
    safety_margin: float = 0.3            # m: extra clearance required to accept a side detour
                                          #    (gate = obstacle_critical + safety_margin);
                                          #    flown maneuvers closer than this are close passes
    maneuver_cooldown: float = 1.0        # s: cautious-speed caution after a maneuver closes
                                          #    (body-frame rays can lose an obstacle mid-heading-change)


# Objective weights per context (each sums to 1.0).
CONTEXT_WEIGHTS: Dict[FlightContext, Dict[Objective, float]] = {
    FlightContext.NORMAL: {
        Objective.SAFETY: 0.25,
        Objective.ENERGY_EFFICIENCY: 0.20,
        Objective.OBSTACLE_CLEARANCE: 0.20,
        Objective.MISSION_PROGRESS: 0.35,
    },
    FlightContext.OBSTACLE_NEAR: {
        Objective.SAFETY: 0.35,
        Objective.ENERGY_EFFICIENCY: 0.10,
        Objective.OBSTACLE_CLEARANCE: 0.45,
        Objective.MISSION_PROGRESS: 0.10,
    },
    FlightContext.OBSTACLE_CRITICAL: {
        Objective.SAFETY: 0.50,
        Objective.ENERGY_EFFICIENCY: 0.05,
        Objective.OBSTACLE_CLEARANCE: 0.40,
        Objective.MISSION_PROGRESS: 0.05,
    },
    FlightContext.LOW_BATTERY: {
        Objective.SAFETY: 0.35,
        Objective.ENERGY_EFFICIENCY: 0.45,
        Objective.OBSTACLE_CLEARANCE: 0.10,
        Objective.MISSION_PROGRESS: 0.10,
    },
    FlightContext.STRONG_WIND: {
        Objective.SAFETY: 0.40,
        Objective.ENERGY_EFFICIENCY: 0.15,
        Objective.OBSTACLE_CLEARANCE: 0.15,
        Objective.MISSION_PROGRESS: 0.30,
    },
    FlightContext.LANDING: {
        Objective.SAFETY: 0.55,
        Objective.ENERGY_EFFICIENCY: 0.15,
        Objective.OBSTACLE_CLEARANCE: 0.15,
        Objective.MISSION_PROGRESS: 0.15,
    },
    FlightContext.EMERGENCY: {
        Objective.SAFETY: 0.75,
        Objective.ENERGY_EFFICIENCY: 0.10,
        Objective.OBSTACLE_CLEARANCE: 0.10,
        Objective.MISSION_PROGRESS: 0.05,
    },
    FlightContext.MISSION_COMPLETE: {
        Objective.SAFETY: 0.40,
        Objective.ENERGY_EFFICIENCY: 0.30,
        Objective.OBSTACLE_CLEARANCE: 0.15,
        Objective.MISSION_PROGRESS: 0.15,
    },
}


class ACMFDS:
    """Adaptive Context-Aware Multi-Objective Flight Decision System."""

    def __init__(self, config: Optional[ACMFDSConfig] = None) -> None:
        self.config = config or ACMFDSConfig()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def decide(self, state: FlightState) -> Decision:
        """Evaluate the full flight state and produce an explainable decision."""
        cfg = self.config
        factors: List[str] = []  # active factor descriptions for the reason

        tilt = max(abs(state.roll), abs(state.pitch))
        # Full directional evaluation (LEFT/RIGHT/FORWARD/CLIMB) from live
        # sensing; attached to every Decision so HUD, dashboard, and logs
        # show the alternatives and why they were accepted or rejected.
        candidates = self._evaluate_candidates(state)

        # The exact sensor reading that triggers obstacle conditions:
        # the nearest forward-cone ray, named explicitly in every reason.
        nearest_ray, nearest_val = min(
            (
                ("FRONT", state.clearance_front),
                ("LEFT", state.clearance_left),
                ("RIGHT", state.clearance_right),
            ),
            key=lambda kv: kv[1],
        )

        # ---- Emergency checks (highest priority) -----------------------
        if state.obstacle_distance <= cfg.obstacle_critical:
            side_gate = cfg.obstacle_critical + cfg.safety_margin
            side_clear = max(state.clearance_left, state.clearance_right)
            if side_clear >= side_gate:
                side = "left" if state.clearance_left >= state.clearance_right else "right"
                detail = (
                    f"{nearest_ray} ray {nearest_val:.2f}m <= critical "
                    f"{cfg.obstacle_critical:.2f}m: escaping away from "
                    f"{nearest_ray}, biased {side} (gate critical+margin "
                    f"{side_gate:.2f}m; clearance "
                    f"L{state.clearance_left:.2f}/R{state.clearance_right:.2f}m)"
                )
                constraints = ["retreat_from_obstacle", f"detour_{side}", f"max_speed={cfg.cautious_speed}"]
                selected: Optional[str] = side.upper()
            elif state.altitude < cfg.max_safe_altitude - 0.5:
                detail = (
                    f"{nearest_ray} ray {nearest_val:.2f}m <= critical "
                    f"{cfg.obstacle_critical:.2f}m and both sides under the "
                    f"{side_gate:.2f}m gate: climbing over"
                )
                constraints = ["climb_above_obstacle", "no_forward_motion"]
                selected = "CLIMB"
            else:
                detail = (
                    f"{nearest_ray} ray {nearest_val:.2f}m <= critical "
                    f"{cfg.obstacle_critical:.2f}m; wedged at ceiling: holding"
                )
                constraints = ["freeze_horizontal_motion", "hold_safe_altitude"]
                selected = None
            rejected = self._rejected_summary(candidates, selected)
            if rejected:
                detail += f"; rejected: {rejected}"
            return self._decision(
                state, FlightContext.EMERGENCY, DecisionType.EMERGENCY,
                factors, detail, constraints,
                candidates=candidates, selected=selected,
            )
        if tilt >= cfg.max_safe_tilt:
            return self._decision(
                state, FlightContext.EMERGENCY, DecisionType.EMERGENCY,
                factors,
                f"tilt {tilt:.2f}rad exceeds safe limit {cfg.max_safe_tilt}rad",
                ["level_attitude_first", "freeze_horizontal_motion"],
            )
        if state.battery_percentage <= cfg.battery_emergency:
            return self._decision(
                state, FlightContext.EMERGENCY, DecisionType.SAFE_LANDING,
                factors,
                f"battery {state.battery_percentage:.1f}% at emergency level",
                ["land_now", "cut_forward_motion"],
            )
        if state.altitude <= cfg.min_safe_altitude and state.target_altitude > cfg.min_safe_altitude:
            # Drone is skimming the ground while it should be airborne.
            return self._decision(
                state, FlightContext.EMERGENCY, DecisionType.CLIMB,
                factors,
                f"altitude {state.altitude:.2f}m near ground while target is {state.target_altitude:.2f}m",
                ["climb_immediately", "no_forward_motion"],
            )

        # ---- Battery ----------------------------------------------------
        battery_low = state.battery_percentage <= cfg.battery_low
        battery_critical = state.battery_percentage <= cfg.battery_critical
        if battery_low:
            factors.append(f"battery {state.battery_percentage:.1f}% <= low {cfg.battery_low}%")

        # ---- Obstacles --------------------------------------------------
        obstacle_near = state.obstacle_distance <= cfg.obstacle_near
        obstacle_avoid = state.obstacle_distance <= cfg.obstacle_avoid
        if obstacle_near:
            factors.append(
                f"obstacle {nearest_ray} ray {nearest_val:.2f}m <= near {cfg.obstacle_near:.2f}m"
            )

        # ---- Wind -------------------------------------------------------
        wind_strong = state.wind_speed >= cfg.wind_strong
        if wind_strong:
            factors.append(f"wind {state.wind_speed:.1f}m/s >= strong {cfg.wind_strong}m/s")

        # ---- Goal / mission ---------------------------------------------
        goal_far = state.goal_distance > cfg.goal_reached_radius
        at_target_altitude = abs(state.altitude_error) <= cfg.altitude_tolerance
        # Mission phase (not goal distance) classifies the landing/complete
        # contexts: reaching the goal *begins* landing; touchdown (phase
        # DONE) *completes* the mission.
        phase = (getattr(state, "mission_phase", "") or "").upper()

        # ---- Composite context selection (priority order) ---------------
        if battery_critical:
            context = FlightContext.LOW_BATTERY
        elif obstacle_avoid:
            context = (
                FlightContext.OBSTACLE_CRITICAL
                if state.obstacle_distance <= cfg.obstacle_critical
                else FlightContext.OBSTACLE_NEAR
            )
        elif battery_low:
            context = FlightContext.LOW_BATTERY
        elif wind_strong:
            context = FlightContext.STRONG_WIND
        elif phase == "LAND":
            context = FlightContext.LANDING
        elif phase == "DONE":
            context = FlightContext.MISSION_COMPLETE
        else:
            context = FlightContext.NORMAL

        # ---- Decision logic per context ---------------------------------
        selected: Optional[str] = None
        if context == FlightContext.LOW_BATTERY and battery_critical:
            decision, extra = DecisionType.SAFE_LANDING, "battery critical: landing in place"
            constraints = ["forward_speed=0", "descend_to_ground"]
        elif obstacle_avoid:
            # Collision safety wins over energy (documented priority rule).
            # Selection among the evaluated candidates (safety thresholds):
            #  1. a side detour whose clearance >= critical + safety_margin
            #     (prefer the clearer side)
            #  2. otherwise climb over the obstacle top if within ceiling
            #  3. otherwise hold position (freeze) — never continue forward.
            _fwd, left_c, right_c, climb_c = candidates
            if left_c.safe or right_c.safe:
                if left_c.safe and (not right_c.safe or left_c.clearance >= right_c.clearance):
                    side, selected = "left", "LEFT"
                else:
                    side, selected = "right", "RIGHT"
                decision = DecisionType.AVOID_OBSTACLE
                gate = cfg.obstacle_critical + cfg.safety_margin
                extra = (
                    f"obstacle inside {cfg.obstacle_avoid:.1f}m: detouring {side} "
                    f"(clearance L {state.clearance_left:.2f}/R {state.clearance_right:.2f}m, "
                    f"gate {gate:.2f}m)"
                )
                constraints = [f"max_forward_speed={cfg.cautious_speed}", f"detour_{side}"]
            elif climb_c.safe:
                decision, selected = DecisionType.CLIMB, "CLIMB"
                extra = f"both detour sides blocked by {cfg.obstacle_critical:.1f}m gate: climbing over ({climb_c.detail})"
                constraints = ["climb_above_obstacle", "no_forward_motion"]
            else:
                decision = DecisionType.HOLD
                extra = "detour sides and climb path all unsafe: holding position"
                constraints = ["freeze_horizontal_motion"]
            rejected = self._rejected_summary(candidates, selected)
            if rejected:
                extra += f"; rejected: {rejected}"
        elif battery_low:
            decision = DecisionType.ENERGY_CONSERVATION
            extra = "reducing speed and climb activity to conserve energy"
            constraints = [f"max_forward_speed={cfg.conserve_speed}", "minimize_climb"]
        elif wind_strong:
            decision = DecisionType.WIND_RESPONSE
            extra = "strong wind: reduced speed, drift-compensated heading"
            constraints = [f"max_forward_speed={cfg.cautious_speed}", "wind_drift_compensation"]
        elif context == FlightContext.LANDING:
            # Reaching the goal begins landing (mission phase LAND);
            # touchdown is confirmed by the sequencer (phase DONE).
            decision = DecisionType.DESCEND
            extra = (
                f"mission phase LAND: descending to touchdown "
                f"(alt {state.altitude:.2f}m, vz {state.velocity_z:+.2f}m/s)"
            )
            constraints = ["descend_to_touchdown", "no_forward_motion"]
        elif context == FlightContext.MISSION_COMPLETE:
            decision, extra = DecisionType.MISSION_COMPLETE, "mission phase DONE: touchdown confirmed"
            constraints = ["hold_position"]
        else:
            # NORMAL context: plain mission logic.
            if not at_target_altitude:
                if state.altitude_error > 0:
                    decision, extra = DecisionType.CLIMB, f"altitude {state.altitude:.2f}m below target {state.target_altitude:.2f}m"
                    constraints = [f"climb_rate<={cfg.climb_rate}"]
                else:
                    decision, extra = DecisionType.DESCEND, f"altitude {state.altitude:.2f}m above target {state.target_altitude:.2f}m"
                    constraints = [f"descend_rate>={cfg.descend_rate}"]
            elif state.horizontal_speed > 0.1 or goal_far:
                decision = DecisionType.MOVE_FORWARD
                if state.maneuver_cooldown_s > 0.0:
                    extra = (
                        f"post-maneuver caution {state.maneuver_cooldown_s:.1f}s left: "
                        f"obstacle recently inside {cfg.obstacle_avoid:.2f}m, "
                        f"holding cautious speed"
                    )
                    constraints = [
                        f"max_forward_speed={cfg.cautious_speed}",
                        "post_maneuver_caution",
                    ]
                else:
                    extra = f"cruising to goal {state.goal_distance:.2f}m away"
                    constraints = [f"max_forward_speed={cfg.cruise_speed}"]
                selected = "FORWARD"
            else:
                decision, extra = DecisionType.HOLD, "on target altitude and stationary"
                constraints = ["hold_position"]

        return self._decision(state, context, decision, factors, extra, constraints,
                              candidates=candidates, selected=selected)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _evaluate_candidates(self, state: FlightState) -> List[AvoidanceCandidate]:
        """Evaluate LEFT / RIGHT / FORWARD / CLIMB responses from live sensing.

        * FORWARD is judged on the minimum clearance across the forward
          corridor (front ray + both diagonals) versus ``obstacle_avoid``.
        * LEFT / RIGHT on their side-ray clearance versus
          ``obstacle_critical + safety_margin`` — a detour is accepted only
          with margin over the critical band, so maneuvers never squeeze
          through nearly-blocked gaps.
        * CLIMB on the detected obstacle top: target altitude
          (top + ``climb_margin``) must stay under ``max_safe_altitude`` and
          the overhead sensor must confirm the vertical path is clear.
        """
        cfg = self.config
        corridor = min(state.clearance_front, state.clearance_left, state.clearance_right)

        fwd_safe = corridor > cfg.obstacle_avoid
        forward = AvoidanceCandidate(
            "FORWARD", corridor, fwd_safe,
            (
                f"forward corridor {corridor:.2f}m clear (> {cfg.obstacle_avoid:.2f}m)"
                if fwd_safe
                else f"forward corridor {corridor:.2f}m <= {cfg.obstacle_avoid:.2f}m required"
            ),
        )

        side_gate = cfg.obstacle_critical + cfg.safety_margin
        left_safe = state.clearance_left >= side_gate
        left = AvoidanceCandidate(
            "LEFT", state.clearance_left, left_safe,
            (
                f"left clearance {state.clearance_left:.2f}m >= gate {side_gate:.2f}m "
                f"(critical {cfg.obstacle_critical:.2f} + margin {cfg.safety_margin:.2f})"
                if left_safe
                else f"left clearance {state.clearance_left:.2f}m < gate {side_gate:.2f}m "
                     f"(critical {cfg.obstacle_critical:.2f} + margin {cfg.safety_margin:.2f})"
            ),
        )

        right_safe = state.clearance_right >= side_gate
        right = AvoidanceCandidate(
            "RIGHT", state.clearance_right, right_safe,
            (
                f"right clearance {state.clearance_right:.2f}m >= gate {side_gate:.2f}m "
                f"(critical {cfg.obstacle_critical:.2f} + margin {cfg.safety_margin:.2f})"
                if right_safe
                else f"right clearance {state.clearance_right:.2f}m < gate {side_gate:.2f}m "
                     f"(critical {cfg.obstacle_critical:.2f} + margin {cfg.safety_margin:.2f})"
            ),
        )

        top = state.obstacle_top
        if top is None:
            climb = AvoidanceCandidate(
                "CLIMB", 0.0, False,
                "no obstacle face on the front ray to climb over",
            )
        else:
            target_alt = top + cfg.climb_margin
            overhead_need = max(0.0, target_alt - state.altitude)
            margin = min(cfg.max_safe_altitude - target_alt, state.clearance_up - overhead_need)
            ceiling_ok = target_alt <= cfg.max_safe_altitude
            overhead_ok = state.clearance_up >= overhead_need
            if ceiling_ok and overhead_ok:
                detail = f"obstacle top {top:.2f}m -> climb to {target_alt:.2f}m (margin {margin:.2f}m)"
            elif not ceiling_ok:
                detail = f"obstacle top {top:.2f}m -> {target_alt:.2f}m exceeds ceiling {cfg.max_safe_altitude:.2f}m"
            else:
                detail = f"overhead blocked: up clearance {state.clearance_up:.2f}m < {overhead_need:.2f}m needed"
            climb = AvoidanceCandidate("CLIMB", margin, ceiling_ok and overhead_ok, detail)

        return [forward, left, right, climb]

    def _rejected_summary(
        self, candidates: List[AvoidanceCandidate], selected: Optional[str]
    ) -> str:
        """Explicit reasons for every candidate that is unsafe (not selected)."""
        return "; ".join(
            f"{c.direction} rejected ({c.detail})"
            for c in candidates
            if not c.safe and c.direction != selected
        )

    def _decision(
        self,
        state: FlightState,
        context: FlightContext,
        decision: DecisionType,
        factors: List[str],
        detail: str,
        constraints: List[str],
        candidates: Optional[List[AvoidanceCandidate]] = None,
        selected: Optional[str] = None,
    ) -> Decision:
        cfg = self.config
        weights = CONTEXT_WEIGHTS[context]
        reason = detail
        if factors:
            reason = f"active factors: [{' ; '.join(factors)}] -> {detail}"

        cmd = self._command_for(state, context, decision)
        return Decision(
            context=context,
            decision=decision,
            reason=reason,
            constraints=list(constraints),
            weights={obj.value: round(w, 2) for obj, w in weights.items()},
            command=cmd,
            candidates=list(candidates or []),
            selected_direction=selected,
        )

    def _command_for(
        self, state: FlightState, context: FlightContext, decision: DecisionType
    ) -> FlightCommand:
        """Map a decision onto a concrete :class:`FlightCommand`."""
        cfg = self.config
        cmd = FlightCommand(target_altitude=state.target_altitude)

        if decision == DecisionType.EMERGENCY:
            cmd.forward_speed = 0.0
            cmd.lateral_speed = 0.0
            cmd.speed_limit = 0.0
            cmd.target_altitude = max(state.altitude, cfg.min_safe_altitude + 0.2)
            if state.obstacle_distance <= cfg.obstacle_critical:
                # Obstacle-triggered emergency: a pure freeze is a trap — wind
                # or residual velocity presses the drone into the obstacle.
                # Escape along the negative sensor gradient: directly away
                # from the body direction of the ray that triggered the
                # emergency. A fixed left/right slide can end up parallel to
                # the obstacle face and stop gaining clearance.
                trigger = min(
                    (
                        (state.clearance_front, (1.0, 0.0)),
                        (state.clearance_left, (0.7071, 0.7071)),
                        (state.clearance_right, (0.7071, -0.7071)),
                    ),
                    key=lambda kv: kv[0],
                )[1]
                side_clear = max(state.clearance_left, state.clearance_right)
                if side_clear >= cfg.obstacle_critical + cfg.safety_margin:
                    side = 1.0 if state.clearance_left >= state.clearance_right else -1.0
                    ex, ey = -trigger[0], -trigger[1] + 0.5 * side
                    norm = math.hypot(ex, ey)
                    cmd.forward_speed = (ex / norm) * cfg.cautious_speed
                    cmd.lateral_speed = (ey / norm) * cfg.cautious_speed
                    cmd.speed_limit = cfg.cautious_speed
                elif state.altitude < cfg.max_safe_altitude - 0.5:
                    # Wedged: both sides blocked — climb over (never descend
                    # blindly into a possible collision). Target just above
                    # the detected obstacle top when the face is known.
                    if state.obstacle_top is not None:
                        cmd.target_altitude = min(
                            cfg.max_safe_altitude,
                            max(state.altitude + 0.4, state.obstacle_top + cfg.climb_margin),
                        )
                    else:
                        cmd.target_altitude = min(cfg.max_safe_altitude, state.altitude + 0.6)
        elif decision == DecisionType.SAFE_LANDING:
            cmd.target_altitude = 0.0
            cmd.forward_speed = 0.0
            cmd.speed_limit = 0.0
        elif decision == DecisionType.AVOID_OBSTACLE:
            cmd.speed_limit = cfg.cautious_speed
            side = 1.0 if state.clearance_left >= state.clearance_right else -1.0
            cmd.lateral_speed = side * cfg.cautious_speed
            # Creep forward only while the corridor keeps the detour gate
            # (critical + margin); back away as soon as the pass tightens
            # below it so the flown clearance never shrinks into a near miss.
            if state.obstacle_distance <= cfg.obstacle_critical + cfg.safety_margin:
                cmd.forward_speed = -0.25
            else:
                cmd.forward_speed = 0.15
        elif decision == DecisionType.DESCEND:
            cmd.speed_limit = cfg.cautious_speed
        elif decision == DecisionType.ENERGY_CONSERVATION:
            cmd.speed_limit = cfg.conserve_speed
            cmd.forward_speed = min(cfg.conserve_speed, cmd.forward_speed)
        elif decision == DecisionType.WIND_RESPONSE:
            cmd.speed_limit = cfg.cautious_speed
        elif decision == DecisionType.CLIMB:
            if state.obstacle_top is not None:
                # Avoidance climb-over: aim just above the detected obstacle
                # top while respecting the altitude ceiling.
                cmd.target_altitude = max(
                    state.target_altitude,
                    min(cfg.max_safe_altitude, state.obstacle_top + cfg.climb_margin),
                )
            else:
                cmd.target_altitude = max(state.target_altitude, state.altitude + 0.4)
            cmd.speed_limit = 0.3
        elif decision == DecisionType.MOVE_FORWARD:
            # Full cruise only outside the post-maneuver caution window;
            # heading changes can sweep body-frame rays off a close obstacle,
            # so speed stays cautious until the window expires.
            cmd.speed_limit = (
                cfg.cautious_speed
                if state.maneuver_cooldown_s > 0.0
                else cfg.cruise_speed
            )
        elif decision == DecisionType.MISSION_COMPLETE:
            cmd.speed_limit = 0.0
        return cmd
