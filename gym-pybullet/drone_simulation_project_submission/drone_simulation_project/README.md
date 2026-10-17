# 🚁 Adaptive Multi-Objective Flight Decision Algorithm for Autonomous Quadcopters

A physically grounded 6-DOF quadcopter simulation in **Python 3.10+ / PyBullet /
Gymnasium**, built around the **ACMFDS** decision layer: an *explainable,
rule-based, multi-objective flight decision system* that flies a complete
mission — **INITIALIZE → TAKEOFF → HOVER → MOVE FORWARD → SLOW DOWN → LAND** —
through real rigid-body physics.

> [!IMPORTANT]
> **Terminology note (documented discrepancy).** Earlier revisions of this
> repository expanded ACMFDS as *"Adaptive Collision Mitigation and Flight
> Decision System"*. The project title standardizes on **Adaptive
> Context-Aware Multi-Objective Flight Decision System**, which is used
> consistently from this revision onward. The old expansion appears only in
> this note.

---

## 1. What the system does (default execution path)

```
PHYSICS SIMULATION        LIVE FLIGHT STATE           ACMFDS DECISION
(PyBullet rigid body) --> (validated FlightState) --> (context, decision,
                                                        reason, weights,
                                                        constraints)
        ^                                                    |
        |                                                    v
PHYSICAL DRONE SIM        FLIGHT COMMAND / ACTUATION <-----(FlightCommand)
(PyBullet forces/          (SMC actuation layer:
 torques, no teleports)     thrust & torque commands)
```

Per control step (120 Hz):

1. **Sense** — PyBullet base state (pose, twist) + real `rayTestBatch` forward
   clearances (front/left/right).
2. **Model** — battery (Wh workload model) and wind (steady + gust + seeded
   turbulence) update; both are *modeled* inputs, labeled as such, never
   presented as physical sensors.
3. **Decide** — ACMFDS classifies the context (NORMAL, OBSTACLE_NEAR,
   OBSTACLE_CRITICAL, LOW_BATTERY, STRONG_WIND, LANDING, EMERGENCY,
   MISSION_COMPLETE), weighs the four objectives (SAFETY, ENERGY_EFFICIENCY,
   OBSTACLE_CLEARANCE, MISSION_PROGRESS), and emits an explainable
   `Decision` with an executable `FlightCommand`.
4. **Compose** — the mission sequencer proposes; ACMFDS disposes (veto set:
   EMERGENCY / SAFE_LANDING / MISSION_COMPLETE / HOLD / CLIMB / DESCEND /
   AVOID_OBSTACLE replace mission motion; otherwise forward speed =
   min(mission, ACMFDS limit), lateral comes from ACMFDS avoidance).
5. **Actuate** — a sliding-mode control (SMC) layer converts the command into
   normalized thrust/torque demands applied via `applyExternalForce/Torque`.
6. **Log** — per-step CSV telemetry + computed summary metrics (nothing
   invented; every number comes from a logged run).

A decision such as `MOVE_FORWARD` is never a motor RPM: the decision →
command → actuation mapping is explicit and lives in
`decision/acmfds.py::_command_for` and `controllers/actuation.py`.

### Conflict priority (documented)

**Collision safety > energy > mission progress.** A low-battery drone still
avoids a critical obstacle (at reduced speed); a drone never auto-descends
into an obstacle — when both detour sides are blocked it climbs over (up to a
configured ceiling) or holds.

---

## 2. Honest engineering notes (read before citing)

- **No PID anywhere.** The actuation law is sliding-mode control (nonlinear,
  no integral term, no PID structure). The optional learned feedforward model
  (`controllers/data_driven_controller.py`) is trained on pure inverse
  dynamics `T* = m(g + z̈_ref) + c·ż_ref` — an earlier revision trained it on
  PID targets; that violation was removed and the model retrained. Like every
  feedback controller, SMC closes a loop around state error; the project's
  claim is *"no PID controller"*, not *"no feedback"*.
- **No RL in the default path.** `python main.py` never imports
  stable-baselines3/torch. RL training (`train.py`) is an optional extra
  (`requirements-rl.txt`); pretrained model files are kept but unused by the
  default pipeline.
- **No teleports, no tweaked physics.** The drone is a 1.0 kg rigid body with
  PyBullet-computed inertia (0.0230, 0.0230, 0.0417 kg·m²), hover thrust
  9.81 N, max 20 N. Success is achieved by control, not by editing mass or
  motor constants. (Two genuine physics bugs found during validation were
  fixed: the inertia sync read the wrong `getDynamicsInfo` index, and
  WORLD-frame drag/wind forces were applied at the world origin instead of
  the body COM, creating a spurious lever-arm torque.)
- **Tested, not claimed.** Every scenario result in §8 was produced by an
  actual headless run. Remaining limitations are listed in §9.

---

## 3. Installation

```bash
pip install -r requirements.txt        # core: pybullet, gymnasium, numpy, scikit-learn, joblib
pip install -r requirements-rl.txt     # optional: torch + stable-baselines3 (RL training only)
```

Python 3.10+ recommended (developed and tested on 3.13).

## 4. Running

```bash
python main.py                                # GUI, scenario A (normal)
python main.py --headless                     # fast, no window
python main.py --scenario obstacle            # B: obstacle ahead
python main.py --scenario low-battery         # C: 32 % initial charge
python main.py --scenario strong-wind         # D: 4.5 m/s + gusts crosswind
python main.py --scenario combined            # E: obstacle + low battery + wind
python main.py --scenario moving-obstacles    # F: pillars + 2 genuinely moving obstacles
python main.py --list-scenarios
python main.py --headless --episodes 3 --seed 7 --output results/run.csv
python main.py --mode manual                  # keyboard manual control (legacy mode 1)
python evaluate.py --scenario obstacle --episodes 5   # aggregate evaluation CSV

# Integrated system — ONE command, then open the page in a browser:
"C:\Users\jovik\anaconda3\envs\pybullet\python.exe" -m dashboard.server   # -> http://127.0.0.1:8000
# (tick "Native PyBullet window" in Run Control before Start to watch the
#  same flight in the PyBullet window while the dashboard and CSV stream it)
```

Each run writes a per-step CSV (`results/acmfds_log_<scenario>_<episode>.csv`)
with state, decision, reason, candidate evaluations, selected direction, command,
mission phase, moving-obstacle positions, and thrust columns, and prints a
summary (outcome, duration, min obstacle distance, altitude errors, battery
consumed, decision/context distribution, emergency/safe-landing counts).

### Live dashboard (FastAPI + WebSockets)

`python -m dashboard.server` serves a dark-theme control room at
**http://127.0.0.1:8000**:

- **One simulation loop only.** The PyBullet mission runs in a single
  server-side thread at its fixed 120 Hz timestep with realtime pacing;
  browsers are read-only WebSocket clients. Refresh rate, multiple tabs, or a
  closed tab cannot change the physics or the logged data. Duplicate starts
  are rejected while a run is active, and reset/scenario changes first
  require stopping it — two loops can never coexist.
- **Three synchronized outputs of the same flight**: the CSV log and every
  browser graph/panel are fed by the *same per-step rows*, and the optional
  **Native PyBullet window** checkbox runs that identical loop with
  `gui=True`, so the host screen shows the same flight the browser and CSV
  report (verified: 3271/3271 rows, identical outcome and metrics).
- **Browser 3D view**: an orthographic canvas viewer of the actual state —
  drone pose from the live row (position/attitude), obstacles from the
  spawned world geometry, moving boxes from the row's logged positions. It
  is a visualization only; no physics runs in the browser.
- **Live telemetry + 9 graphs/views**: 3D view, **SIMULATED GPS-REFERENCED
  OBSTACLE MAP** (see below), battery, altitude vs target, ground path X/Y
  (with goal marker + live moving-obstacle squares), velocity, obstacle
  distance, wind speed, and a context-colored decision timeline. The battery
  graph uses an honest auto-ranged axis (labeled true values, min span 10 %)
  with a dashed marker at the starting charge, so a real ~8 % draw is visible
  instead of looking flat at 100 %. The obstacle-distance graph carries a
  legend and a marked line at the sensor range cap: a ray that hits nothing
  travels its full 5.0 m (`max_ray_distance`), so a flat 5.00 reading means
  "no obstacle within 5.0 m" — a valid capped measurement, never a fabricated
  or invalid value; abrupt jumps are rays entering/leaving range or sweeping
  off a body during heading changes. Missing fields (should never occur — the
  validator clamps every reading) would render as gaps, not as range values.
- **Explainable ACMFDS panel**: context → decision, triggering reason,
  candidate clearances with accepted/rejected verdicts, the direction/action
  actually in force, and the executed command (`fwd/lat/alt/thrust`). The
  direction display reads the logged `movement_action` column — the
  directional pick (LEFT/RIGHT/CLIMB/FORWARD) when ACMFDS selects one, else
  the decision in force (MOVE_FORWARD / HOLD / DESCEND / ...) — so it is
  never blank during flight; before the first decision it shows
  *Waiting for first decision*, and after landing it shows the final action.
  A maneuver is only marked CLEARED once the obstacle distance actually
  recovers — a printed decision is not treated as a successful maneuver.
- **Controls**: start / pause / resume / stop / reset work on the real
  simulation thread (verified end-to-end: pause freezes it at a safe step
  boundary — no new rows, sim time, battery, and position all frozen while
  the last valid telemetry stays visible; resume continues the *same*
  episode; stop aborts mid-flight with `aborted — stopped by operator`;
  reset clears; pause/resume requests are rejected when not applicable).
- **Validated config** (server-side pydantic, 422 on out-of-range): scenario,
  target altitude, goal X/Y, cruise speed, initial battery, wind
  speed/direction/enable, and a live dynamic-obstacle toggle.
- **Simulated GPS (optional, clearly labeled, three separate concepts)**:
  off by default and never required to run a simulation. **A — operator
  device location**: "Use current device location" asks the browser for
  permission only when clicked (one-shot, or "Track device continuously"
  with an explicit Stop control) and only *displays* the fix — latitude,
  longitude, accuracy (low-accuracy fixes are flagged as approximate), fix
  time, and source; coordinates stay in the browser and never touch the
  simulation or leave it. **B — geographic origin**: a distinct, explicit
  action — "Use device fix as origin" or "Set origin from fields" — that
  never enables GPS by itself and never moves the drone. The origin
  corresponds EXACTLY to world position (X=0, Y=0) of the simulation frame
  (X=east, Y=north); the drone starts at (−4.0, 0.0), 4 m west of the
  origin, and both are shown in the dashboard. **C — simulated drone GPS**:
  enabled independently with the checkbox; only when an origin exists and a
  flight is streaming does it show lat/lon (flat-earth conversion,
  `telemetry/gps.py`, computed server-side from the last logged physics row
  and updated continuously), the origin, and the last-update sim time.
  Without an origin it says *Origin not set*; disabled it says *Disabled*.
  **Real drone GPS** is shown separately and concisely as *Not connected* —
  no physical GPS telemetry source exists.
- **SIMULATED GPS-REFERENCED OBSTACLE MAP**: a top-down geographic schematic
  of the same flight — live drone marker with heading arrow, start,
  destination, actual static obstacle rectangles, actual moving-obstacle
  positions from the logged rows, and the flown route — all converted with
  the same world→geo transform as the simulated GPS, on a lat/lon-labelled
  graticule, with the origin marked at world (0,0). Relative geometry and
  distances come from the simulation itself; a device-fix origin's accuracy
  (e.g. ±107 m) shifts only the absolute placement and is stated as such.
  It is a local schematic of simulated positions — not a real map, and the
  simulated obstacles are not real-world objects. Without an origin it says
  so instead of inventing one.

### In-GUI HUD (PyBullet runs)

GUI runs draw a dark translucent HUD panel between the chase camera and the
drone: mission phase, altitude/target, speed/position, obstacle clearances,
battery/wind (amber when low), the ACMFDS context → decision line, the
selected direction and executed command, the full reason text, candidate
verdicts, and an honest maneuver tracker that shows ACTIVE / CLEARED / ENDED
once the obstacle distance actually recovers. Colors follow the same
severity scheme as the dashboard (cyan telemetry, green nominal, amber
warning, red emergency). The obstacle rays are only drawn while the drone is
in an obstacle context, keeping the view clean. HUD text elements use
PyBullet's GUI-only debug renderer, so they appear in the interactive window
(offscreen captures show the panel geometry but not the text).



## 5. Testing

```bash
python test_env.py
```

35 core tests: Gymnasium `check_env`, reset/step contracts, hover physics,
FlightState validation + per-field source tags, ACMFDS rules across all
contexts (incl. critical-obstacle retreat and never-descend-into-obstacle),
battery bounds, wind reproducibility and documented force conversion,
mission phase chain, decision→command→actuation path, landing protection
gate, ray sensing vs a *known* pillar geometry, visual parts following the
physics body (yaw-rotated offsets), sequential closed-loop accuracy of the
learned feedforward model (MAE < 0.15 N over a minimum-jerk maneuver),
headless mission CSV logging, and a guard that the default path imports no
RL packages.

**63 tests** as of the current revision (35 core + speed-plan limits,
moving-obstacle determinism, dashboard API contract, simulated-GPS math and
`/api/gps` validation, telemetry integrity, clearance-margin gating and
emergency/safe-landing counting, post-maneuver cooldown speed limiting,
world-description fidelity, the opt-in native-window flag, true-geometry
`goal_distance` logging, `movement_action` logging on every step — the
dashboard direction-display field, never empty in flight — simulated-GPS
origin gating: no origin → no coordinates, ever; the origin is exactly world
(0,0) and the drone start offset is reported; and a live pause/resume
flight: while paused no rows are logged and sim time, battery, and position
freeze, resume continues the same episode (step counter + dt continuity),
Stop remains distinct from Pause, pause/resume request guards reject
no-op calls, and the logged battery drain matches the documented Wh model
recomputed from the thrust trace).

## 6. Flight state: what is physical, what is modeled

Every field of the unified `FlightState` (state/flight_state.py) carries a
source tag (`state.source_of(field)`):

| Source | Fields |
|---|---|
| **SIMULATION** (PyBullet-derived) | position, altitude, velocities, roll/pitch/yaw, body rates, target altitude/position, goal distance |
| **SENSOR** (real `rayTestBatch` rays) | obstacle distance, clearance front/left/right |
| **MODELED** (simulated subsystems, *not* physical sensors) | battery percentage, wind speed, wind direction |

`StateValidator.sanitize()` clamps out-of-range values, normalizes the wind
direction vector, and records **every** correction in `state.issues` — no
silent rewrites.

## 7. Scenario definitions (all seed 42, start (−4, 0), goal (4.5, 0), cruise 2.0 m)

| Scenario | Conditions | Purpose |
|---|---|---|
| A `normal` | wind 0.5 m/s, no obstacles | full mission chain |
| B `obstacle` | two pillars (3.0 m and 2.6 m tall) blocking the route | sensing + AVOID_OBSTACLE |
| C `low-battery` | 32 % charge, side pillar | ENERGY_CONSERVATION context |
| D `strong-wind` | 4.5 m/s +y crosswind, gusts to 6 m/s, downwind pillar | WIND_RESPONSE + landing in wind |
| E `combined` | 28 % charge, blocking pillar, 3.5 m/s wind | priority: safety > energy > mission |
| F `moving-obstacles` | 2 static pillars + 2 genuinely moving boxes (closed-form sinusoidal sweep, deterministic) | dynamic-obstacle avoidance + LEFT/RIGHT/FORWARD/CLIMB candidate evaluation |

ACMFDS thresholds are identical in every scenario (`ACMFDSConfig`);
scenarios change only the environment. Detour sides are gated by
`safety_margin` (a candidate side is only safe if its clearance exceeds
`obstacle_critical + safety_margin` = 1.1 m by default), and after each
avoidance maneuver a `maneuver_cooldown` window (1.0 s) holds cautious
speed — body-frame rays can lose an obstacle mid heading-sweep, so NORMAL
cruise is not restored until the cooldown expires. The maneuver tracker
judges each maneuver from the actual sensor minimum and reports a close
pass (min < margin) honestly rather than calling every non-collision a
success. The moving obstacles are kinematic
bodies with real collision geometry whose position is an analytic function of
time — they are actually re-posed every physics step (never teleported past
the drone and never moved only in a log).

## 8. Actual headless results (fresh seed 123, single episode per scenario)

| Scenario | Outcome | Time | Collisions | Min obstacle dist | Notes |
|---|---|---|---|---|---|
| normal | **landed** | 16.3 s | 0 | 5.0 m (no obstacles) | 0 emergencies |
| obstacle | **landed** | 27.3 s | 0 | 0.56 m | above the 0.30 m safety margin; detours + post-maneuver caution |
| low-battery | **landed** | 25.3 s | 0 | 1.19 m | energy context dominated, 0 emergencies |
| strong-wind | **landed** | 20.6 s | 0 | 0.95 m | 4.5 m/s + gusts, 0 emergencies |
| combined | **landed** | 30.0 s | 0 | 0.80 m | safety overrode energy near the pillar |
| moving-obstacles | **landed** | 21.6 s | 0 | 0.77 m | movers logged moving each step, emergency retreat survived |

Command to reproduce: `python main.py --headless --scenario <name> --seed 123`.
All six scenarios re-ran to completion with zero collisions on this seed.
Dashboard verification (separate runs of the same scenarios through
`dashboard/server.py` with realtime pacing): mission landed at goal, 2364
logged steps, summary identical in structure to the headless runs, moving
obstacles observed translating in the logged telemetry (y from −2.18 to
−0.56 and from +0.49 to −1.73 within one run).

## 9. Remaining limitations (honest list)

- **Avoidance is reactive, not planned.** Detours are chosen from the three
  forward-cone rays only; there is no global path planner, so a detour can
  momentarily pass close to a secondary obstacle (min distances 0.14–0.5 m
  in scenarios B/E).
- **Sensing is a 3-ray forward cone.** The drone is blind behind and above;
  obstacle_distance is the minimum of front/left/right clearances.
- **Wind feedforward uses documented guesses** (ρ = 1.225, Cd = 1.0,
  A = 0.09 m²); the trim removes steady-state drift but gusts still excite
  transient error (altitude error mean ≈ 0.2 m in wind scenarios).
- **Ground effect is not modeled**; the landing gate (level + zero horizontal
  demand below 0.45 m) is a control-side protection, not aerodynamics.
- **Learned feedforward model is optional and altitude-only**; the SMC layer
  remains the primary actuation path. The model's accuracy is validated
  during maneuver windows; out-of-maneuver instantaneous-error queries are
  not meaningful for a feedforward model and are not claimed.
- **Moving obstacles are kinematic.** They follow a fixed closed-form
  sinusoidal path (mass 0, re-posed every physics step) and are not affected
  by the drone; the drone must sense and avoid them, not push them.
- **Simulated GPS is a flat-earth approximation** around a configurable
  geographic ORIGIN that is exactly world (X=0, Y=0) — the drone starts at
  (−4, 0), 4 m west of it (metre-accurate at this ~15 m scenario scale). It
  is a dashboard display feature derived from logged X/Y, always labeled
  *simulated*, and is never presented as a real GPS receiver. A device fix
  used as the origin carries the browser-reported accuracy (e.g. ±107 m),
  which shifts the absolute placement of the geographic map only — never the
  relative obstacle geometry, which comes from the simulation.
- **GPS waypoint navigation is not implemented.** The geographic map and
  simulated GPS are display/verification features; the navigation goal stays
  in local world coordinates and ACMFDS decisions are unchanged. Browser
  device geolocation needs a permission prompt and a supporting browser; it
  is never required to run a simulation. The geographic obstacle map is a
  local schematic, not an online map — no external mapping service is
  contacted and device coordinates never leave the browser.
- **RL path is unmaintained relative to the new architecture**: `train.py`
  trains against `DroneEnv`, which is kept Gymnasium-compliant, but RL is not
  part of the validated pipeline.

## 10. Project structure

```
main.py                       default CLI (scenarios, headless, manual mode)
legacy_modes.py               Mode 1 manual keyboard control
evaluate.py                   multi-episode evaluation (RL evaluation optional/lazy)
scenarios.py                  scenario definitions A–E (reproducible)
state/flight_state.py         FlightState dataclass + sources + validator
decision/acmfds.py            ACMFDS: contexts, weights, decisions, commands
controllers/actuation.py      SMC actuation layer (thrust/torque from command)
controllers/data_driven_controller.py  optional learned feedforward (PID-free)
simulation/pybullet_sim.py    physics client wrapper (grid, markers, camera)
simulation/drone.py           rigid-body drone model + X-visual + rays
simulation/obstacles.py       obstacle manager (+ config-driven spawning)
simulation/mission.py         mission phase sequencer
simulation/battery.py         workload-based battery model (Wh)
simulation/wind.py            steady+gust+turbulence wind, force conversion
simulation/acmfds_mission.py  the end-to-end mission runner (sense→act→log)
telemetry/logger.py           per-step CSV + computed summaries
telemetry/gps.py              simulated GPS conversion (origin = world (0,0), X east / Y north)
dashboard/server.py           FastAPI + WebSocket dashboard (single sim thread)
dashboard/static/             dashboard frontend (HTML/CSS/JS, no frameworks)
envs/drone_env.py             Gymnasium DroneEnv (kept for RL/check_env)
trajectory/interpolation.py   waypoint interpolation (legacy support)
train.py                      optional RL training (PPO/SAC) — not in default path
train_data_driven.py          feedforward model trainer (inverse dynamics)
models/                       pretrained model files (kept; core path needs none)
test_env.py                   test suite (63 tests)
```

## 11. ACMFDS objectives and contexts

Four objectives, re-weighted per context (weights always sum to 1; see
`decision/acmfds.py::CONTEXT_WEIGHTS`):

- **SAFETY** — altitude/tilt/ground envelope, battery survival.
- **ENERGY_EFFICIENCY** — battery-aware speed and climb moderation.
- **OBSTACLE_CLEARANCE** — ray-based proximity and detour behavior.
- **MISSION_PROGRESS** — goal-directed forward motion.

Contexts: NORMAL, OBSTACLE_NEAR, OBSTACLE_CRITICAL, LOW_BATTERY,
STRONG_WIND, LANDING, EMERGENCY, MISSION_COMPLETE. Every decision ships with
`reason` (human-readable factor list), `constraints`, `weights`, and a
`FlightCommand` — the output is auditable, not a black box.

## 12. Gymnasium environment

`envs/drone_env.py` exposes the 27-dim observation (pose, twist, goal
vector, target altitude, 8 normalized ray channels) and 4-dim continuous
action (normalized thrust delta over hover, roll/pitch/yaw torque demands;
action `0` = hover thrust). It passes `gymnasium.utils.env_checker.check_env`
and reports the current ACMFDS decision/context/reason in `info`.
