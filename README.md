#  Adaptive Multi-Objective Flight Decision Algorithm for Autonomous Quadcopters

A physically grounded 6-DOF quadcopter simulation built with **Python 3.10+ and PyBullet**, centered around ACMFDS: an explainable, rule-based, multi-objective flight decision system.

The system executes a complete simulated mission:

**INITIALIZE → TAKEOFF → HOVER → MOVE FORWARD → SLOW DOWN → LAND**

All flight behaviour is evaluated through rigid-body physics and an explicit decision-to-actuation pipeline.

> [!IMPORTANT]
> **Terminology note:** Earlier revisions expanded ACMFDS as "Adaptive Collision Mitigation and Flight Decision System." The project title now standardizes the name as **Adaptive Context-Aware Multi-Objective Flight Decision System**. The earlier expansion is retained only as a historical note.

---

## 1. What the System Does

The default execution path combines PyBullet physics, a validated flight state, explainable ACMFDS decisions, a sliding-mode actuation layer, and per-step telemetry logging.

### System Architecture

```text
┌─────────────────────────┐
│   PyBullet Physics      │
│   Rigid-body dynamics   │
└────────────┬────────────┘
             │
             ▼
┌─────────────────────────┐
│    Live FlightState     │
│ Pose, velocity, sensors │
│ Battery and wind models │
└────────────┬────────────┘
             │
             ▼
┌─────────────────────────┐
│        ACMFDS           │
│ Context classification  │
│ Multi-objective scoring │
│ Safety checks and       │
│ explainable decisions   │
└────────────┬────────────┘
             │
             ▼
┌─────────────────────────┐
│   FlightCommand         │
│ Mission/ACMFDS command  │
│ composition             │
└────────────┬────────────┘
             │
             ▼
┌─────────────────────────┐
│ Sliding-Mode Actuation  │
│ Normalized thrust       │
│ Torque commands         │
└────────────┬────────────┘
             │
             ▼
┌─────────────────────────┐
│   PyBullet Physics      │
│ Forces and torques      │
│ Updated physical state  │
└─────────────────────────┘

    Per-step state and decisions
                 │
                 ▼
┌─────────────────────────┐
│ Flight Logger           │
│ CSV + summary metrics   │
└─────────────────────────┘
```

### Per-Control-Step Workflow

The simulation operates at a control rate of **120 Hz**.

1. **Sense:** Read the drone's pose, velocity, orientation, and forward-cone ray clearances from PyBullet.
2. **Model:** Update battery consumption and wind disturbances. These are modeled inputs, not measurements from physical battery or wind sensors.
3. **Decide:** ACMFDS classifies the current context, evaluates objective weights and safety constraints, and produces an explainable `Decision` with an executable `FlightCommand`.
4. **Compose:** The mission sequencer proposes motion. ACMFDS can override or restrict that motion according to its decision and safety constraints.
5. **Actuate:** The sliding-mode control layer converts the command into normalized thrust and torque demands.
6. **Simulate:** Apply forces and torques to the rigid body through PyBullet.
7. **Log:** Record the state, decisions, reasons, candidate evaluations, commands, and evaluation metrics.

A decision such as `MOVE_FORWARD` is not a motor RPM. The decision-to-command mapping is explicit in `decision/acmfds.py`, while control commands are executed by `controllers/actuation.py`.

### Conflict Priority

The documented priority is:

**Collision safety > energy conservation > mission progress**

A low-battery drone must still respond to a critical obstacle. When the available detour directions are blocked, the system can climb within its configured ceiling or hold, rather than automatically descending into an obstacle.

The exact action depends on the current context, clearances, and implemented safety checks.

---

## 2. Engineering Design and Important Distinctions

### No PID Controller

The default actuation architecture uses **Sliding Mode Control (SMC)** rather than a PID controller.

SMC is a nonlinear feedback-control method based on a sliding surface and switching or saturation-based control terms. It can provide robustness against certain disturbances when appropriately designed and tuned.

The project does not claim to have no feedback. SMC is a feedback controller; the design claim is that the default controller does not use PID.

The optional learned feedforward model in `controllers/data_driven_controller.py` is trained using inverse-dynamics targets:

$$
T^* = m\left(g+\ddot{z}_{\mathrm{ref}}\right)
+ c\dot{z}_{\mathrm{ref}}
$$

where:

- $T^*$ is the target thrust.
- $m$ is the modeled mass.
- $g$ is gravitational acceleration.
- $\ddot{z}_{\mathrm{ref}}$ is reference vertical acceleration.
- $\dot{z}_{\mathrm{ref}}$ is reference vertical velocity.
- $c$ is the modeled drag coefficient.

An earlier revision trained the learned model on PID-derived targets. That approach was removed, and the model was retrained using inverse-dynamics targets.

### No Reinforcement Learning in the Default Path

The default flight-decision path uses explainable ACMFDS rules and the SMC actuation layer. Reinforcement learning is not required to execute the default mission.

The optional learned component is an altitude-only feedforward controller. It is separate from the ACMFDS decision layer and is not required for the documented scenario results.

### Physics-Based Flight

The drone is modeled as a 1.0 kg rigid body with PyBullet-computed inertia:

| Parameter | Value |
|---|---:|
| Mass | 1.0 kg |
| $I_{xx}$ | 0.0230 kg·m² |
| $I_{yy}$ | 0.0230 kg·m² |
| $I_{zz}$ | 0.0417 kg·m² |
| Hover thrust | 9.81 N |
| Maximum thrust | 20 N |

The implementation uses physical forces and torques instead of teleporting the drone to desired positions.

Two physics implementation bugs were corrected during validation:

- The inertia synchronization logic used the wrong `getDynamicsInfo` index.
- World-frame drag and wind forces were applied at the world origin rather than at the drone's center of mass, creating an unintended lever-arm torque.

### Tested, Not Merely Claimed

Scenario results are based on logged simulation runs. Remaining limitations are documented in Section 9.

---

## 3. Installation

Python 3.10 or later is recommended. The project was developed and tested on Python 3.13.

Install the core dependencies:

```bash
pip install -r requirements.txt
```

Core dependencies include PyBullet, NumPy, scikit-learn, joblib, FastAPI, Uvicorn, and WebSockets.

---

## 4. Running the Project

### Standard Simulation

```bash
python main.py
```

### Headless Simulation

```bash
python main.py --headless
```

### Individual Scenarios

```bash
python main.py --scenario obstacle
python main.py --scenario low-battery
python main.py --scenario strong-wind
python main.py --scenario combined
python main.py --scenario moving-obstacles
python main.py --list-scenarios
```

### Multiple Episodes

```bash
python main.py --headless --episodes 3 --seed 7 --output results/run.csv
```

### Evaluate Multiple Runs

```bash
python evaluate.py --scenario obstacle --episodes 5
```

### Manual Keyboard Mode

```bash
python main.py --mode manual
```

Manual keyboard control is a legacy mode.

### Integrated Dashboard

Start the dashboard with:

```bash
python -m dashboard.server
```

Then open:

**http://127.0.0.1:8000**

On Windows, use the Python executable from the configured project environment if necessary:

```cmd
"C:\Users\jovika\anaconda3\envs\pybullet\python.exe" -m dashboard.server
```

To view the native PyBullet window for the same flight, enable **Native PyBullet window** in Run Control before starting the mission.

### Flight Logs

Each run writes a per-step CSV file, typically:

```text
results/acmfds_log_<scenario>_<episode>.csv
```

The output records include:

- Position, altitude, velocity, and orientation.
- Mission phase and goal distance.
- Obstacle distances and directional clearances.
- Battery and wind values.
- ACMFDS context, decision, and reason.
- Candidate evaluations and selected direction.
- Movement action and executed command.
- Thrust, collisions, and mission status.

A summary reports the outcome, duration, minimum obstacle distance, altitude errors, battery consumption, decision distribution, and emergency/safe-landing counts.

### Live Dashboard Features

The dashboard is implemented with FastAPI, WebSockets, HTML, CSS, and JavaScript.

**One authoritative simulation loop:** The PyBullet mission runs on the server. The browser receives read-only telemetry snapshots. Refreshing the browser does not advance the physics simulation, and duplicate mission starts are rejected while a flight is active.

**Synchronized outputs:** The native PyBullet window, browser 3D view, graphs, and CSV log use data from the same flight.

**3D visualization:** The browser draws the actual logged drone pose and the spawned obstacle geometry. Moving obstacles use positions from logged simulation rows. The browser view is a visualization, not a separate physics engine.

**Telemetry and graphs:** The dashboard displays altitude, target altitude, trajectory, velocity, obstacle distance, wind, battery, and the decision timeline.

**Explainable decision panel:** The interface shows the ACMFDS context, decision, triggering reason, candidate clearances, accepted/rejected verdicts, movement action, and executed command.

**Pause, resume, stop, and reset:** These controls act on the authoritative simulation. Pause freezes the flight at a safe step boundary; resume continues the same episode. Stop is distinct from pause.

**Validated configuration:** The server validates scenario, target altitude, goal coordinates, cruise speed, battery charge, wind configuration, and moving-obstacle settings.

---

## 5. Automated Testing

Run the test suite:

```bash
python test_env.py
```

The test suite covers the following areas:

- Gymnasium API compliance and reset/step contracts.
- Drone physics and hover behaviour.
- FlightState validation and per-field source tags.
- ACMFDS decision rules and explainable outputs.
- Battery bounds and workload-based consumption.
- Wind reproducibility and drag-force calculations.
- Mission phase progression.
- Decision-to-command and actuation limits.
- Sequential accuracy of the learned feedforward controller.
- Ray sensing against known obstacle geometry.
- Orientation-transformed visual-part positions.
- Per-step CSV logging.
- Reproducibility of moving obstacles.
- Dashboard API validation.
- Simulated GPS conversion and origin gating.
- Telemetry integrity and decision explanations.
- Obstacle-clearance margins and event counting.
- Speed-planning limits and braking behaviour.
- Pause/resume semantics and battery-model consistency.
- A guard against reinforcement-learning imports in the default application path.

### Reported Test Count

The supplied project revision documents **62 automated tests**.

Always run the current `test_env.py` before reporting the final test count. Report the actual result from the current project rather than assuming that an earlier test count still applies.

---

## 6. Flight State: Physical, Sensor, and Modeled Data

Each field in `state/flight_state.py` carries a source tag through `state.source_of(field)`.

| Source | Examples | Meaning |
|---|---|---|
| `SIMULATION` | Position, altitude, velocities, roll, pitch, yaw, target position | Derived from PyBullet's simulated state |
| `SENSOR` | Obstacle distance, front/left/right clearance | Obtained from simulated ray sensing |
| `MODELED` | Battery percentage, wind speed, wind direction | Calculated by modeled subsystems, not physical sensors |

`StateValidator.sanitize()` clamps invalid values, normalizes the wind direction vector, and records corrections in `state.issues`.

This separation helps prevent modeled values from being misrepresented as physical sensor measurements.

---

## 7. Reproducible Test Scenarios

The project contains six configured scenarios. Each scenario defines its environment and initial conditions while ACMFDS uses its documented default thresholds.

| Scenario | Conditions | Purpose |
|---|---|---|
| A: `normal` | Low wind, adequate battery, no obstacle directly blocking the planned route | Full mission sequence |
| B: `obstacle` | Two static pillars near the planned route | Sensing and obstacle avoidance |
| C: `low-battery` | Initial battery charge of 32%, with a side obstacle | Energy conservation and safe landing |
| D: `strong-wind` | 4.5 m/s base crosswind, gusts, and a downwind obstacle | Wind response |
| E: `combined` | 28% initial charge, an obstacle, and active wind | Safety priority under competing conditions |
| F: `moving-obstacles` | Two static pillars and two moving obstacles | Dynamic obstacle avoidance |

The standard start position is:

$$
(X_s,Y_s)=(-4.0,0.0)\ \text{m}
$$

The default target position is:

$$
(X_g,Y_g)=(4.5,0.0)\ \text{m}
$$

The default cruise altitude is 2.0 m.

The default horizontal goal distance from the start is:

$$
d=\sqrt{(X_g-X_s)^2+(Y_g-Y_s)^2}=8.5\ \text{m}
$$

The scenarios change the environment and initial conditions rather than deliberately changing ACMFDS thresholds between tests.

### Obstacle Safety Margin

The documented default detour gate is:

$$
d_{\mathrm{gate}} =
d_{\mathrm{critical}} + d_{\mathrm{safety\ margin}}
= 1.1\ \text{m}
$$

A candidate detour side must pass the relevant clearance check before it is accepted.

After an avoidance maneuver, a cooldown interval maintains cautious speed. The maneuver tracker checks actual sensor readings and does not automatically mark every non-collision maneuver as successful.

Moving obstacles follow deterministic analytic trajectories and are updated in the physics simulation at each step.

---

## 8. Reported Headless Scenario Results

The supplied project report records a set of headless runs using seed 123.

| Scenario | Outcome | Duration | Collisions | Minimum obstacle distance |
|---|---|---:|---:|---:|
| Normal | Landed | 16.3 s | 0 | 5.0 m |
| Obstacle | Landed | 27.3 s | 0 | 0.56 m |
| Low-battery | Landed | 25.3 s | 0 | 1.19 m |
| Strong-wind | Landed | 20.6 s | 0 | 0.95 m |
| Combined | Landed | 30.0 s | 0 | 0.80 m |
| Moving-obstacles | Landed | 21.6 s | 0 | 0.77 m |

These are reported observations from the specified runs, not universal safety guarantees. Re-run the scenarios before publication if you need to claim that these results were reproduced on the current revision.

Example reproduction command:

```bash
python main.py --headless --scenario obstacle --seed 123
```

The project also reports a dashboard synchronization test in which the browser stream and CSV contained matching records for the same episode. Such a result should be reported as verified only for the run actually checked.

---

## 9. Limitations and Honest Engineering Notes

### Reactive, Not Global Path Planning

Obstacle avoidance is reactive and depends on forward-cone ray measurements. There is no global path planner in the described default decision path. A detour can therefore pass close to a secondary obstacle even without producing a collision.

### Three-Ray Sensing

The drone uses a three-ray forward cone. It does not have complete omnidirectional obstacle sensing and is blind behind and above within this sensing arrangement.

### Simplified Wind Model

The wind model uses assumed parameters, including air density, drag coefficient, and exposed area. These provide modeled disturbances rather than calibrated measurements of real aerodynamic behaviour. Gusts can still produce transient tracking errors.

### Landing and Ground Effect

Ground effect is not modeled. The landing gate is a control-side protection that reduces horizontal demand near touchdown; it is not a complete aerodynamic ground-effect model.

### Learned Feedforward Model

The learned feedforward controller is optional and altitude-only. The main actuation path uses SMC. The optional model is trained for inverse-dynamics feedforward prediction and is not required for the documented default mission results.

### Moving Obstacles

Moving obstacles are kinematic bodies following deterministic sinusoidal trajectories. They move through the simulation but are not pushed by the drone.

### Simulated GPS

The geographic map converts simulated world coordinates into approximate latitude and longitude around a configurable geographic origin. It uses a local flat-earth approximation suitable for the short distances of these scenarios.

The geographic origin corresponds to world position `(X=0, Y=0)`. The default drone start is `(-4, 0)`, four metres west of that origin.

The mapping is a visualization derived from logged PyBullet positions. It is **not a real GPS receiver or a real-world obstacle map**.

If the browser supplies a device location with reported uncertainty, for example ±107 m, this affects the absolute geographic placement of the map. It does not change the relative obstacle geometry measured inside the simulation.

### GPS Waypoint Navigation

GPS waypoint navigation is not implemented in the described revision. The simulated GPS and geographic obstacle map are display and verification features. The navigation goal remains in local world coordinates, and ACMFDS decisions are unchanged by the GPS origin.

Browser geolocation is optional and is not required to run the simulation.

### No Physical GPS Telemetry

No physical drone GPS telemetry source is connected. The interface must not represent simulated GPS or operator device location as the physical drone's measured GPS position.

### Safety Claims

Zero recorded collisions in a scenario means that no collisions were recorded in that run. It does not prove universal safety in every obstacle layout, wind condition, initial state, or real-world flight.

---

## 10. Project Structure

```text
drone_simulation_project/
├── main.py
├── legacy_modes.py
├── evaluate.py
├── scenarios.py
├── README.md
├── requirements.txt
├── requirements-rl.txt
│
├── state/
│   └── flight_state.py
│
├── decision/
│   └── acmfds.py
│
├── controllers/
│   ├── actuation.py
│   └── data_driven_controller.py
│
├── simulation/
│   ├── acmfds_mission.py
│   ├── mission.py
│   ├── pybullet_sim.py
│   ├── drone.py
│   ├── obstacles.py
│   ├── battery.py
│   └── wind.py
│
├── telemetry/
│   ├── gps.py
│   └── logger.py
│
├── dashboard/
│   ├── server.py
│   └── static/
│       ├── index.html
│       ├── app.js
│       └── style.css
│
├── trajectory/
│   └── interpolation.py
│
├── models/
│   ├── data_driven_controller.pkl
│   └── obstacle_avoidance_model.pkl
│
├── results/
│   └── acmfds_log_<scenario>_<episode>.csv
│
└── test_env.py
```

The exact files present may vary by repository revision. Optional learned models and legacy components are not required to claim that the default ACMFDS path is a rule-based, no-RL decision system.

---

## 11. ACMFDS Objectives and Contexts

ACMFDS evaluates four objectives:

| Objective | Purpose |
|---|---|
| `SAFETY` | Maintain safety-related limits and respond to critical conditions |
| `ENERGY_EFFICIENCY` | Moderate motion according to available battery energy |
| `OBSTACLE_CLEARANCE` | Evaluate obstacle proximity and detour options |
| `MISSION_PROGRESS` | Encourage movement toward the configured destination |

The relative weights change according to the active flight context. The supplied tests check that the configured context weights sum to one.

The documented contexts include:

- `NORMAL`
- `OBSTACLE_NEAR`
- `OBSTACLE_CRITICAL`
- `LOW_BATTERY`
- `STRONG_WIND`
- `LANDING`
- `EMERGENCY`
- `MISSION_COMPLETE`

Each decision includes a human-readable reason, constraints, objective weights, and a `FlightCommand`.

Candidate evaluations can describe whether LEFT, RIGHT, FORWARD, or CLIMB is accepted or rejected. This allows the decision process to be examined using the dashboard and CSV output rather than treated as a black box.

The project's central contribution is an **explainable multi-objective flight-decision layer operating over a physics-based simulation**, supported by logged evidence and reproducible scenario tests.

---

## License and Contributions

Add the applicable license, authorship, and contribution information for your repository before public release.
