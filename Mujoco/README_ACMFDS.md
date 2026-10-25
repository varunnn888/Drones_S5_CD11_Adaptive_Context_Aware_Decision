# ACMFDS — Adaptive Context-Aware Multi-Objective Flight Decision System

**Autonomous Quadrotor Simulation | Python · MuJoCo · PySide6 · A* · Random Forest**

ACMFDS is an academic/research simulation demonstrating how an autonomous quadrotor can adapt route selection and movement to simulated obstacles, wind, battery level, contextual risk, and mission progress. It combines grid-based path planning, collision checks, candidate-route scoring, a synthetic-data Random Forest risk estimator, adaptive objective weights, simplified motion/wind/battery models, dynamic-obstacle replanning, and an interactive desktop dashboard.

> **Safety note:** This is a simulation and research project. Its heuristic risk score, simplified dynamics, battery model, and collision checks are not certified for real-world drone flight and do not guarantee physical safety.

---

## Visual Overview & Architecture

The architecture and workflow are stored as high-resolution PNG figures under `assets/` and embedded as regular Markdown images. GitHub therefore displays them as image files; this README does not use Mermaid code blocks for diagrams. The figures are conceptual illustrations, not measured experimental results.

![ACMFDS architecture showing contextual state, risk estimation, adaptive weights, route planning, multi-objective scoring, control, MuJoCo and dashboard feedback](assets/acmfds_architecture.png)

*Figure 1. ACMFDS decision and feedback architecture. The full-size source image is `assets/acmfds_architecture.png`.*

---

## Contents

1. [Overview](#1-overview)
2. [Problem statement and objectives](#2-problem-statement-and-objectives)
3. [Features](#3-features)
4. [Architecture and files](#4-architecture-and-files)
5. [Technology stack](#5-technology-stack)
6. [Simulation environment and coordinates](#6-simulation-environment-and-coordinates)
7. [Obstacle model](#7-obstacle-model)
8. [A* path planning](#8-a-path-planning)
9. [Candidate routes and multi-objective scoring](#9-candidate-routes-and-multi-objective-scoring)
10. [Machine-learning risk estimation](#10-machine-learning-risk-estimation)
11. [Adaptive objective weights](#11-adaptive-objective-weights)
12. [Motion controller and wind](#12-motion-controller-and-wind)
13. [Battery and context-aware policies](#13-battery-and-context-aware-policies)
14. [Dashboard and telemetry](#14-dashboard-and-telemetry)
15. [Runtime workflow](#15-runtime-workflow)
16. [Installation and running](#16-installation-and-running)
17. [Demonstration guide](#17-demonstration-guide)
18. [Configuration and experiments](#18-configuration-and-experiments)
19. [Validation checklist](#19-validation-checklist)
20. [Limitations](#20-limitations)
21. [Future enhancements](#21-future-enhancements)
22. [Troubleshooting](#22-troubleshooting)
23. [Academic use and license](#23-academic-use-and-license)
24. [References](#24-references)

---

## At a glance

![ACMFDS runtime decision cycle showing observation, risk estimation, adaptive weights, planning, route scoring, control, simulation and telemetry](assets/decision_cycle.png)

*Figure 2. Runtime decision-cycle illustration. It summarises the intended flow; the exact update order is defined by the implementation.*

ACMFDS observes the simulated state, estimates contextual risk, updates objective weights, plans and checks routes, selects a valid candidate, applies control and disturbance models, advances MuJoCo, and refreshes telemetry for the next cycle. The numbered figure is a conceptual workflow, not a claim that each step runs as an independent process.

---

## 1. Overview

A shortest-path planner may find a route that is not necessarily best when obstacle clearance, wind, battery state, turning difficulty, and mission time are considered together. ACMFDS evaluates route candidates against several objectives and adjusts the relative importance of those objectives according to the simulated context.

The system has two major parts:

1. **Simulation and decision logic:** vehicle state, route planning, obstacle geometry, risk estimate, adaptive weights, controller, wind, battery, and replanning.
2. **Interactive dashboard:** a desktop UI showing the arena, route, obstacles, vehicle telemetry, risk, objective weights, control outputs, and plots.

The representative initial position is $(0, 0, 2)$ metres and the default goal is $(12, 10, 2)$ metres. The environment contains static and moving obstacles. Users can change supported settings and observe how the simulation responds.

---

## 2. Problem statement and objectives

### Problem statement
An autonomous quadrotor must reach a target while conditions may change. Static obstacles constrain available paths; dynamic obstacles can move into a route; wind disturbs motion; battery depletion makes long routes less desirable; and sharp turns may be harder to track. Optimising only distance ignores these competing concerns.

### Objectives
- Simulate quadrotor motion in MuJoCo.
- Plan routes with A* and collision-aware checks.
- Generate and compare candidate route profiles.
- Score routes using safety, energy, time, tracking, and mission-detour terms.
- Estimate contextual risk with a Random Forest regressor.
- Adapt objective weights based on risk and vehicle/environment context.
- Simulate wind disturbance and battery depletion.
- Support static and sinusoidally moving obstacles.
- Replan after relevant changes and periodically for moving obstacles.
- Present state, route, risk, energy, controls, and plots in a PySide6 dashboard.

---

## 3. Features

- 3D MuJoCo simulation and interactive PySide6 dashboard.
- A* grid search with eight-connected neighbours and Euclidean heuristic.
- Collision checks, diagonal corner-cutting prevention, and line-of-sight route simplification.
- Candidate route families: `LOWER`, `LOWER-MID`, `SLIGHT-LOW`, `BALANCED`, `SLIGHT-HIGH`, `UPPER-MID`, `UPPER`.
- Multi-objective route scoring and adaptive normalised weights.
- Random Forest risk estimate trained on synthetic data.
- Static obstacles (red) and moving obstacles (orange).
- Sinusoidal dynamic-obstacle patrol.
- Wind speed and direction controls.
- Battery-aware speed, return-threshold, and emergency policies.
- Goal editing and route replanning.
- 360-degree orbit/rotation, zoom, and Overview/Top/Side camera presets.
- Telemetry for position, velocity, speed, altitude, target distance, progress, battery, clearance, risk, forces, and weights.
- GPS-like latitude/longitude derived from local coordinates for display only.

---

## 4. Architecture and files

```text
ACMFDS_Autonomous_Quadrotor/
├── app.py                 # Application entry point
├── config.py              # Shared configuration and coordinate utilities
├── data_model.py          # Risk model and adaptive weights
├── dashboard.py           # PySide6 dashboard and interactions
├── planner.py             # A*, collision checks, candidate routes, scoring
├── simulation.py          # Vehicle, wind, battery, obstacles, telemetry
├── requirements.txt       # Python dependencies
├── run_mac.sh             # macOS launch helper
├── run_windows.bat        # Windows launch helper
├── README.md              # Project documentation
└── assets/
    ├── acmfds_architecture.png
    ├── decision_cycle.png
    ├── key_equations_visual.png
    ├── multiobjective_score.png
    ├── route_candidates_illustration.png
    ├── risk_formula_curve.png
    └── adaptive_weights_vs_risk.png
```

| File | Responsibility |
|---|---|
| `app.py` | Creates and starts the Qt application and dashboard. |
| `config.py` | Shared settings, arena bounds, simulation parameters, coordinate utilities. |
| `simulation.py` | Vehicle state, motion controller, wind, battery, obstacles, route following, telemetry. |
| `planner.py` | Grid search, A*, collision/segment checks, route simplification, candidate paths, scoring. |
| `data_model.py` | Synthetic training data, Random Forest risk regression, adaptive weight calculation. |
| `dashboard.py` | UI, visualisation, plots, controls, camera interaction, supported editing. |
| `requirements.txt` | Python package dependencies. |
| `run_mac.sh` / `run_windows.bat` | Platform-specific launch helpers. |
| `assets/` | Supporting visual resources. |

Keep the modules in this relative arrangement unless their imports are updated. These descriptions reflect the project structure summarised for this version; the actual source code in your repository is authoritative if it has changed.

### High-level data flow

See Figure 1 for the image-based architecture and feedback path, and Figure 2 for the numbered runtime sequence. Both figures are files in `assets/`, so they remain visible when the repository is viewed on GitHub.

---

## 5. Technology stack

- **Python 3.12:** Recommended for the current project setup.
- **MuJoCo:** Simulation physics and 3D environment.
- **PySide6 / Qt:** Desktop UI and controls.
- **NumPy:** Numerical and vector calculations.
- **scikit-learn:** Random Forest regression.
- **Matplotlib:** Telemetry and analytical plots.

Use the active `requirements.txt` as the source of truth for dependency installation. Package versions can differ across computers.

---

## Mathematical notation and GitHub rendering

Mathematical expressions use GitHub Flavored Markdown's supported LaTeX delimiters:

- Inline expressions: `$...$`, for example `$f(n) = g(n) + h(n)$`.
- Display equations: `$$` on separate lines before and after the equation.
- Keep equations outside fenced code blocks; code fences display LaTeX as literal text instead of rendering it.
- Use balanced delimiters and standard KaTeX-supported commands. Avoid adding HTML tags inside equations.

The equations below describe the project's stated heuristic models. They are not claims of experimentally validated flight dynamics. The core equations are also collected in a graphical equation plate; the following Markdown equations remain available as rendered, copyable LaTeX.

![Graphical equation guide for A* search, route scoring, synthetic risk, adaptive-weight normalisation, motion control and the battery update](assets/key_equations_visual.png)

*Figure 3. Graphical summary of six core equations. Detailed definitions and assumptions remain in the corresponding sections below.*

---

## 6. Simulation environment and coordinates

The world uses local Cartesian coordinates in metres: X is east/positive horizontal direction, Y is north/positive horizontal direction, and Z is altitude.

| Parameter | Representative value |
|---|---:|
| Start | $(0, 0, 2)\text{ m}$ |
| Default goal | $(12, 10, 2)\text{ m}$ |
| X bounds | Approximately $[-2, 14.5]\text{ m}$ |
| Y bounds | Approximately $[-2, 12.5]\text{ m}$ |
| Physics update rate | $60\text{ Hz}$ |
| Physics timestep | $1/60\text{ s}$ |
| Simulated mass | $1.2\text{ kg}$ |
| Planner grid resolution | $0.25\text{ m}$ |
| Planner obstacle margin | $0.55\text{ m}$ |
| Representative static obstacles | $6$ |
| Representative dynamic obstacles | $2$ |

These are the described defaults; verify `config.py` if the local version has changed. Some building visuals may be decorative/non-colliding. Interpret route safety using the planner's actual obstacle representation, not visuals alone.

The initial direct distance between the default start and goal is:

$$
d = \sqrt{(12-0)^2 + (10-0)^2 + (2-2)^2} = \sqrt{244} \approx 15.62\text{ m}
$$

---

## 7. Obstacle model

### Static obstacles
Static obstacles remain at configured locations unless edited and are displayed in red. When an obstacle is changed, its MuJoCo geometry and planner representation must remain synchronised.

### Dynamic obstacles
Dynamic obstacles are displayed in orange and move along a configured X or Y axis using a sinusoidal offset:

$$
\Delta(t) = A \sin(\omega t + \phi)
$$

Here, $A$ is amplitude, $\omega$ is angular rate, $t$ is simulation time, and $\phi$ is phase. This deterministic patrol model is not a learned prediction of real-world traffic.

The planner uses obstacle margins and segment checks to reduce collisions in the model. The representative margin is $0.55\text{ m}$; it is not a certified real-drone clearance standard.

---

## 8. A* path planning

A* searches a grid from start to goal using eight-connected neighbours: horizontal, vertical, and diagonal steps.

$$
f(n) = g(n) + h(n)
$$

- $g(n)$: Accumulated cost from the start.
- $h(n)$: Heuristic estimate of remaining cost.
- $f(n)$: Node priority.

The Euclidean heuristic is:

$$
h(n) = \sqrt{(x_g - x_n)^2 + (y_g - y_n)^2}
$$

A diagonal grid move has length $\sqrt{2}$ times the grid spacing; a horizontal or vertical move has length equal to the grid spacing.

The planner includes the following checks/simplifications as described by the implementation:
1. Grid-cell obstacle and margin checks.
2. Sampled collision checks along path segments.
3. Diagonal corner-cutting prevention when adjacent cells are blocked.
4. Line-of-sight path simplification when a shortcut remains clear.

A* results depend on grid resolution, obstacle geometry, margins, and implementation details. These checks do not prove real-world safety.

---

## 9. Candidate routes and multi-objective scoring

ACMFDS compares candidate route families with different spatial preferences: `LOWER`, `LOWER-MID`, `SLIGHT-LOW`, `BALANCED`, `SLIGHT-HIGH`, `UPPER-MID`, and `UPPER`. These describe route preferences in the horizontal map; they do not necessarily mean changing altitude. A candidate must be found and pass the implemented collision checks before it should be treated as valid.

![Schematic candidate-route profiles with example static and moving obstacles](assets/route_candidates_illustration.png)

*Figure 4. Schematic route-family illustration. The obstacle locations and curves are illustrative and were not captured from a real run of the simulation; do not interpret them as validated collision-free candidates.*

The weighted route score is:

$$
J = w_s J_s + w_e J_e + w_t J_t + w_{tr} J_{tr} + w_m J_m
$$

The terms represent safety, energy, time, tracking, and mission detour. The route selector prefers a lower score.

Representative normalisations:

$$
J_s = 1 - \mathrm{clip}\left(\frac{c}{2.5}, 0, 1\right)
$$

$$
J_e = \mathrm{clip}\left(\frac{L}{16}, 0, 1\right)
$$

$$
T_{est} = \frac{L}{\max(v, 0.8)}, \qquad J_t = \mathrm{clip}\left(\frac{T_{est}}{14}, 0, 1\right)
$$

$$
J_m = \mathrm{clip}\left(\frac{L - d_{direct}}{\max(d_{direct}, \epsilon)}, 0, 1\right)
$$

Here, $c$ is the clearance metric, $L$ is route length, $v$ is the speed used for the time estimate, and $d_{direct}$ is the direct start-to-goal distance. $\mathrm{clip}(x, 0, 1)$ limits a value to $[0, 1]$; $\epsilon$ avoids division by zero. The tracking term uses a normalised turning/tracking penalty; see `planner.py` for the exact implementation.

These are heuristic normalised costs, not calibrated energy consumption, collision probability, or guaranteed flight time. Constants such as $2.5$, $16$, and $14$ are scoring scales that may require calibration for quantitative experiments.

### What the route score represents

![Equation diagram showing the total route cost and its safety, energy, time, tracking and mission-detour terms](assets/multiobjective_score.png)

*Figure 5. Multi-objective score composition. The terms are combined by their weights; the figure is conceptual and is not a graph of measured performance.*

The weights determine the relative influence of these terms. This is a **conceptual illustration**, not a graph of measured performance or a claim that one objective always dominates another.

---

## 10. Machine-learning risk estimation

`data_model.py` uses a Random Forest regressor to estimate contextual risk. The described setup uses approximately 2,500 synthetic training examples, 80 trees, maximum depth 8, and random seed 42.

### Input features
- Nearest-obstacle clearance (m).
- Vehicle speed (m/s).
- Battery percentage.
- Normalised tracking error.

### Synthetic target

$$
R_{target} = \mathrm{clip}\left( 0.62 e^{-d_o / 1.15} + 0.16 \mathrm{clip}\left(\frac{v}{2.6}, 0, 1\right) + 0.12 \left(1 - \frac{b}{100}\right) + 0.10 e_{track} + \eta, \; 0, \; 1 \right)
$$

Here, $d_o$ is nearest-obstacle clearance, $v$ is speed, $b$ is battery percentage, $e_{track}$ is normalised tracking error, and $\eta$ is synthetic noise. Check `data_model.py` for exact feature order, scaling, and noise implementation.

The learned prediction is combined with a wind term:

$$
R = \mathrm{clip}\left( 0.66 R_{ML} + 0.34 \frac{v_{wind}}{12}, 0, 1 \right)
$$

A value nearer to 1 means higher risk under this designed scale. It is **not** a calibrated probability of collision or crash. Since the labels are synthetic, real-flight generalisation has not been established.

![Formula-derived curve showing the documented synthetic target risk decreasing as obstacle clearance increases](assets/risk_formula_curve.png)

*Figure 6. Formula-derived illustration using fixed example context (speed 1.2 m/s, battery 70%, tracking error 0.15, and zero noise). It illustrates the target equation only; it is not Random Forest accuracy, validation performance, or a measured simulation result.*

---

## 11. Adaptive objective weights

The raw weights described for this implementation are:

$$
w_s^{raw} = 0.18 + 0.48 \mathrm{clip}(R, 0, 1)
$$

$$
w_e^{raw} = 0.12 + 0.30 \left(1 - \mathrm{clip}\left(\frac{b}{100}, 0, 1\right)\right)
$$

$$
w_t^{raw} = 0.07 + 0.18 \mathrm{clip}\left(\frac{d}{18}, 0, 1\right)
$$

$$
w_{tr}^{raw} = 0.14 + 0.18 \mathrm{clip}\left(\frac{v}{2.4}, 0, 1\right)
$$

$$
w_m^{raw} = 0.14 + 0.22 \mathrm{clip}\left(\frac{t_r}{150}, 0, 1\right)
$$

$R$ is risk, $b$ is battery percentage, $d$ is target distance, $v$ is speed, and $t_r$ is remaining-time context. The raw weights are normalised:

$$
w_i = \frac{w_i^{raw}}{\sum_j w_j^{raw}}, \qquad \sum_i w_i = 1
$$

Interpretation: higher risk raises safety emphasis; lower battery raises energy emphasis; target distance influences time emphasis; speed influences tracking emphasis; remaining-time context influences mission emphasis. The live simulation can further increase energy emphasis under low battery or high wind and then renormalise. Check `data_model.py` and `simulation.py` for the exact update order. These are hand-designed policy weights, not learned optimal weights.

![Formula-derived graph of the normalised objective weights while contextual risk changes](assets/adaptive_weights_vs_risk.png)

*Figure 7. Formula illustration with battery, target distance, speed and remaining-time context held fixed at the values shown inside the graph. The plot demonstrates the documented normalisation equations; it excludes any additional live-simulation modifiers and is not a measured run.*

---

## 12. Motion controller and wind

A representative controller uses position error to calculate desired velocity and acceleration:

$$
\mathbf{e} = \mathbf{p}_{target} - \mathbf{p}
$$

$$
\mathbf{v}_{desired} = 1.65 s_{scale} \mathbf{e}
$$

$$
\mathbf{a}_{cmd} = 2.8 (\mathbf{v}_{desired} - \mathbf{v})
$$

The implementation applies command/speed limits. The speed scale depends on wind and battery:

$$
s_{wind} = \max(0.62, 1 - 0.032 v_{wind})
$$

$$
s_{battery} = \begin{cases} 1, & b \ge 35 \\ 0.62 + 0.38 \frac{b}{35}, & b < 35 \end{cases}
$$

$$
s_{scale} = \min(s_{wind}, s_{battery})
$$

### Wind direction and force

| Direction | Horizontal force direction |
|---:|---|
| $0^\circ$ | $+X$ |
| $90^\circ$ | $+Y$ |
| $180^\circ$ | $-X$ |
| $270^\circ$ | $-Y$ |

$$
\theta = \theta_{deg} \frac{\pi}{180}, \qquad a_{wind} = 0.18 v_{wind}
$$

$$
\mathbf{F}_{wind} = m a_{wind} \begin{bmatrix} \cos\theta \\ \sin\theta \\ 0 \end{bmatrix}
$$

The described mass is $1.2\text{ kg}$. This is a simplified disturbance model; it does not reproduce full aerodynamic drag, turbulence, rotor dynamics, or a validated wind field.

---

## 13. Battery and context-aware policies

The described battery model uses a speed-dependent cost and a wind multiplier:

$$
C_{speed} = 0.48 + 0.14 \min\left(\frac{v}{1.6}, 1\right)
$$

$$
M_{wind} = 1 + 0.025 v_{wind}^{1.25}
$$

$$
b_{new} = \max(0, b_{old} - C_{speed} M_{wind} \Delta t)
$$

Battery is a percentage-like simulated state. This is a demonstration model, not a measured battery or endurance model.

Representative policies described for this version:
- At battery $35\%$ or lower, reduce the speed scale.
- Return threshold: approximately $b_{\mathrm{return}} \approx \min(43,\,25 + 1.5v_{\mathrm{wind}})\%$.
- At battery $8\%$ or lower, invoke the emergency-landing policy at the current horizontal location.
- Wind around $8\text{ m/s}$ or higher is treated as a high-wind context.
- Replanning occurs after relevant changes and periodically for moving obstacles (approximately every 3 seconds in the described configuration).

These thresholds are simulation policies, not real-aircraft operational recommendations. Verify exact behaviours and labels in source before reporting them as exact runtime states.

### High-level decision pipeline
1. Read vehicle and environment state.
2. Calculate contextual features.
3. Predict risk and include the wind term.
4. Compute and normalise adaptive objective weights.
5. Generate or refresh route candidates when needed.
6. Check candidates and calculate their costs.
7. Select the preferred valid route.
8. Follow the active waypoint with the controller.
9. Apply wind and battery updates.
10. Update telemetry and replan when triggered.

---

## 14. Dashboard and telemetry

For screenshots or exported plots added to the repository, use a high-resolution image, descriptive axis labels, readable tick labels, units, a legend where needed, and a caption stating the scenario/settings. Avoid screenshots where text is too small to read. Do not present illustrative diagrams as experimental results.

Representative panels include:
- Simulation status.
- Quadrotor position, velocity, speed, and altitude.
- Target, distance, mission progress, and waypoint.
- Nearest obstacle, clearance, and risk.
- Battery and energy information.
- Control outputs ($F_x$, $F_y$, $F_z$, and total force).
- Top-down map showing start, goal, obstacles, vehicle, and route.
- Candidate route profiles and score information.
- Obstacle controls (type, ID, X/Y, dynamic axis/range/rate).
- Wind speed and direction.
- Distance-to-target, battery, and speed plots.
- Current adaptive objective weights.

Representative colours: blue start, green goal, red static obstacles, orange dynamic obstacles, cyan planned route. The interface also supports 3D orbit/rotation, zoom, and Overview/Top/Side views. Exact panel labels depend on the current dashboard source.

### GPS-like coordinates
Latitude/longitude are derived from local XY coordinates using an equirectangular-style approximation. Conceptually, around reference latitude $\varphi_0$ and Earth-radius approximation $R_E$:

$$
\Delta\varphi \approx \frac{\Delta y}{R_E}, \qquad \Delta\lambda \approx \frac{\Delta x}{R_E \cos\varphi_0}
$$

These are display conversions, not GPS sensor readings or a real geographic navigation solution. See `config.py` for the actual reference point and units.

---

## 15. Runtime workflow

The conceptual update cycle is:
1. Read simulation state and time.
2. Update dynamic obstacle positions.
3. Update contextual measurements.
4. Estimate risk.
5. Calculate adaptive weights.
6. Check whether replanning is required.
7. Generate and collision-check route candidates if required.
8. Select the preferred valid route.
9. Choose the active waypoint.
10. Compute motion commands.
11. Apply wind disturbance.
12. Advance MuJoCo.
13. Update battery state.
14. Calculate telemetry and mission progress.
15. Refresh dashboard widgets and plots.

The exact update order is defined by the implementation and may differ slightly from this conceptual overview.

---

## 16. Installation and running

### Prerequisites
- Python 3.12 is recommended for the current setup.
- Git if cloning the repository.
- A supported desktop operating system and graphics setup for MuJoCo/Qt.
- A virtual environment.

### Clone the repository
Replace `YOUR-USERNAME/YOUR-REPOSITORY` with the actual GitHub owner and repository name before running the command:

```bash
git clone https://github.com/YOUR-USERNAME/YOUR-REPOSITORY.git
cd ACMFDS_Autonomous_Quadrotor
```

Or open your existing project folder in VS Code.

### Create a virtual environment

**macOS / Linux**
```bash
python3.12 -m venv .venv
source .venv/bin/activate
```

**Windows PowerShell**
```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
```

### Install dependencies (use the repository's `requirements.txt`)
Run these commands from the **project root**, with the virtual environment activated:

```bash
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip check
```

`requirements.txt` is the authoritative dependency list for this checkout. Do not copy package versions from another computer: use the file committed with the source code. If `requirements.txt` is missing or incomplete, the technology stack documented here uses the package names `mujoco`, `PySide6`, `numpy`, `scikit-learn`, and `matplotlib`. As a temporary bootstrap only, install them with `python -m pip install mujoco PySide6 numpy scikit-learn matplotlib`, verify that the application launches, then update `requirements.txt` with the versions tested in that environment. This README does not claim an unverified set of package pins.

Confirm that the active interpreter is the virtual environment's interpreter:

```bash
python --version
python -m pip --version
python -c "import mujoco, numpy, sklearn, matplotlib, PySide6; print('Core imports OK')"
```

The last command checks common core imports only; it does not replace launching the full GUI. If this project version imports Matplotlib or other packages, ensure those are also listed in `requirements.txt`.

Run from the project root (the folder containing `app.py`):

```bash
python app.py
```

On macOS, use the launcher if it is configured:

```bash
bash run_mac.sh
```

On Windows Command Prompt:

```bat
run_windows.bat
```

If a launcher fails, activate `.venv` and run `python app.py` directly to see the underlying error.

### Syntax check

```bash
python -m py_compile app.py config.py dashboard.py data_model.py planner.py simulation.py
```

No output normally means the listed files compiled. This does not prove the GUI, physics simulation, planner, or runtime behaviour passed functional tests.

---

## 17. Demonstration guide

1. Launch the app and verify the dashboard opens.
2. Identify the start, goal, static obstacles, dynamic obstacles, and planned route.
3. Observe position, speed, target distance, risk, battery, and adaptive weights.
4. Move the goal using the supported interaction and check that the route updates.
5. Change a supported obstacle parameter and verify the visual and planning representations remain synchronised.
6. Observe dynamic patrol and route replanning.
7. Change wind speed/direction and observe movement and context metrics.
8. Monitor battery and note when configured low-battery policies activate.
9. Rotate/zoom the camera or use view presets.
10. Record scenario settings, route, clearance, risk, weights, battery trend, warnings, and outcome for the report.

Do not claim mission success unless the simulation actually reaches its completion condition.

---

## 18. Configuration and experiments

Inspect `config.py` for shared constants, `planner.py` for route/collision logic, `data_model.py` for risk/weights, and `simulation.py` for motion, wind, battery, obstacles, and telemetry.

Settings worth studying include start/goal, arena bounds, timestep, mass, controller limits, grid resolution, obstacle margin, obstacle geometry and patrol settings, wind, initial battery, policy thresholds, route families, score normalisation, Random Forest settings, and feature scaling.

Change one parameter at a time, record a baseline, and repeat time-dependent scenarios. Keep MuJoCo geometry and the planner obstacle list synchronised.

### Suggested experiment log

| Field | Record |
|---|---|
| Scenario ID | Unique test label |
| Start/goal | Local XYZ coordinates |
| Obstacles | Type, geometry, position, patrol settings |
| Wind | Speed and direction |
| Initial battery | Starting percentage |
| Route | Candidate and selected family |
| Clearance | Planner-reported metric |
| Risk | Estimated risk |
| Weights | Safety, energy, time, tracking, mission |
| Outcome | Completed, incomplete, or failed |
| Notes | Warnings and observations |

### Results graphs and reporting

The README source supplied for this revision did not include saved MuJoCo run logs, exported dashboard screenshots, or benchmark tables. For that reason, this document does **not** claim a mission-success rate, average route length, collision rate, energy saving, execution time, or Random Forest validation score. Figures 4, 6 and 7 are labelled schematic or formula-derived illustrations, not empirical results.

For the final project report, run the simulation, export readable dashboard screenshots, and record actual values for each scenario. Use the following as a reporting checklist; fill it with measured or directly logged values rather than estimates:

| Result to report | What to record | Evidence source |
|---|---|---|
| Mission outcome | Completed / incomplete / failed and the configured completion condition | Simulation status or run log |
| Route result | Selected route family, route length, number of replans | Planner output or added telemetry logging |
| Clearance | Minimum planner-reported clearance and obstacle configuration | Planner diagnostics; verify units and definition |
| Time | Simulated mission duration and wall-clock runtime, kept separate | Simulation clock and timed run |
| Battery | Initial and final simulated battery state, plus modelled change | Dashboard telemetry / run log |
| Context | Wind speed/direction, obstacle settings, start and goal | Scenario settings captured before each run |
| Decision values | Risk score and all normalised objective weights over the run | Dashboard telemetry or exported logs |
| ML evaluation | Held-out synthetic-data metrics, split protocol and random seed | Separate model evaluation; do not infer from the risk graph |
| Reproducibility | Python, MuJoCo, scikit-learn and other dependency versions | `python --version` and `python -m pip freeze` |

If the current dashboard does not expose one of these values, instrument or export it before reporting it. Repeat each scenario consistently and compare ACMFDS against a defined baseline before claiming that adaptive weights improve performance.

---

## 19. Validation checklist

These are recommended tests, not a claim that every test has already passed.

- [ ] Dependencies install in the intended virtual environment.
- [ ] `app.py` launches and the dashboard refreshes.
- [ ] A clear start-to-goal route is found.
- [ ] Static obstacles are avoided by planner checks.
- [ ] Diagonal corner cutting is rejected when applicable.
- [ ] Segment checks and simplification preserve route clearance.
- [ ] Goal movement triggers replanning.
- [ ] Dynamic obstacle state updates in both simulation and planner.
- [ ] Risk remains within $[0, 1]$.
- [ ] Adaptive weights are finite and sum to approximately one.
- [ ] Wind changes the configured disturbance/context.
- [ ] Battery decreases according to the model.
- [ ] Low-battery/emergency policies execute as implemented.
- [ ] Telemetry and plots update.
- [ ] Mission completion is reported only when the configured condition is met.

---

## 20. Limitations

1. The Random Forest is trained on synthetic, formula-generated data, not representative real-flight measurements.
2. Risk is not a calibrated collision probability.
3. Controller and wind force are simplified models, not a validated flight-dynamics stack.
4. The battery equation is not based on real battery chemistry or measured power.
5. A* behaviour depends on grid resolution, geometry, and margins.
6. Periodic replanning does not guarantee avoidance of arbitrary moving obstacles.
7. GPS-like coordinates are derived values, not GPS measurements.
8. Some building visuals may be decorative and non-colliding.
9. Route-cost scales are heuristic and need calibration for scientific comparisons.
10. Candidate scoring does not prove global multi-objective optimality.
11. Platform graphics and launch behaviour can vary by OS/Python/MuJoCo/Qt version.
12. The project has not been certified or validated for real-world flight.

---

## 21. Future enhancements

- Train and evaluate risk models on measured or high-fidelity simulation data.
- Add train/test separation, cross-validation, regression metrics, and uncertainty analysis.
- Calibrate energy cost using power, thrust, velocity, and battery characteristics.
- Improve dynamics and control with a validated quadrotor model.
- Add time-aware prediction for moving obstacles.
- Explore model predictive control or other receding-horizon methods.
- Add automated scenarios, repeatable benchmarks, and telemetry export.
- Compare against baseline planners and fixed objective weights.
- Add unit tests for geometry, collision checking, weight normalisation, and battery logic.
- Improve experiment replay and side-by-side comparisons.
- Separate visual-only geometry from collision geometry explicitly.

These are potential extensions, not features claimed to exist in the current version.

---

## 22. Troubleshooting

### `ModuleNotFoundError`
Activate the correct virtual environment and install dependencies:

```bash
source .venv/bin/activate
python -m pip install -r requirements.txt
```

On Windows PowerShell:

```powershell
.\.venv\Scripts\Activate.ps1
```

### `ImportError: cannot import name 'clearance_at' from 'planner'`
The caller expects a function that may be missing, renamed, or not exported by the current `planner.py`.
1. Search `planner.py` for the expected function.
2. Search `simulation.py` and other modules for its imports and expected arguments.
3. Align the function definition and caller with the actual API.
4. Check for circular imports.
5. Run the syntax check above.

Do not add a stub that returns a constant; the function must implement the clearance calculation expected by its caller.

### MuJoCo or Qt window fails to open
- Confirm the active Python interpreter and virtual environment.
- Read the complete terminal traceback.
- Check installed MuJoCo/PySide6 versions and graphics support.
- Follow the viewer launch method used by the current code. Do not assume `mjpython` is required unless the source or error indicates it.

### Route does not change after editing
Check whether the UI callback updates the simulation state, the planner obstacle list, the replan flag, and the displayed route. A visual-only update does not necessarily update the collision model.

### Dependency conflicts
```bash
python --version
python -m pip --version
python -m pip list
```

Prefer a clean virtual environment over mixing system Python packages with project dependencies.

---

## README pre-submission checklist

Before submitting or publishing this repository, verify the following items against the actual checkout:

- [ ] The README is saved as `README.md` in the repository root (GitHub automatically displays this file).
- [ ] Every referenced image exists at the exact relative path under `assets/`; the architecture, workflow and score diagrams are PNG assets rather than Mermaid code blocks.
- [ ] `requirements.txt` lists the packages imported by the project and has been tested in a clean Python 3.12 virtual environment.
- [ ] The installation and launch commands work from the documented project root on the target operating system.
- [ ] All equations render on the GitHub repository page, not just in the local Markdown preview.
- [ ] Any reported metrics are backed by recorded experiment outputs; illustrative diagrams are not presented as results.
- [ ] The repository contains a selected `LICENSE` file if you intend to distribute the code publicly.
- [ ] Replace the example GitHub clone URL with the real repository URL wherever it is used.

This README has been reviewed for Markdown structure, image paths and LaTeX delimiter consistency. The uploaded material did not include the complete source checkout or recorded simulation results, so package versions, GUI launch, runtime behaviour and empirical performance still need to be verified locally. Run the validation checklist in Section 19 before submission.

---

## 23. Academic use and license

ACMFDS can demonstrate autonomous navigation, A* search, collision-aware planning, multi-objective decisions, context-aware adaptation, regression-based risk scoring, simulated environmental disturbances, battery-aware policies, and telemetry visualisation.

For academic reporting, distinguish simulated outcomes from physical measurements, record configuration and package versions, disclose that the risk model uses synthetic labels, and do not present heuristic risk as a real collision probability.

**License:** Add the license selected for your repository and a corresponding `LICENSE` file before public distribution. This README does not assume a license has already been selected.

---

## Project summary

ACMFDS combines A* path planning, collision checks, candidate-route scoring, a synthetic-data Random Forest risk estimate, adaptive objective weights, simplified wind and battery models, dynamic-obstacle replanning, and a PySide6 dashboard around a MuJoCo quadrotor simulation. Its purpose is to demonstrate a modular context-aware decision pipeline in a controllable environment. Results should be interpreted as simulation/research outputs, not proof of real-world flight safety.

---

## 24. References

The references below document the principal algorithms and software frameworks discussed in this README. The project-specific equations, thresholds, synthetic labels, and implementation details should be cited to the repository source files when reporting this implementation; they are not attributed to these external sources.

1. Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). “A Formal Basis for the Heuristic Determination of Minimum Cost Paths.” *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. https://doi.org/10.1109/TSSC.1968.300136
2. Breiman, L. (2001). “Random Forests.” *Machine Learning*, 45, 5–32. https://doi.org/10.1023/A:1010933404324
3. Todorov, E., Erez, T., & Tassa, Y. (2012). “MuJoCo: A Physics Engine for Model-Based Control.” In *2012 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 5026–5033. https://doi.org/10.1109/IROS.2012.6386109
4. Google DeepMind. (n.d.). *MuJoCo Documentation*. https://mujoco.readthedocs.io/
5. The scikit-learn developers. (n.d.). *RandomForestRegressor documentation*. https://scikit-learn.org/stable/modules/generated/sklearn.ensemble.RandomForestRegressor.html
6. The NumPy developers. (n.d.). *NumPy documentation*. https://numpy.org/doc/stable/
7. The Matplotlib developers. (n.d.). *Matplotlib documentation*. https://matplotlib.org/stable/
8. Qt Group. (n.d.). *Qt for Python (PySide6) documentation*. https://doc.qt.io/qtforpython-6/
9. Python Software Foundation. (n.d.). *Python 3.12 documentation*. https://docs.python.org/3.12/
10. GitHub. (n.d.). *Writing mathematical expressions*. GitHub Docs. https://docs.github.com/en/get-started/writing-on-github/working-with-advanced-formatting/writing-mathematical-expressions
11. GitHub. (n.d.). *Creating diagrams*. GitHub Docs. https://docs.github.com/en/get-started/writing-on-github/working-with-advanced-formatting/creating-diagrams

### Citation and reproducibility notes
- Cite A* and Random Forest using the original publications above when discussing the general methods.
- Cite MuJoCo and PySide6 documentation when discussing their software APIs; include the versions actually used in the experiment environment.
- Record the Python and dependency versions used for each experiment (for example, with `python --version` and `python -m pip freeze`).
- Add citations for any additional algorithm, dataset, or paper actually used by the project. Do not list a paper as a project source unless it was consulted and is relevant.
- The equations and policy constants in this README describe the project implementation and are not validated physical models.
