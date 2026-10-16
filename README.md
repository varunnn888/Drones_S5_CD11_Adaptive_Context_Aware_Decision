# ACMFDS: Adaptive Context-Aware Multi-Objective Flight Decision System

### A Gymnasium-Compatible, PyBullet-Based Quadcopter Simulation Framework

![Python](https://img.shields.io/badge/Python-3.10%2B-blue)
![Simulation](https://img.shields.io/badge/Physics-PyBullet-orange)
![Environment](https://img.shields.io/badge/API-Gymnasium-green)
![Status](https://img.shields.io/badge/Project-Research%20Prototype-purple)

## Abstract

Autonomous unmanned aerial vehicles (UAVs) operate in environments characterized by dynamic obstacles, uncertain flight conditions, limited energy resources, and competing mission objectives. Reliable autonomous navigation therefore requires a systematic approach to flight-state estimation, decision-making, control, and safety evaluation.

This project presents the **Adaptive Context-Aware Multi-Objective Flight Decision System (ACMFDS)**, a software framework for autonomous quadcopter simulation and context-dependent flight decision-making. The project combines a PyBullet-based rigid-body simulation environment with a Gymnasium-compatible Markov Decision Process (MDP) interface. ACMFDS provides an explainability layer that classifies flight conditions, assigns context-dependent objective priorities, and records the selected decision and its rationale.

The simulation models translational and rotational dynamics, bounded thrust and torque inputs, obstacle proximity through virtual range sensing, and reward-based evaluation of goal progress, attitude, control effort, and safety-related events. The Gymnasium interface supports experimentation with reinforcement-learning algorithms, while the default mission pipeline remains a separate execution path.

The framework is intended for research, educational experimentation, algorithm testing, and reproducible evaluation of autonomous flight behavior. It is a simulation-based prototype and does not claim validation on physical aircraft.

**Keywords:** Autonomous UAVs, quadcopter, ACMFDS, PyBullet, Gymnasium, reinforcement learning, multi-objective optimization, obstacle avoidance, rigid-body dynamics, flight safety.

---

## 1. Introduction

Quadcopters are widely studied platforms for autonomous navigation because of their maneuverability and ability to operate in three-dimensional environments. However, autonomous flight requires balancing multiple objectives, including:

- **Safety:** reducing collision risk and maintaining acceptable obstacle clearance.
- **Energy efficiency:** limiting unnecessary control effort and conserving battery resources.
- **Time efficiency:** progressing toward a destination within the available mission duration.
- **Trajectory tracking:** maintaining consistency with a desired flight path.
- **Mission completion:** reaching the specified goal while satisfying applicable constraints.

These objectives can conflict. For example, a direct route may reduce travel time but increase obstacle exposure, whereas a longer route may improve clearance at the expense of energy and time.

ACMFDS addresses this decision-making problem through context-aware classification and objective prioritization. The simulation provides a controlled environment in which flight behavior and algorithm outputs can be inspected and evaluated.

### 1.1 Project objectives

The principal objectives are:

1. Develop a three-dimensional quadcopter simulation using PyBullet.
2. Represent the flight task as a Gymnasium-compatible MDP.
3. Define continuous observation and action spaces for the simulated vehicle.
4. Model rigid-body motion using forces, torques, gravity, and drag.
5. Provide virtual range sensing for obstacle-proximity assessment.
6. Evaluate goal progress, control effort, attitude, and safety events through a reward function.
7. Integrate ACMFDS decision outputs as an explainability layer.
8. Support environment validation and optional reinforcement-learning experiments.

---

## 2. Software Architecture

The project separates physical simulation, reinforcement-learning interaction, and high-level decision interpretation.

### 2.1 PyBullet physics engine

PyBullet provides the rigid-body physics simulation. The drone responds to applied forces and torques, and the engine updates its position, velocity, orientation, and angular velocity.

The physical transition is determined by the engine rather than by directly assigning a scripted trajectory.

### 2.2 Gymnasium environment

The `DroneEnv` class exposes the simulation through the Gymnasium interface. It defines:

- An observation space containing vehicle state, goal information, and virtual sensor readings.
- A continuous action space containing normalized thrust and torque commands.
- A reward function for evaluating actions.
- Reset and step operations.
- Termination and truncation conditions.
- Rendering and environment cleanup.

The environment can be used for reinforcement-learning experiments and is checked using Gymnasium's environment validation utilities.

### 2.3 ACMFDS decision layer

ACMFDS classifies flight conditions and provides contextual decision information. Its outputs include the selected context, decision, and explanation.

In the Gymnasium environment, these outputs are included in the `info` dictionary for interpretation and logging. **They do not directly control the vehicle in this environment path**; the action supplied to `DroneEnv.step()` determines the applied control input.

The default mission runner is a separate pipeline in which ACMFDS decisions can influence or override mission commands according to the mission runner's decision-composition rules.

### 2.4 Overall workflow

```text
Mission / Environment Configuration
              |
              v
       PyBullet Simulation
              |
              v
   Vehicle State and Ray Sensing
              |
              v
      Gymnasium Observation
              |
              v
    External Policy / Controller
              |
              v
     Normalized Action Vector
              |
              v
      Force and Torque Mapping
              |
              v
       PyBullet Physics Step
              |
              v
  Reward, Termination and Telemetry

ACMFDS Decision Layer
       |
       +----> Context, Decision and Reason
               (Gymnasium info / explainability)
```

---

## 3. Mathematical Formulation

### 3.1 Markov Decision Process

The reinforcement-learning environment is represented by the tuple

$$
\mathcal{M}
=
\left(
\mathcal{S},
\mathcal{A},
P,
r,
\gamma,
\mathcal{D}
\right),
$$

where:

- \(\mathcal{S}\) is the observation space.
- \(\mathcal{A}\) is the action space.
- \(P(s_{k+1}\mid s_k,a_k)\) is the transition distribution.
- \(r(s_k,a_k,s_{k+1})\) is the reward function.
- \(\gamma\) is the discount factor used by an RL algorithm.
- \(\mathcal{D}\) is the initial-state distribution.

The physical transition is produced by the PyBullet simulation:

$$
s_{k+1}
=
F_{\mathrm{PyBullet}}
\left(
s_k,a_k,\Delta t_{\mathrm{ctrl}}
\right).
$$

Here, \(F_{\mathrm{PyBullet}}\) represents the numerical rigid-body simulation and associated contact calculations.

The environment advances in discrete control steps.

| Parameter | Symbol | Configured value |
|---|---|---:|
| Physics timestep | \(\Delta t_{\mathrm{phys}}\) | \(1/120\) s |
| Physics substeps per control step | \(n_{\mathrm{sub}}\) | 2 |
| Control period | \(\Delta t_{\mathrm{ctrl}}\) | \(1/60\) s |
| Maximum episode length | \(k_{\max}\) | 500 control steps |
| Nominal episode duration | \(T_{\max,\mathrm{episode}}\) | Approximately 8.33 s |
| Physics solver iterations | — | 10 |

The episode duration follows

$$
\Delta t_{\mathrm{ctrl}}
=
n_{\mathrm{sub}}\Delta t_{\mathrm{phys}}
=
\frac{1}{60}\ \mathrm{s},
$$

and

$$
T_{\mathrm{episode}}
=
k_{\max}\Delta t_{\mathrm{ctrl}}
=
\frac{500}{60}
\approx 8.33\ \mathrm{s}.
$$

These values describe the configured environment, not a validated real-world flight duration.

### 3.2 Rigid-body dynamics

The simulated vehicle is modeled as a rigid body with translational and rotational dynamics.

The translational equation is

$$
m\dot{\mathbf{v}}
=
T\hat{\mathbf{b}}_3
+
\mathbf{F}_{\mathrm{drag}}
+
m\mathbf{g}
+
\mathbf{F}_{\mathrm{contact}},
$$

where:

- \(m\) is the vehicle mass.
- \(\mathbf{v}\) is the world-frame linear velocity.
- \(T\) is the commanded thrust magnitude.
- \(\hat{\mathbf{b}}_3\) is the vehicle's body-frame thrust direction expressed in world coordinates.
- \(\mathbf{g}\) is the gravitational acceleration vector.
- \(\mathbf{F}_{\mathrm{contact}}\) represents forces arising from physical contact.

The rotational dynamics are expressed as

$$
\mathbf{I}\dot{\boldsymbol{\omega}}
+
\boldsymbol{\omega}
\times
\left(
\mathbf{I}\boldsymbol{\omega}
\right)
=
\boldsymbol{\tau}_{\mathrm{cmd}}
+
\boldsymbol{\tau}_{\mathrm{drag}},
$$

where \(\mathbf{I}\) is the inertia tensor, \(\boldsymbol{\omega}\) is the angular velocity, and \(\boldsymbol{\tau}_{\mathrm{cmd}}\) is the commanded torque.

The configured linear and angular drag models are

$$
\mathbf{F}_{\mathrm{drag}}
=
-c_v\mathbf{v},
$$

$$
\boldsymbol{\tau}_{\mathrm{drag}}
=
-c_\omega\boldsymbol{\omega}.
$$

The drone receives thrust along its body-frame vertical axis. Tilting the vehicle therefore changes the direction of thrust and produces horizontal acceleration without requiring a separately commanded lateral force.

#### Physical parameters

| Parameter | Symbol | Value |
|---|---|---:|
| Vehicle mass | \(m\) | 1.0 kg |
| Gravitational acceleration | \(g\) | 9.81 m/s² |
| Nominal hover thrust | \(T_h=mg\) | 9.81 N |
| Minimum thrust | \(T_{\min}\) | 0 N |
| Maximum thrust | \(T_{\max}\) | 20 N |
| Roll/pitch torque limit | \(\tau_{\max}\) | 1.5 N·m |
| Yaw torque limit | \(\tau_{\max}^{\mathrm{yaw}}\) | 0.75 N·m |
| Arm length | \(\ell\) | 0.25 m |
| Linear drag coefficient | \(c_v\) | 0.25 N·s/m |
| Angular drag coefficient | \(c_\omega\) | 0.15 N·m·s |

These parameters are software configuration values for the simulated model.

---

## 4. Action Space

The environment defines a four-dimensional continuous action space:

$$
\mathcal{A}=[-1,1]^4.
$$

The action vector is

$$
\mathbf{a}_k
=
\begin{bmatrix}
a_T &
a_\phi &
a_\theta &
a_\psi
\end{bmatrix}^{\mathsf T},
$$

where the entries represent normalized thrust, roll torque, pitch torque, and yaw torque commands.

### 4.1 Thrust mapping

The neutral thrust is set to the nominal hover thrust,

$$
T_h=mg=9.81\ \mathrm{N}.
$$

The normalized thrust command is mapped to physical thrust using a piecewise function:

$$
T(a_T)=
\begin{cases}
T_h+a_T(T_{\max}-T_h),
& a_T\geq 0,\\[6pt]
T_h+a_T(T_h-T_{\min}),
& a_T<0.
\end{cases}
$$

With the configured parameters,

$$
T(a_T)=
\begin{cases}
9.81+10.19a_T, & a_T\geq0,\\
9.81+9.81a_T, & a_T<0.
\end{cases}
$$

Consequently,

$$
T(-1)=0,\qquad
T(0)=9.81,\qquad
T(1)=20\ \mathrm{N}.
$$

The mapping is asymmetric around the hover point. A neutral normalized thrust action corresponds to nominal hover thrust, not half of the maximum thrust.

### 4.2 Torque mapping

The torque commands are defined by

$$
\tau_{\mathrm{roll}}=1.5a_\phi,
$$

$$
\tau_{\mathrm{pitch}}=1.5a_\theta,
$$

$$
\tau_{\mathrm{yaw}}=0.75a_\psi,
$$

with each action component constrained to \([-1,1]\).

The final thrust command is clipped to its configured physical range.

---

## 5. Observation Space

The Gymnasium observation space is a 27-dimensional `float32` vector.

The observation at control step \(k\) is

$$
\mathbf{o}_k
=
\left[
\mathbf{p},
\mathbf{v},
\boldsymbol{\eta},
\boldsymbol{\omega},
\mathbf{p}_g,
\Delta\mathbf{p}_g,
d_g,
\boldsymbol{\rho}
\right],
$$

where the components are defined below.

| Indices | Variable | Description | Unit |
|---|---|---|---|
| 0–2 | \(\mathbf{p}\) | Position \((x,y,z)\) | m |
| 3–5 | \(\mathbf{v}\) | Linear velocity | m/s |
| 6–8 | \(\boldsymbol{\eta}\) | Roll, pitch and yaw | rad |
| 9–11 | \(\boldsymbol{\omega}\) | Angular velocity | rad/s |
| 12–14 | \(\mathbf{p}_g\) | Goal position | m |
| 15–17 | \(\Delta\mathbf{p}_g\) | Relative goal vector | m |
| 18 | \(d_g\) | Euclidean goal distance | m |
| 19–26 | \(\boldsymbol{\rho}\) | Eight normalized range-sensor readings | Dimensionless |

The relative goal vector is

$$
\Delta\mathbf{p}_g
=
\mathbf{p}_g-\mathbf{p}.
$$

The Euclidean distance to the goal is

$$
d_g
=
\left\|
\Delta\mathbf{p}_g
\right\|_2
=
\sqrt{
(x_g-x)^2+
(y_g-y)^2+
(z_g-z)^2
}.
$$

Observation components are clipped to their declared bounds before being returned. The components are expressed in physically meaningful units rather than all being normalized to \([-1,1]\). Algorithms that are sensitive to different input scales may therefore benefit from an explicit observation-normalization wrapper.

---

## 6. Virtual Obstacle Sensing

The simulated drone uses eight virtual range-sensor rays to measure obstacle proximity.

The sensor directions include:

1. Front
2. Back
3. Left
4. Right
5. Up
6. Down
7. Front-left
8. Front-right

The rays are defined in the vehicle body frame and rotated into world coordinates using the vehicle rotation matrix \(R(\boldsymbol{\eta})\).

For sensor direction \(\hat{\mathbf{d}}_i\), the ray origin and endpoint are

$$
\mathbf{q}_i^+
=
\mathbf{p}
+
r_{\mathrm{off}}R\hat{\mathbf{d}}_i,
$$

$$
\mathbf{q}_i^\times
=
\mathbf{p}
+
d_{\max}R\hat{\mathbf{d}}_i,
$$

where the configured origin offset and maximum range are

$$
r_{\mathrm{off}}=0.35\ \mathrm{m},
\qquad
d_{\max}=5.0\ \mathrm{m}.
$$

PyBullet's ray-test result supplies a segment fraction \(f_i\in[0,1]\). The observation stores this fraction:

$$
\rho_i=f_i.
$$

The corresponding estimated range is

$$
\tilde d_i=d_{\max}f_i=5f_i.
$$

The minimum range is

$$
d_{\min}
=
\min_{i\in\{0,\ldots,7\}}
\tilde d_i.
$$

A value of \(5.0\) m indicates that no intersection was detected within the configured sensor range.

Because the ray begins at an offset from the vehicle position, the true distance from the vehicle reference point along a detected ray is

$$
d_i^{\mathrm{true}}
=
r_{\mathrm{off}}
+
(d_{\max}-r_{\mathrm{off}})f_i.
$$

The difference between this distance and the reported linear estimate is

$$
\delta_i
=
d_i^{\mathrm{true}}-\tilde d_i
=
r_{\mathrm{off}}(1-f_i).
$$

Thus, the reported range underestimates the corresponding distance by at most \(0.35\) m under this geometric model. This is a conservative bias for a proximity penalty, although it does not replace comprehensive collision-safety validation.

---

## 7. Reward Function

The environment uses an additive, manually calibrated reward function to evaluate goal progress, flight stability, control effort, obstacle proximity, and termination events.

The total reward is

$$
\begin{aligned}
r_k={}&
r_{\mathrm{prog}}
+r_{\mathrm{goal}}
+r_{\mathrm{alive}}
+r_z
+r_{\mathrm{tilt}}\\
&+r_u
+r_{\mathrm{prox}}
+r_{\mathrm{term}}
+r_{\mathrm{time}}.
\end{aligned}
$$

### 7.1 Goal-progress reward

Let \(d_{k-1}\) be the previous goal distance and \(d_k\) the current goal distance.

The progress term is

$$
r_{\mathrm{prog}}
=
15(d_{k-1}-d_k).
$$

Positive reward is obtained when the drone reduces its distance to the goal. Moving away from the goal produces a negative contribution.

### 7.2 Goal-reaching reward

The goal-reaching reward is

$$
r_{\mathrm{goal}}
=
100\,\mathbb{1}[d_k<r_{\mathrm{tgt}}],
$$

where

$$
r_{\mathrm{tgt}}=0.40\ \mathrm{m}.
$$

Here, \(\mathbb{1}[\cdot]\) denotes the indicator function.

### 7.3 Altitude and attitude penalties

The altitude-error penalty is

$$
r_z=-0.75e_z^2,
$$

where

$$
e_z=z-p_{g,z}.
$$

The roll and pitch penalty is

$$
r_{\mathrm{tilt}}
=
-0.4(\phi^2+\theta^2).
$$

Yaw is not penalized by this term because the specified task does not require a particular heading.

### 7.4 Control-effort penalty

The control-effort penalty is

$$
r_u=-0.01\|\mathbf{a}_k\|_2^2.
$$

This term discourages unnecessarily large action magnitudes.

### 7.5 Obstacle-proximity penalty

Let \(d_{\min}\) denote the minimum estimated ray distance and \(d_{\mathrm{safe}}\) the configured safe-distance reference.

The proximity penalty is

$$
r_{\mathrm{prox}}
=
-2.5
\max(0,d_{\mathrm{safe}}-d_{\min}),
$$

where

$$
d_{\mathrm{safe}}=0.8\ \mathrm{m}.
$$

The penalty is zero when the minimum estimated range is at least \(0.8\) m and becomes increasingly negative as the range falls below this value.

### 7.6 Termination penalties and time cost

The configured terminal penalties are:

| Event | Reward contribution |
|---|---:|
| Obstacle collision | \(-60\) |
| Ground contact | \(-50\) |
| Out of bounds | \(-50\) |
| Excessive roll or pitch | \(-40\) |
| Per-step time cost | \(-0.05\) |

The goal reward, progress term, and safety penalties are designed to encourage goal-reaching behavior while discouraging unsafe flight.

These reward coefficients are hand-calibrated. Their effectiveness must be evaluated experimentally, and the reward function should not be interpreted as a formal safety guarantee.

---

## 8. ACMFDS Context Classification

ACMFDS provides a context-aware interpretation of the flight state. The decision layer assigns a context using configured conditions and gives priority to more critical conditions.

The documented context classification is

$$
c_k=
\begin{cases}
\mathrm{EMERGENCY},
&
\mathrm{SoC}\leq5\%
\ \lor
\mathrm{tilt}>0.55
\ \lor
z<0.25,\\[4pt]
\mathrm{OBSTACLE\_CRITICAL},
&
d_{\min}<0.8\ \mathrm{m},\\[4pt]
\mathrm{OBSTACLE\_NEAR},
&
d_{\min}<1.5\ \mathrm{m},\\[4pt]
\mathrm{LOW\_BATTERY},
&
\mathrm{SoC}\leq30\%,\\[4pt]
\mathrm{STRONG\_WIND},
&
\|\mathbf{v}_{\mathrm{wind}}\|\geq4.0\ \mathrm{m/s},\\[4pt]
\mathrm{LANDING/MISSION\_COMPLETE},
&
d_g\leq0.4\ \mathrm{m},\\[4pt]
\mathrm{NORMAL},
&
\text{otherwise}.
\end{cases}
$$

Here, \(\mathrm{SoC}\) denotes the state of charge and \(\mathbf{v}_{\mathrm{wind}}\) denotes the wind-velocity vector.

The conditions are evaluated according to their precedence in the decision implementation. They are software thresholds for this prototype, not validated operational limits for a physical aircraft.

### 8.1 Context-dependent objective priorities

The ACMFDS decision layer reweights four objectives:

- Safety
- Energy
- Clearance
- Mission progress

The configured weights sum to one:

$$
\sum_i w_i=1,
\qquad
w_i\geq0.
$$

For normal conditions, the documented weights are

$$
\mathbf{w}_{\mathrm{normal}}
=
(0.25,0.20,0.20,0.35).
$$

For emergency conditions, the weights are

$$
\mathbf{w}_{\mathrm{emergency}}
=
(0.75,0.10,0.10,0.05).
$$

The emergency configuration gives safety the highest priority and reduces the relative importance of mission progress.

The Gymnasium environment exposes the ACMFDS context, decision, and reason through its `info` dictionary. These fields provide decision explainability; they do not directly determine the RL environment's action.

---

## 9. Initial Conditions and Episode Termination

### 9.1 Nominal initial state

The documented nominal start and goal positions are

$$
\mathbf{p}_0=(-4.0,0.0,2.0)\ \mathrm{m},
$$

$$
\mathbf{p}_g=(4.5,0.0,2.0)\ \mathrm{m}.
$$

The environment can optionally randomize the initial position and obstacle layout. The attitude is reset to zero, and the simulation advances several settling physics ticks before producing the initial observation.

### 9.2 Permitted region

The configured flight region is

$$
\mathcal{B}
=
[-7,7]\times[-4,4]\times(0.12,5.0]\ \mathrm{m}.
$$

### 9.3 Termination conditions

An episode terminates when the goal is reached or a specified physical failure condition occurs.

$$
\begin{aligned}
\mathrm{terminated}_k
=
\mathbb{1}\big[
&d_k<0.40\\
&\lor\ \mathrm{obstacle\ collision}\\
&\lor\ \mathrm{ground\ contact}\\
&\lor\ \mathbf{p}_k\notin\mathcal{B}\\
&\lor\ \max(|\phi|,|\theta|)>1.3
\big].
\end{aligned}
$$

The episode is truncated when the time horizon is reached without an earlier termination:

$$
\mathrm{truncated}_k
=
\mathbb{1}
\left[
k\geq k_{\max}
\land
\neg\mathrm{terminated}_k
\right].
$$

This distinction follows the Gymnasium API: goal arrival and physical failure are terminations, whereas the episode time limit is a truncation.

Collision detection uses PyBullet contact reports filtered to the obstacle bodies managed by the environment, rather than relying only on a distance threshold.

---

## 10. Installation and Execution

### 10.1 Prerequisites

The core environment requires a compatible Python installation and the dependencies specified by the project. Reinforcement-learning dependencies are optional.

Clone the repository:

```bash
git clone <YOUR_GITHUB_REPOSITORY_URL>
cd <YOUR_PROJECT_DIRECTORY>
```

Create and activate a virtual environment:

```bash
python -m venv .venv
```

On Windows:

```powershell
.venv\Scripts\activate
```

On Linux or macOS:

```bash
source .venv/bin/activate
```

Install the core dependencies using the repository's dependency file, if provided:

```bash
pip install -r requirements.txt
```

The optional RL dependencies can be installed with:

```bash
pip install -r requirements-rl.txt
```

The RL dependency file includes the additional packages required for reinforcement-learning experiments.

### 10.2 Run the default mission pipeline

The documented default mission entry point is:

```bash
python main.py
```

This executes the mission runner, which is separate from the Gymnasium RL environment.

### 10.3 Validate the Gymnasium environment

Run the environment test suite:

```bash
python test_env.py
```

The test suite includes Gymnasium environment-checker compliance.

### 10.4 Train an optional reinforcement-learning policy

The training script supports PPO and SAC:

```bash
python train.py --algo ppo --timesteps 50000
```

Alternatively:

```bash
python train.py --algo sac --timesteps 50000
```

Training requires the optional RL dependencies. A successful environment validation does not, by itself, establish that a trained policy achieves reliable flight or safe navigation.

---

## 11. Gymnasium API

The environment follows the Gymnasium interface.

### Reset

```python
observation, info = env.reset(seed=42)
```

The reset operation initializes the simulation and returns the initial observation and auxiliary information.

### Step

```python
observation, reward, terminated, truncated, info = env.step(action)
```

The returned values are:

- `observation`: the next 27-dimensional observation.
- `reward`: the scalar reward for the transition.
- `terminated`: whether a goal or physical terminal condition occurred.
- `truncated`: whether the episode ended because of its time limit.
- `info`: auxiliary information, including ACMFDS context and decision information.

### Rendering

The environment supports human-readable visualization and RGB-array rendering. In RGB-array mode, the documented rendered image has dimensions \(320\times320\times3\) and uses an unsigned 8-bit integer representation.

The environment's `close()` method disconnects the PyBullet client and supports repeated cleanup calls.

---

## 12. Limitations

The current implementation has several limitations that must be considered when interpreting results.

1. **Hand-calibrated rewards:** Reward coefficients are manually specified and have not been established as globally optimal.
2. **Limited sensing:** The RL environment uses eight fixed range-sensor rays with a maximum range of 5 m. It does not implement a complete mapping or memory system.
3. **Observation scaling:** Observation components use different physical units and are clipped to bounds but are not uniformly normalized.
4. **Context-input limitations:** The documented Gymnasium path constructs an incomplete flight state without battery and wind fields. Consequently, low-battery and strong-wind ACMFDS contexts cannot be activated through this path as currently implemented.
5. **Decision-versus-control separation:** ACMFDS output is telemetry in `DroneEnv`; the policy action controls the physics in this path.
6. **Reward validation:** No ablation study is documented to isolate the contribution of individual reward terms.
7. **Goal definition:** Goal completion is based on a three-dimensional distance threshold. Reaching the goal sphere does not independently enforce a separate altitude requirement.
8. **Simulation-to-reality gap:** The configured dynamics, sensing, contact behavior, and physical parameters are simulation assumptions. Real-world performance has not been established.
9. **Optional reinforcement learning:** The RL environment and training workflow are optional and are not part of the validated results of the default mission pipeline.

These limitations identify opportunities for future development, including observation normalization, broader randomized testing, improved directional sensing, systematic reward analysis, integration of context-dependent decisions with control, and evaluation under more realistic disturbances.

---

## 13. Future Work

Potential future work includes:

- Integrating battery and wind state into the Gymnasium environment's ACMFDS inputs.
- Improving directional obstacle information and introducing richer perception or mapping.
- Evaluating learned policies using repeated randomized scenarios.
- Comparing PPO, SAC, and baseline controllers under consistent conditions.
- Conducting reward ablation studies and sensitivity analysis.
- Measuring goal-reaching rate, collision rate, minimum clearance, cumulative reward, and episode duration.
- Improving the integration between ACMFDS decisions and low-level control.
- Assessing robustness to uncertain dynamics and environmental disturbances.
- Investigating sim-to-real transfer only after adequate simulation and hardware validation.

---

## 14. Conclusion

This project provides a modular framework for studying autonomous quadcopter behavior through rigid-body simulation, a Gymnasium-compatible MDP, virtual obstacle sensing, reward-based evaluation, and context-aware decision interpretation.

PyBullet determines the physical evolution of the simulated vehicle, Gymnasium defines the interaction interface for optional reinforcement-learning experiments, and ACMFDS exposes contextual flight decisions and objective priorities.

The separation between simulation, policy actions, and decision explainability supports structured experimentation and debugging. Nevertheless, the framework remains a research prototype, and its safety and performance characteristics must be established through systematic evaluation rather than inferred from implementation alone.

---

## References

1. Gymnasium Documentation. *Gymnasium: A Standard API for Reinforcement Learning*. https://gymnasium.farama.org/
2. PyBullet Documentation and Examples. *Bullet Physics Simulation*. https://pybullet.org/
3. Stable-Baselines3 Documentation. *Reliable Reinforcement Learning Implementations*. https://stable-baselines3.readthedocs.io/
4. Project source code: `drone_env.py`, `drone.py`, `train.py`, `test_env.py`, and the ACMFDS decision and mission modules.

---

**Project status:** Simulation-based research prototype. Results and claims should be restricted to the implemented and tested functionality of the submitted code.
