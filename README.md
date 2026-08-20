
<img width="1948" height="568" alt="logo-branding-amrita-universiy-2024" src="https://github.com/user-attachments/assets/db019883-cfb1-46ad-9811-7f7ec523249e" />

<h1> Adaptive Context-Aware Multi-Objective Flight Decision Algorithm</h1>

## Team Members

| S. No. | Name | Roll Number | Email |
|--------|------|-------------|-------|
| 1 | Deepika Reddy | cb.sc.u4aie24215 | [cb.sc.u4aie24215@cb.students.amrita.edu](mailto:cb.sc.u4aie24215@cb.students.amrita.edu) |
| 2 | Gade Varshini | cb.sc.u4aie24216 | [cb.sc.u4aie24216@cb.students.amrita.edu](mailto:cb.sc.u4aie24216@cb.students.amrita.edu) |
| 3 | Jovika N B | cb.sc.u4aie24223 | [cb.sc.u4aie24223@cb.students.amrita.edu](mailto:cb.sc.u4aie24223@cb.students.amrita.edu) |
| 4 | Poojitha P | cb.sc.u4aie24237 | [cb.sc.u4aie24237@cb.students.amrita.edu](mailto:cb.sc.u4aie24237@cb.students.amrita.edu) |
| 5 | Potnuru Varun | cb.sc.u4aie24244 | [cb.sc.u4aie24244@cb.students.amrita.edu](mailto:cb.sc.u4aie24244@cb.students.amrita.edu) |

## Abstract

This project proposes an Adaptive Contextual Multi-Factor Decision System (ACMFDS) for autonomous UAV navigation in dynamic environments. The system continuously evaluates battery level, wind conditions, obstacle distance, mission priority, safety, energy, time, and tracking requirements to determine appropriate flight behavior. Based on the changing context, it dynamically adjusts decision weights, flight speed, operating mode, and navigation strategy. When obstacles block the direct path, an adaptive path planner evaluates alternative routes based on distance, clearance, and cost, while unsafe routes are rejected. The system also responds to environmental changes such as nearby obstacles and high wind by switching to appropriate safety-oriented modes and replanning the mission. A simulation demonstrates successful context detection, adaptive decision-making, obstacle avoidance, dynamic path planning, waypoint navigation, and mission completion while maintaining energy efficiency and safety. Thus, ACMFDS provides a unified framework for intelligent, context-aware, and adaptive UAV mission planning and autonomous navigation.



# Introduction

ACMFDS (Adaptive Contextual Multi-Factor Decision System) is a MATLAB/Simulink-based framework for autonomous UAV navigation in dynamic environments. It considers battery level, wind speed, obstacle distance, and mission priority to identify the current context and adapt the UAV's speed, flight mode, and navigation action. The system uses five factors—Energy,Time, Safety, Tracking, and Mission—with context-dependent weights to make balanced decisions.

The overall decision cost is represented as:

$$C = \sum_i w_i f_i$$

| Symbol | Description |
|--------|-------------|
| **C** | Overall decision / path cost |
| **w_i** | Importance (weight) of each factor |
| **f_i** | Value / cost of that factor |

When the direct path is blocked, the planner evaluates alternative routes using distance, clearance, and cost, rejects unsafe routes, and selects a feasible detour.

The project integrates **MATLAB**, **Simulink**, and **Stateflow** to demonstrate context detection, adaptive decision-making, obstacle avoidance, dynamic path planning, waypoint navigation, changing environmental conditions, and successful mission completion.



## Methodology

### 1.Overall Methodology

The proposed Adaptive Contextual Multi-Factor Decision System (ACMFDS) follows a closed-loop decision-making process. The UAV receives environmental and mission information such as battery level, wind speed, obstacle distance, and mission priority. These inputs are used to determine the current operating context.

Based on the detected context, ACMFDS selects appropriate decision weights, flight speed, operating mode, and action. If the direct path is safe, the UAV continues toward the waypoint. If an obstacle blocks the path, the adaptive planner evaluates possible detours based on distance, safety clearance, and cost. The selected path is then used by the UAV simulation, while changing environmental conditions can trigger a new decision.

### 2.Methodology Flow Diagram

```mermaid
flowchart TD
    A([Mission Inputs\nBattery · Wind · Obstacle · Priority])
    B([Context Detection\nNormal / High Wind / Obstacle / Low Battery])
    C([Adaptive Weights\nEnergy · Safety · Time · Tracking · Mission])
    D([Decision Making\nSpeed · Mode · Action])
    E{Direct Path Safe?}
    F([Continue Flight])
    G([Adaptive Path Planning\nEvaluate Routes])
    H([Select Safe Detour])
    I([UAV Simulation\nWaypoint Navigation])
    J([Environment Update\nBattery · Wind · Obstacles · Context])

    A --> B --> C --> D --> E
    E -- YES --> F
    E -- NO --> G --> H
    F --> I
    H --> I
    I --> J
    J -->|Re-evaluate| B
```

### 3.Input and Environment Modeling

The system uses four main environmental/mission inputs:

| Input | Purpose |
|-------|---------|
| Battery Level | Represents the available UAV energy |
| Wind Speed | Represents environmental disturbance |
| Obstacle Distance | Determines the level of navigation risk |
| Mission Priority | Represents the importance of completing the mission |

These values can be changed during simulation to demonstrate how the UAV responds to different operating conditions.

### 4.Context Detection

The input conditions are interpreted by the decision system to determine the UAV's current context. Typical contexts used in the project include:

- **Normal** – Suitable environmental conditions.
- **Obstacle Nearby** – An obstacle requires increased safety consideration.
- **High Wind** – Strong wind requires safer flight behavior.
- **Low Battery** – Energy conservation becomes important.

The detected context determines which decision strategy and weights should be applied.

### 5.Adaptive Multi-Factor Decision Making

#### 5.1 Decision Factors

ACMFDS considers five major decision factors:

| Factor | Description |
|--------|-------------|
| **Energy** | Reduces unnecessary energy consumption |
| **Safety** | Prioritizes safe operation and obstacle clearance |
| **Time** | Considers the time required to complete the route |
| **Tracking** | Encourages the UAV to remain close to the desired route |
| **Mission** | Considers the importance of mission completion |

#### 5.2 Decision Cost Formula

The combined decision cost is represented as:

$$C = \sum_{i=1}^{n} w_i f_i$$

For this project, the expanded form is:

$$C = w_E E + w_S S + w_T T + w_R R + w_M M$$

| Symbol | Description |
|--------|-------------|
| **C** | Overall decision cost |
| **E** | Energy cost |
| **S** | Safety cost |
| **T** | Time cost |
| **R** | Tracking cost |
| **M** | Mission cost |
| **w_E, w_S, w_T, w_R, w_M** | Corresponding adaptive weights |

#### 5.3 Context-Dependent Weights

Not every factor is equally important in every situation. The weights are adapted based on the current context rather than remaining fixed:

| Context | Priority |
|---------|----------|
| **Normal** | Efficiency and mission progress receive more consideration |
| **Obstacle Nearby** | Safety receives greater importance |
| **High Wind** | Safe operation becomes more important than minimizing travel time |

### 6 Adaptive Path Planning

After the decision stage, the system checks whether the direct path to the next waypoint is safe.

#### 6.1 Case 1 — Direct Path is Clear

The UAV follows the direct path toward the waypoint without any replanning.

#### 6.2 Case 2 — Direct Path is Blocked

The planner generates alternative routes in the following directions:

| Direction | Description |
|-----------|-------------|
| **East** | Alternative route to the east |
| **West** | Alternative route to the west |
| **North** | Alternative route to the north |
| **South** | Alternative route to the south |

Each candidate route is evaluated using its distance, obstacle clearance, and associated cost.

#### 6.3 Path Cost Formula

A simplified path-cost representation is:

$$C_{path} = w_d D + w_s S + w_t T$$

| Symbol | Description |
|--------|-------------|
| **D** | Path distance |
| **S** | Safety / clearance cost |
| **T** | Travel-time cost |
| **w_d, w_s, w_t** | Corresponding importance weights |

#### 6.4 Route Selection

- Routes that do not satisfy the required safety clearance are considered **infeasible** and are **rejected**.
- The UAV selects the **lowest-cost feasible route** among the remaining candidates.

### 7.Obstacle Clearance and Safety

The system does not consider the UAV as a point. A vehicle radius and safety buffer are included when determining whether a route is safe.

The required clearance is represented as:

$$d_{required} = r_{UAV} + b_{safety}$$

| Symbol | Description |
|--------|-------------|
| **d_required** | Minimum required clearance distance |
| **r_UAV** | UAV / vehicle radius |
| **b_safety** | Additional safety buffer |

A candidate path is rejected when it enters the unsafe region around an obstacle. This allows the simulation to demonstrate realistic obstacle avoidance rather than simply checking whether the UAV's center crosses the obstacle.

### 8.Dynamic Context Update

One of the main features of ACMFDS is that the environment can change during the mission, and the system continuously re-evaluates its decisions accordingly.

#### Context Transition Example

```mermaid
flowchart LR
    A([Normal]) --> B([Obstacle Nearby]) --> C([High Wind]) --> D([Normal])
```

#### What Gets Updated on Context Change?

| Parameter | Description |
|-----------|-------------|
| **Flight Mode** | Switches between Normal, Safety, and Emergency modes |
| **UAV Speed** | Adjusted based on current risk level |
| **Decision Weights** | Re-prioritized according to the new context |
| **Navigation Action** | Flight behavior is updated accordingly |
| **Selected Path** | A new safe route is planned if required |

> **Key Principle:** The UAV does not need to complete the entire mission using the original decision. It continuously re-evaluates its behavior as conditions change, ensuring safe and efficient mission completion at all times.

### 9.Stateflow Implementation

The Stateflow component represents the UAV's behavioral logic. The input conditions are supplied to the Stateflow model, where transitions determine the appropriate operating state.

#### State Transition Flow

```mermaid
flowchart TD
    A([Inputs\nBattery · Wind · Obstacle · Priority])
    B([Context Conditions])
    C{Stateflow}
    D([Normal Flight])
    E([High Wind Mode])
    F([Obstacle Nearby Mode])
    G([Speed / Mode / Action])

    A --> B --> C
    C --> D
    C --> E
    C --> F
    D --> G
    E --> G
    F --> G
```

#### Operating States

| State | Trigger Condition | UAV Behavior |
|-------|------------------|--------------|
| **Normal Flight** | Stable environment, clear path | Standard speed, efficiency-focused |
| **High Wind Mode** | Wind speed exceeds threshold | Reduced speed, safety-prioritized |
| **Obstacle Nearby** | Obstacle within safety distance | Replanning triggered, safety weight increased |

> **Key Principle:** Each state transition is driven by the input conditions, ensuring the UAV always operates in the most appropriate mode for the current environment.

### 10 MATLAB and Simulink Integration

The project uses MATLAB for the main UAV simulation and adaptive path-planning logic, while Simulink/Stateflow is used to represent and test the decision-making behavior.

| Component | Role |
|-----------|------|
| **MATLAB** | Main UAV simulation and adaptive path-planning logic |
| **Simulink** | System-level modeling and integration |
| **Stateflow** | Behavioral decision-making representation and testing |

The integration allows the project to demonstrate both:

- **Behavioral decision making** through Stateflow.
- **Full UAV mission simulation** through MATLAB.

The simulation records information such as UAV position, battery level, wind conditions, obstacle conditions, context, selected path, and mission progress.

---

### 11 Mission Execution

The UAV moves through a sequence of mission waypoints. At each stage, the following process is followed:

````mermaid
flowchart TD
    A([1. Observe Environmental Conditions])
    B([2. Determine Current Context])
    C([3. Apply Decision Weights])
    D([4. Select Speed and Flight Mode])
    E{5. Check Direct Path}
    F([6. Generate Detour if Necessary])
    G([7. Move Toward Waypoint])
    H([8. Update Environment])
    I{Mission Complete?}
    J([End Mission])

    A --> B --> C --> D --> E
    E -- Clear --> G
    E -- Blocked --> F --> G
    G --> H --> I
    I -- No --> A
    I -- Yes --> J
````

> **Key Principle:** This creates a continuous adaptive decision loop rather than a one-time decision.

---

### 12 Simulation and Testing

The system is tested under different conditions by varying the following parameters:

| Test Parameter | Description |
|----------------|-------------|
| **Battery Level** | Tests energy-aware decision making |
| **Wind Speed** | Tests safety mode activation |
| **Obstacle Distance / Count** | Tests path replanning and avoidance |
| **Mission Conditions** | Tests mission priority handling |

The resulting changes in UAV speed, operating mode, selected path, travel distance, and mission behavior are observed and compared. The obstacle experiments also demonstrate that increasing environmental obstacles causes the UAV to select alternative paths instead of simply continuing along the direct route.

---

### 13 Overall Methodology Summary

The complete ACMFDS methodology can be summarized as a continuous adaptive loop:

````mermaid
flowchart LR
    A([Sense]) --> B([Identify Context]) --> C([Assign Weights]) --> D([Make Decision]) --> E([Check Path]) --> F([Avoid Obstacles]) --> G([Fly]) --> H([Update Environment]) --> A
````

This enables the UAV to adapt its behavior according to changing environmental and mission conditions while balancing:

| Factor | Role |
|--------|------|
| **Safety** | Avoid obstacles and unsafe conditions |
| **Energy** | Conserve battery for mission completion |
| **Time** | Minimize unnecessary delays |
| **Tracking** | Stay close to the planned route |
| **Mission** | Ensure successful mission completion |

# Results

## Simulation Results

The Adaptive Context-Aware Multi-Objective Flight Decision Algorithm (ACMFDS) was evaluated under multiple dynamic operating conditions. During execution, the UAV continuously monitored battery level, wind speed, obstacle distance, and mission priority to determine the current operating context.

The simulation demonstrates that the proposed framework successfully performs:

- Context-aware decision making
- Dynamic weight adaptation
- Adaptive obstacle avoidance
- Multi-objective route selection
- Safe waypoint navigation
- Dynamic context switching
- Mission completion under changing environmental conditions

---

## Test Case 1 – Dynamic Context Switching

The first experiment demonstrates how the UAV adapts to changing environmental conditions during mission execution.

| Parameter | Observation |
|-----------|-------------|
| Initial Context | Obstacle Nearby |
| Context Changes | Obstacle Nearby → High Wind → Normal |
| Obstacle Avoidance | Successful |
| Waypoints Reached | 5 |
| Mission Status | Completed |

### Observations

- Safety weight increased when an obstacle appeared.
- Flight speed reduced automatically.
- The planner generated a safe detour.
- During high wind conditions the UAV entered Safety Mode.
- After the environment became normal, the UAV resumed normal flight.
- Mission completed successfully.

---

## Test Case 2 – Adaptive Path Planning

The planner evaluates multiple candidate routes whenever the direct path is blocked.

Example planner output:

| Route | Clearance | Status |
|--------|-----------|--------|
| East | Safe | Selected |
| West | Unsafe | Rejected |
| North | Safe | Candidate |
| South | Unsafe | Rejected |

The planner selected the route having minimum cost while satisfying safety constraints.

---

## Test Case 3 – Context Transition

During execution, the UAV transitioned through multiple operating contexts.

```mermaid
flowchart LR
A[Obstacle Nearby] --> B[High Wind]
B --> C[Normal]
```

The transition demonstrates that ACMFDS continuously monitors environmental conditions instead of relying on a single precomputed decision.

---

## Test Case 4 – Mission Completion

The final simulation completed successfully.

| Metric | Result |
|---------|--------|
| Mission Status | Success |
| Final Context | Normal |
| Final Position | Mission Completed |
| Battery Remaining | Approximately 84.6% |
| Waypoints Reached | All Waypoints |

---

## Performance Comparison

| Feature | Conventional Planning | Proposed ACMFDS |
|-----------|----------------------|-----------------|
| Context Awareness | No | Yes |
| Dynamic Decision Making | No | Yes |
| Adaptive Weights | No | Yes |
| Obstacle Replanning | Limited | Yes |
| Stateflow Integration | No | Yes |
| Dynamic Context Switching | No | Yes |
| Mission Adaptation | No | Yes |

---

## Result Summary

The simulation demonstrates that the proposed ACMFDS successfully:

- Detects changing operating conditions.
- Updates decision priorities dynamically.
- Adjusts flight speed automatically.
- Generates safe alternative paths.
- Avoids obstacles.
- Maintains mission progress.
- Successfully completes the mission.

---

# Project Files

The repository contains the following project files.

| File | Description |
|------|-------------|
| ACMFDS_Stateflow.slx | Simulink Stateflow model |
| ACMFDS_MissionData.mat | Mission parameters |
| acmfds_uav_v2.m | Main MATLAB simulation |
| multiObstaclePlanner.m | Adaptive path planner |
| multiObstacleExperiment.m | Multiple obstacle experiments |
| obstacleCountExperiment.m | Obstacle analysis |
| MultiObjectiveResults.mat | Simulation results |
| MultiObjectiveResults.xlsx | Exported results |
| README.md | Project documentation |

---

# Conclusion

This project presented an Adaptive Context-Aware Multi-Objective Flight Decision Algorithm (ACMFDS) for autonomous UAV navigation in dynamic environments. The proposed framework combines context detection, adaptive multi-objective decision making, Stateflow-based behavioral modeling, and MATLAB-based adaptive path planning within a unified architecture.

Unlike conventional approaches that operate with fixed decision priorities, ACMFDS continuously updates its decision weights according to environmental conditions such as obstacle proximity, wind speed, battery level, and mission priority. The adaptive planner evaluates multiple feasible routes and selects the safest low-cost alternative whenever the direct path becomes blocked.

Simulation results demonstrate successful context switching, adaptive obstacle avoidance, safe waypoint navigation, dynamic flight mode selection, and mission completion under changing environmental conditions. The integration of MATLAB, Simulink, and Stateflow provides a practical framework for developing intelligent autonomous UAV systems capable of balancing safety, efficiency, mission objectives, and energy utilization.

Overall, the proposed ACMFDS improves the adaptability, robustness, and autonomy of UAV mission planning while providing an extensible platform for future intelligent aerial navigation research.

---

# Future Scope

Several enhancements can be incorporated into the proposed framework.

- Integration with real UAV hardware.
- Real-time GPS based navigation.
- Vision-based obstacle detection.
- Deep Learning based context prediction.
- Reinforcement Learning for adaptive policy optimization.
- Multi-UAV cooperative mission planning.
- Dynamic weather forecasting integration.
- 3D terrain-aware path planning.
- Real-time onboard implementation using embedded systems.

---

# References

1. Base Research Paper (Provided for the Project)

2. MathWorks — MATLAB

https://www.mathworks.com/products/matlab.html

3. MathWorks — Simulink

https://www.mathworks.com/products/simulink.html

4. MathWorks — Stateflow

https://www.mathworks.com/products/stateflow.html

5. LaValle, S. M.

Planning Algorithms.

https://planning.cs.uiuc.edu/

6. Sebastian Thrun, Wolfram Burgard, Dieter Fox.

Probabilistic Robotics.

https://mitpress.mit.edu/9780262201629/probabilistic-robotics/

7. UAV Path Planning Survey

https://ieeexplore.ieee.org/

8. Multi-Objective Optimization for Autonomous UAV Navigation

https://www.sciencedirect.com/

---

# Repository Structure

```
ACMFDS/
│
├── ACMFDS_Stateflow.slx
├── ACMFDS_MissionData.mat
├── acmfds_uav_v2.m
├── multiObstaclePlanner.m
├── multiObstacleExperiment.m
├── obstacleCountExperiment.m
├── MultiObjectiveResults.mat
├── MultiObjectiveResults.xlsx
├── README.md
├── report.pdf
└── presentation.pptx
```

---

# Software Requirements

| Software | Version |
|-----------|----------|
| MATLAB | R2023a or later |
| Simulink | Included |
| Stateflow | Included |

---

# How to Run

1. Open MATLAB.

2. Place all project files in the same folder.

3. Open

```
ACMFDS_Stateflow.slx
```

4. Run

```matlab
acmfds_uav_v2
```

5. Execute the experiment files if required.

```matlab
multiObstacleExperiment
```

```matlab
obstacleCountExperiment
```

---

# Acknowledgement

The authors sincerely thank **Amrita Vishwa Vidyapeetham**, the Department of Artificial Intelligence, and the faculty members for their continuous guidance and support throughout the development of this project.
