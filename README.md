<img width="1948" height="568" alt="logo-branding-amrita-universiy-2024" src="https://github.com/user-attachments/assets/db019883-cfb1-46ad-9811-7f7ec523249e" />

## Course Details

- **Course:** 22AIE448 – Introduction to Data Driven Control of Drones
- **Batch:** AIE-CD
- **Team:** CD11
<h1>Adaptive Context-Aware Multi-Objective Flight Decision Algorithm (ACMFDS)</h1>

## Team Members

| S. No. | Name | Roll Number | Email |
|--------|------|-------------|-------|
| 1 | Deepika Reddy | cb.sc.u4aie24215 | [cb.sc.u4aie24215@cb.students.amrita.edu](mailto:cb.sc.u4aie24215@cb.students.amrita.edu) |
| 2 | Gade Varshini | cb.sc.u4aie24216 | [cb.sc.u4aie24216@cb.students.amrita.edu](mailto:cb.sc.u4aie24216@cb.students.amrita.edu) |
| 3 | Jovika N B | cb.sc.u4aie24223 | [cb.sc.u4aie24223@cb.students.amrita.edu](mailto:cb.sc.u4aie24223@cb.students.amrita.edu) |
| 4 | P Poojitha | cb.sc.u4aie24237 | [cb.sc.u4aie24237@cb.students.amrita.edu](mailto:cb.sc.u4aie24237@cb.students.amrita.edu) |
| 5 | Potnuru Varun | cb.sc.u4aie24244 | [cb.sc.u4aie24244@cb.students.amrita.edu](mailto:cb.sc.u4aie24244@cb.students.amrita.edu) |
# 1. About the Project

**ACMFDS (Adaptive Context-Aware Multi-Objective Flight Decision Algorithm)** is an academic project exploring how an autonomous drone can make navigation and mission decisions while operating in changing simulated environments.

Traditional route planning often focuses on finding a short path between a starting point and a destination. However, a drone operating in a dynamic environment may also need to consider obstacle clearance, wind conditions, energy consumption, battery reserves, tracking difficulty, and mission deadlines.

ACMFDS investigates these competing considerations through multiple simulation implementations. The repository brings together MATLAB-based simulations, MATLAB and Simulink models, MuJoCo-based experiments, and a Gym/PyBullet simulation project.

Each implementation provides its own representation of the environment, dynamics, control logic, and navigation workflow. The implementations should therefore be examined individually rather than assumed to use identical algorithms or physical models.

### Project Objectives

- Study autonomous drone navigation in simulated environments.
- Explore path-planning algorithms and obstacle avoidance.
- Investigate multi-objective route selection.
- Examine the effects of wind, mission duration, and energy constraints where implemented.
- Study decision-making, recovery strategies, and mission-state transitions.
- Compare simulation approaches and their respective capabilities.
- Establish a mathematical foundation for interpreting navigation, motion, energy, and feasibility.

> **Important:** ACMFDS is a simulation and research project. Its results are not proof of real-world flight safety, and the software is not certified autopilot software.

---

## 2. Repository Overview

The repository contains multiple simulation implementations and supporting project documents.

| Resource | Purpose |
|---|---|
| [MATLAB Simulation](./Matlab%20Simulation/) | Explore the MATLAB-based navigation and mission-decision implementation. |
| [MATLAB + Simulink](./Matlab%2B%20Simulink/) | Examine the MATLAB/Simulink models and their associated control or state-based simulation workflows. |
| [MuJoCo](./Mujoco/) | Explore the simulation implementation based on the MuJoCo physics engine. |
| [Gym-PyBullet Simulation](./gym-pybullet/) | Examine the Python-based drone simulation project and its environment implementation. |
| [Stateflow Model](./ACMFDS_Stateflow.slx) | Inspect the Stateflow model used in the project. |
| [Project Presentation](./ACMFDS_Project_Presentation.pptx) | Review the project presentation and overall design. |
| [Project Brief](./FRP.md) | Refer to the project requirements and problem description. |

**Where should you start?**

Start with the simulation directory that interests you. Read its local documentation, inspect its source files, identify its entry point, and verify its configuration before running it.

The individual simulation folders are the authoritative sources for their own dependencies, implementation details, and execution instructions.

---
## 3. Conceptual System Architecture

The following diagram describes the general decision-making workflow explored by ACMFDS. It is a conceptual overview, not a claim that every implementation follows precisely the same pipeline.

```mermaid
flowchart TD
    A["Mission Configuration"] --> B["Environment and Vehicle State"]
    B --> C["Context Assessment"]
    C --> D["Route Planning"]
    D --> E["Obstacle and Feasibility Checks"]
    E --> F["Route Evaluation and Decision"]
    F --> G["Selected Route or Waypoint"]
    G --> H["Motion and Simulation Update"]
    H --> I["Updated Position, Time and Energy"]
    I --> C
    E -->| "Infeasible route" | J["Replanning or Recovery Evaluation"]
    J --> D
