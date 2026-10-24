# ACMFDS — Adaptive Context-Aware Multi-Objective Flight Decision System

*Autonomous Quadrotor Simulation | Python · MuJoCo · PySide6 · A · Random Forest*\*

ACMFDS is an academic/research simulation demonstrating how an autonomous quadrotor can adapt route selection and movement to simulated obstacles, wind, battery level, contextual risk, and mission progress. It combines grid-based path planning, collision checks, candidate-route scoring, a synthetic-data Random Forest risk estimator, adaptive objective weights, simplified motion/wind/battery models, dynamic-obstacle replanning, and an interactive desktop dashboard.

> **Safety note:** This is a simulation and research project. Its heuristic risk score, simplified dynamics, battery model, and collision checks are not certified for real-world drone flight and do not guarantee physical safety.

## Visual Overview & Architecture

*Figure 1: High-level visualization of the autonomous quadrotor navigating the simulation arena featuring dynamic and static obstacles alongside telemetry feedback loops.*

## Contents

 1. [Overview](#1-overview)

 2. [Problem statement and objectives](#2-problem-statement-and-objectives)

 3. [Features](#3-features)

 4. [Architecture and files](#4-architecture-and-files)

 5. [Technology stack](#5-technology-stack)

 6. [Simulation environment and coordinates](#6-simulation-environment-and-coordinates)

 7. [Obstacle model](#7-obstacle-model)

 8. [A\* path planning](#8-a-path-planning)

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

## At a glance

```
flowchart TD
    A["Vehicle + environment state"] --> B["Context features"]
    B --> C["Random Forest risk estimate"]
    B --> D["Adaptive objective weights"]
    E["A* route candidates"] --> F["Collision and clearance checks"]
    F --> G["Multi-objective route scoring"]
    C --> G
    D --> G
    G --> H["Select valid route"]
    H --> I["Controller + wind + battery model"]
    I --> J["MuJoCo simulation step"]
    J --> K["Telemetry and dashboard"]
    K --> B

```

**How to read the diagram:** The current simulated state informs the risk estimate and objective weights; route candidates are checked and scored; the selected route is followed by the controller; updated telemetry feeds the next decision cycle. The diagram is a conceptual overview, not a claim that every operation runs in a separate process.

### Main decision loop

```
flowchart LR
    S["Observe state"] --> R["Estimate risk"]
    R --> W["Adapt weights"]
    W --> P["Plan / replan"]
    P --> C["Check and score routes"]
    C --> F["Follow route"]
    F --> U["Update physics and telemetry"]
    U --> S

```

## 1. Overview

A shortest-path planner may find a route that is not necessarily best when obstacle clearance, wind, battery state, turning difficulty, and mission time are considered together. ACMFDS evaluates route candidates against several objectives and adjusts the relative importance of those objectives according