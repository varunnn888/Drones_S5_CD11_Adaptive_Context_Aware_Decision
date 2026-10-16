# ACMFDS — Autonomous Quadrotor Simulation

**Adaptive Context-Aware Multi-Objective Flight Decision System (ACMFDS)** is a desktop simulation project for exploring quadrotor navigation, candidate path selection, obstacle-aware replanning, and live flight telemetry. It uses **MuJoCo** for the simulation scene and **PySide6** for the dashboard.

> This repository is for simulation, learning, and demonstration. It is **not** a flight-ready autopilot or a safety-certified control system.

## Highlights

- **Expanded 3D arena:** approximately 16.5 m × 14.5 m, with a target at `(12, 10, 2)` metres.
- **Orbitable 3D camera:** drag with the left mouse button to orbit manually, scroll to zoom, or use the automatic 360° orbit control.
- **Camera presets:** `OVERVIEW`, `TOP`, and `SIDE`.
- **Static obstacles:** fixed red obstacles that can be enabled or disabled from the dashboard.
- **Dynamic obstacles:** moving orange hazards with repeatable patrol paths. Toggle them on or off and replan around their current simulated positions.
- **Candidate path profiles:** inspect available paths, choose a profile, and apply it using `USE ROUTE`.
- **Replanning:** use `REPLAN` to refresh candidate paths for the currently enabled obstacles.
- **Restartable mission:** after the drone reaches the destination, choose `START AGAIN` to reset the mission and run it again.
- **Telemetry and plots:** position, velocity, speed, altitude, distance to target, battery, risk indicators, route metrics, adaptive objective weights, and time-series graphs.
- **Simulated GPS:** local X/Y coordinates are converted to approximate latitude/longitude using an editable origin in `config.py`.

## Technology stack

- Python 3.12 recommended
- MuJoCo
- PySide6 / Qt
- NumPy
- scikit-learn

Dependencies are listed in [`requirements.txt`](requirements.txt).

## Run on macOS

1. Download and extract the project ZIP.
2. Open the extracted `ACMFDS_Orbit_Expanded` folder in Terminal or VS Code.
3. Run the launcher:

```bash
bash run_mac.sh
```

The launcher prefers Python 3.12 when available, creates a local `.venv` if needed, installs the dependencies from `requirements.txt`, and starts the dashboard. The initial dependency installation requires an internet connection and may take a few minutes.

To run it manually instead:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python app.py
```

On later runs, from the project folder use:

```bash
source .venv/bin/activate
python app.py
```

## Run on Windows

Install Python 3.12 first, then open Command Prompt or PowerShell in the project folder and run:

```powershell
py -3.12 -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python app.py
```

The `run_windows.bat` launcher can be used after the environment and requirements have been installed.

## Dashboard controls

| Control | Purpose |
| --- | --- |
| **AUTO 360° ORBIT** | Start continuous camera orbit; use **STOP 360° ORBIT** to pause it. |
| **Drag with left mouse** | Manually rotate/orbit the 3D camera. |
| **Scroll wheel** | Zoom in or out. |
| **OVERVIEW / TOP / SIDE** | Switch to a camera preset. |
| **PATH PROFILE** | Select one of the currently available candidate route profiles. |
| **USE ROUTE** | Apply the selected route profile from the drone's current position. |
| **Static obstacles** | Enable or disable static hazards for collision checks and planning. |
| **Dynamic obstacles** | Enable or disable moving hazards and their planning/collision effects. |
| **REPLAN** | Recalculate and rank candidate paths using the current obstacle settings. |
| **RESET MISSION / START AGAIN** | Reset position, mission time, battery/history, and begin from the start marker. |

## Project layout

```text
ACMFDS_Orbit_Expanded/
├── app.py             # Application entry point
├── config.py          # Mission start/goal, simulation rate, GPS origin and conversion
├── dashboard.py       # PySide6 dashboard, camera interaction, controls, plots and XY map
├── data_model.py      # Data-driven risk estimation and adaptive objective weights
├── planner.py         # Candidate path generation, collision checks, scoring and ranking
├── simulation.py      # MuJoCo scene, quadrotor simulation, obstacles, telemetry and reset
├── requirements.txt   # Python dependencies
├── run_mac.sh         # macOS environment setup and launch script
├── run_windows.bat    # Windows launch script (after dependency setup)
└── README.md          # Project documentation
```

## Configuration notes

- **Mission coordinates:** change `START` and `DESTINATION` in `config.py`.
- **Simulation rate:** `SIM_HZ` is configured in `config.py`.
- **GPS origin:** edit `GPS_ORIGIN_LAT`, `GPS_ORIGIN_LON`, and `GPS_ORIGIN_ALT_M` in `config.py` to change the origin used for simulated GPS telemetry.
- **GPS disclaimer:** latitude and longitude are derived from the simulation's local position. They are **not** live data from a physical GPS receiver, and the local conversion is an approximation intended for this small simulated arena.

## Troubleshooting

### `python: command not found`

Activate the project's virtual environment first, or use `bash run_mac.sh` on macOS. If Python 3.12 is not installed, install it and retry.

### Dependencies are missing

Activate `.venv` and run:

```bash
python -m pip install -r requirements.txt
```

### MuJoCo or OpenGL rendering errors

Confirm that the installed MuJoCo and PySide6 dependencies finished installing. On macOS, update the graphics drivers/OS if applicable, close other GUI applications that may be using the graphics context, and relaunch from the project's own virtual environment. Include the complete terminal traceback when reporting a rendering problem.

### Route choices change when obstacles move

This is expected: dynamic hazards change position during the simulation, so the planner can produce a different list of available candidate paths after replanning.

## Publishing this README to GitHub

To update a repository through the GitHub website:

1. Open your repository on GitHub.
2. Choose **Add file → Upload files**.
3. Upload this `README.md` to the repository's **root directory**. If GitHub asks whether to replace an existing file, confirm only if you want this version to become the repository's README.
4. Enter a commit message such as `Update ACMFDS project README` and choose **Commit changes**.

If you use Git locally, place this file at the repository root and run:

```bash
git add README.md
git commit -m "Update ACMFDS project README"
git push
```

## Disclaimer

All navigation, obstacle movement, risk estimates, battery behavior, and GPS readings in this project are simulated. Outputs are intended for education and visualization only; do not connect this demo directly to a real drone without a separate, rigorously validated flight-control and safety system.
