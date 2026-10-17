"""FastAPI + WebSocket live dashboard for the ACMFDS drone simulation.

Architecture — exactly ONE simulation loop:

    browsers (any number, any refresh rate)
        --HTTP controls--> DashboardState (thread-safe store)
        --> one background thread runs ACMFDSMissionRunner (real physics)
        --> every logged step is appended to a shared history
        --WebSocket--> browsers poll *copies* of the history (read-only)

Consequences of this design (verified, not aspirational):
* The physics loop runs at its own fixed timestep on its own thread and is
  never stepped by a request handler — closing, opening, or refreshing a
  browser cannot change the simulation.
* All control inputs are validated server-side (pydantic) and are applied to
  the actual simulation (scenario goal/altitude/battery/wind + ACMFDS and
  mission cruise-speed limits + live moving-obstacle enable).
* Every value shown in the UI comes from the logged per-step rows — no
  synthesized frontend values; graphs plot the same rows that are written
  to the CSV flight log.

Run:
    python -m dashboard.server          -> http://127.0.0.1:8000
"""

import asyncio
import dataclasses
import json
import math
import os
import sys
import threading
from typing import Dict, List, Optional

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from scenarios import SCENARIOS, get_scenario
from simulation.acmfds_mission import ACMFDSMissionRunner
from simulation.wind import WindConfig
from telemetry.gps import GpsReference
from telemetry.logger import EpisodeSummary

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
CSV_PATH = os.path.join(_PROJECT_ROOT, "results", "dashboard_run.csv")


# ----------------------------------------------------------------------
# Validated operator configuration
# ----------------------------------------------------------------------
class FlightConfig(BaseModel):
    """Operator inputs. Ranges are enforced server-side (422 on violation)."""

    scenario: str = "normal"
    target_altitude: float = Field(2.0, ge=0.5, le=4.5, description="m")
    goal_x: float = Field(4.5, ge=-7.0, le=7.0, description="m")
    goal_y: float = Field(0.0, ge=-4.5, le=4.5, description="m")
    cruise_speed: float = Field(1.2, ge=0.2, le=1.5, description="m/s")
    initial_battery: float = Field(100.0, ge=5.0, le=100.0, description="%")
    wind_enabled: bool = True
    wind_speed: float = Field(0.5, ge=0.0, le=8.0, description="m/s")
    wind_direction_deg: float = Field(0.0, ge=-360.0, le=360.0, description="deg CCW from +X")
    dynamic_obstacles_enabled: bool = True
    # Open the native PyBullet window for THIS run (same flight, same loop —
    # the window is a third output of the one simulation, not a second sim).
    native_window: bool = False


def build_scenario(cfg: FlightConfig):
    """Overlay the validated operator configuration onto a base scenario.

    Only environment/initial conditions change; ACMFDS thresholds stay at
    their documented defaults in every run.
    """
    base = get_scenario(cfg.scenario)
    angle = math.radians(cfg.wind_direction_deg)
    wind = WindConfig(
        enabled=cfg.wind_enabled,
        base_speed=cfg.wind_speed,
        direction=(math.cos(angle), math.sin(angle), 0.0),
        gust_amplitude=base.wind.gust_amplitude,
        gust_period=base.wind.gust_period,
        seed=base.wind.seed,
    )
    battery = dataclasses.replace(
        base.battery, initial_charge_percent=cfg.initial_battery
    )
    moving = list(base.moving_obstacles) if cfg.dynamic_obstacles_enabled else []
    return dataclasses.replace(
        base,
        goal=(cfg.goal_x, cfg.goal_y),
        cruise_altitude=cfg.target_altitude,
        wind=wind,
        battery=battery,
        moving_obstacles=moving,
    )


class MovingToggle(BaseModel):
    enabled: bool


class GpsSetup(BaseModel):
    """Simulated-GPS display settings (never a real GPS receiver).

    ``ref_lat``/``ref_lon`` are the geographic origin. None means NO origin
    is configured — the dashboard must show "Origin not set", never fall back
    to a fabricated coordinate pair.
    """

    enabled: bool = False
    ref_lat: Optional[float] = Field(None, ge=-90.0, le=90.0)
    ref_lon: Optional[float] = Field(None, ge=-180.0, le=180.0)


# ----------------------------------------------------------------------
# Thread-safe run state
# ----------------------------------------------------------------------
class RunControl:
    """Cross-thread control flags (plain bools — atomic under the GIL)."""

    def __init__(self) -> None:
        self.pause_requested = False
        self.stop_requested = False
        self.moving_enabled = True


def _summary_dict(summary: Optional[EpisodeSummary]) -> Optional[Dict]:
    if summary is None:
        return None
    return {
        "steps": summary.steps,
        "duration_s": round(summary.duration_s, 2),
        "mission_completed": summary.mission_completed,
        "landed": summary.landed,
        "collision_count": summary.collision_count,
        "min_obstacle_distance": (
            None if math.isinf(summary.min_obstacle_distance)
            else round(summary.min_obstacle_distance, 3)
        ),
        "mean_altitude_error": round(summary.mean_altitude_error, 3),
        "max_altitude_error": round(summary.max_altitude_error, 3),
        "battery_end_percent": round(summary.battery_end_percent, 1),
        "battery_consumed_percent": round(summary.battery_consumed_percent, 1),
        "final_altitude": round(summary.final_altitude, 3),
        "final_vertical_speed": round(summary.final_vertical_speed, 3),
        "emergency_events": summary.emergency_events,
        "emergency_steps": summary.emergency_steps,
        "safe_landing_events": summary.safe_landing_events,
        "safe_landing_steps": summary.safe_landing_steps,
        "wind_mean": round(summary.wind_mean, 2),
        "wind_max": round(summary.wind_max, 2),
        "decisions": summary.decisions,
    }


class DashboardState:
    """Single-flight simulation manager + telemetry history (thread-safe)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.config = FlightConfig()
        self.gps = GpsSetup()
        self.status = "idle"          # idle | running | paused | finished
        self.control: Optional[RunControl] = None
        self.thread: Optional[threading.Thread] = None
        self.history: List[Dict] = []
        self.world: Optional[Dict] = None   # static world geometry for the 3D view
        self.drone_start: List[float] = [-4.0, 0.0]  # from the active scenario
        self.outcome = ""
        self.reason = ""
        self.summary: Optional[Dict] = None

    # -- readers --------------------------------------------------------
    def snapshot(self, since: int) -> Dict:
        with self._lock:
            total = len(self.history)
            rows = self.history[since:] if 0 <= since <= total else self.history
            moving = (
                self.control.moving_enabled if self.control is not None
                else self.config.dynamic_obstacles_enabled
            )
            origin_set = self.gps.ref_lat is not None and self.gps.ref_lon is not None
            gps_out = {
                "enabled": self.gps.enabled,
                "origin_set": origin_set,
                # The geographic origin corresponds EXACTLY to world position
                # (X=0, Y=0) of the PyBullet simulation frame (X=east, Y=north).
                # The drone's spawn point is scenario-defined (default (-4, 0)
                # — 4 m west of the origin); it is reported, never assumed away.
                "origin_world": [0.0, 0.0],
                "drone_start_world": list(self.drone_start),
                "ref_lat": self.gps.ref_lat,
                "ref_lon": self.gps.ref_lon,
                "lat": None,
                "lon": None,
                "t": None,          # sim timestamp of the row the fix came from
                "simulated": True,
            }
            if self.gps.enabled and origin_set and self.history:
                # Authoritative state only: the coordinates are computed from
                # the last logged physics row, never from a separate animation.
                last = self.history[-1]
                lat, lon = GpsReference(self.gps.ref_lat, self.gps.ref_lon).fix(
                    last["position_x"], last["position_y"]
                )
                gps_out["lat"] = round(lat, 7)
                gps_out["lon"] = round(lon, 7)
                gps_out["t"] = last["timestamp"]
            return {
                "status": self.status,
                "total": total,
                "rows": rows,
                "world": self.world,
                "outcome": self.outcome,
                "reason": self.reason,
                "summary": self.summary,
                "config": self.config.model_dump(),
                "scenario_name": self.config.scenario,
                "moving_enabled": moving,
                "gps": gps_out,
            }

    # -- controls -------------------------------------------------------
    def start(self, cfg: FlightConfig) -> Dict:
        with self._lock:
            if self.status in ("running", "paused"):
                return {"ok": False, "error": "a run is already active — stop or reset it first"}
            scenario = build_scenario(cfg)
            runner = ACMFDSMissionRunner(
                scenario, gui=cfg.native_window, seed=scenario.seed, csv_path=CSV_PATH,
            )
            # Operator cruise speed feeds BOTH the mission planner and the
            # ACMFDS speed cap (the composition rule uses min() of them).
            runner.mission.config.cruise_speed = cfg.cruise_speed
            runner.acmfds.config.cruise_speed = cfg.cruise_speed
            control = RunControl()
            control.moving_enabled = cfg.dynamic_obstacles_enabled

            self.config = cfg
            self.control = control
            self.history = []
            self.world = None
            self.drone_start = [float(scenario.start[0]), float(scenario.start[1])]
            self.outcome = ""
            self.reason = ""
            self.summary = None
            self.status = "running"

        def loop() -> None:
            try:
                runner.setup()
                # World geometry for the browser 3D view: the actual obstacles
                # this flight was spawned with (identical to the CSV's world).
                with self._lock:
                    self.world = runner.describe_world()
                report = runner.run(realtime=True, on_step=self._on_step, control=control)
                with self._lock:
                    self.status = "finished"
                    self.outcome = report.outcome
                    self.reason = report.termination_reason
                    self.summary = _summary_dict(report.summary)
            except Exception as exc:  # keep the server alive on sim failure
                with self._lock:
                    self.status = "finished"
                    self.outcome = "error"
                    self.reason = f"{type(exc).__name__}: {exc}"
            finally:
                try:
                    runner.close()
                except Exception:
                    pass

        self.thread = threading.Thread(target=loop, name="acmfds-sim", daemon=True)
        self.thread.start()
        return {"ok": True}

    def _on_step(self, row: Dict) -> None:
        with self._lock:
            self.history.append(row)

    def pause(self) -> Dict:
        with self._lock:
            if self.status != "running" or self.control is None:
                return {"ok": False, "error": "no active run to pause"}
            self.control.pause_requested = True
            self.status = "paused"
            return {"ok": True}

    def resume(self) -> Dict:
        with self._lock:
            if self.status != "paused" or self.control is None:
                return {"ok": False, "error": "no paused run to resume"}
            self.control.pause_requested = False
            self.status = "running"
            return {"ok": True}

    def stop(self) -> Dict:
        with self._lock:
            if self.status not in ("running", "paused") or self.control is None:
                return {"ok": False, "error": "no active run to stop"}
            self.control.pause_requested = False
            self.control.stop_requested = True
            return {"ok": True}

    def reset(self) -> Dict:
        with self._lock:
            if self.status in ("running", "paused"):
                return {"ok": False, "error": "stop the active run before resetting"}
            self.history = []
            self.world = None
            self.outcome = ""
            self.reason = ""
            self.summary = None
            self.control = None
            self.status = "idle"
            return {"ok": True}

    def set_gps(self, cfg: GpsSetup) -> Dict:
        with self._lock:
            self.gps = cfg
            return {"ok": True, "simulated": True}

    def set_moving(self, enabled: bool) -> Dict:
        with self._lock:
            if self.control is not None:
                # Live toggle: the physics loop reads this flag every step.
                self.control.moving_enabled = bool(enabled)
                self.config = self.config.model_copy(
                    update={"dynamic_obstacles_enabled": bool(enabled)}
                )
                return {"ok": True, "live": True}
            self.config = self.config.model_copy(
                update={"dynamic_obstacles_enabled": bool(enabled)}
            )
            return {"ok": True, "live": False}


# ----------------------------------------------------------------------
# FastAPI application
# ----------------------------------------------------------------------
app = FastAPI(title="ACMFDS Live Dashboard", version="1.0")
state = DashboardState()

if os.path.isdir(STATIC_DIR):
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


@app.get("/")
def index() -> FileResponse:
    return FileResponse(os.path.join(STATIC_DIR, "index.html"))


@app.get("/api/scenarios")
def api_scenarios() -> List[Dict[str, str]]:
    return [{"name": n, "description": s.description} for n, s in SCENARIOS.items()]


@app.get("/api/status")
def api_status() -> Dict:
    snap = state.snapshot(-1)
    snap.pop("rows", None)
    return snap


@app.post("/api/start")
def api_start(cfg: FlightConfig) -> Dict:
    return state.start(cfg)


@app.post("/api/pause")
def api_pause() -> Dict:
    return state.pause()


@app.post("/api/resume")
def api_resume() -> Dict:
    return state.resume()


@app.post("/api/stop")
def api_stop() -> Dict:
    return state.stop()


@app.post("/api/reset")
def api_reset() -> Dict:
    return state.reset()


@app.post("/api/moving")
def api_moving(body: MovingToggle) -> Dict:
    return state.set_moving(body.enabled)


@app.post("/api/gps")
def api_gps(body: GpsSetup) -> Dict:
    return state.set_gps(body)


@app.websocket("/ws")
async def ws_telemetry(websocket: WebSocket) -> None:
    """Push incremental telemetry: the client sends nothing, the sim loop
    is never touched from here. Poll cadence is UI-only."""
    await websocket.accept()
    since = 0
    try:
        while True:
            snap = state.snapshot(since)
            since = snap["total"]
            await websocket.send_text(json.dumps(snap))
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        return
    except Exception:
        return


def main() -> None:
    import uvicorn
    print("ACMFDS dashboard: http://127.0.0.1:8000")
    uvicorn.run(app, host="127.0.0.1", port=8000, log_level="warning")


if __name__ == "__main__":
    main()
