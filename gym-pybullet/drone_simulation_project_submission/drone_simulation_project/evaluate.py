"""Evaluation and benchmarking for ACMFDS mission scenarios.

Runs N headless episodes of a scenario through the full ACMFDS pipeline and
writes aggregate metrics to ``results/evaluation_results.csv``. All metrics
are computed from actually logged flight rows — nothing is synthesized.

An optional RL-policy evaluation path (Stable-Baselines3) is retained for
legacy comparison and imports sb3 lazily so it never blocks the default
non-RL usage.
"""

import os
import sys
import csv
import argparse
from typing import Any, Dict, List, Optional

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from scenarios import SCENARIO_NAMES, get_scenario
from telemetry.logger import compute_summary


def evaluate_scenario(
    scenario_name: str = "normal",
    num_episodes: int = 3,
    seed: int = 42,
    output_csv: str = "results/evaluation_results.csv",
    max_duration: Optional[float] = None,
) -> List[Dict[str, Any]]:
    """Run several ACMFDS mission episodes and record aggregate metrics."""
    from simulation.acmfds_mission import run_acmfds_mission

    scenario = get_scenario(scenario_name)
    results: List[Dict[str, Any]] = []

    print("\n" + "=" * 68)
    print(f"ACMFDS SCENARIO BENCHMARK: '{scenario_name}' x {num_episodes} episodes")
    print("=" * 68)
    print(scenario.description)
    print("=" * 68 + "\n")

    for ep in range(1, num_episodes + 1):
        csv_path = os.path.join("results", f"eval_{scenario_name}_ep{ep}.csv")
        report = run_acmfds_mission(
            scenario,
            gui=False,
            seed=seed + ep - 1,
            csv_path=csv_path,
            max_duration=max_duration,
        )
        s = report.summary
        row = {
            "episode": ep,
            "scenario": scenario_name,
            "outcome": report.outcome,
            "mission_completed": bool(report.mission_completed),
            "duration_s": round(report.duration_s, 2),
            "steps": report.steps,
            "min_obstacle_distance": (
                round(s.min_obstacle_distance, 3) if s else None
            ),
            "mean_altitude_error": round(s.mean_altitude_error, 3) if s else None,
            "max_altitude_error": round(s.max_altitude_error, 3) if s else None,
            "battery_consumed_percent": round(s.battery_consumed_percent, 2) if s else None,
            "emergency_events": s.emergency_events if s else None,
            "safe_landing_events": s.safe_landing_events if s else None,
            "wind_mean": round(s.wind_mean, 2) if s else None,
        }
        results.append(row)
        print(
            f"Ep {ep:2d} | {report.outcome:14s} | completed={report.mission_completed!s:5s} | "
            f"steps={report.steps:5d} | minObs={row['min_obstacle_distance']}m | "
            f"altErr={row['mean_altitude_error']}m | battUsed={row['battery_consumed_percent']}% | "
            f"emerg={row['emergency_events']}"
        )

    # Aggregate summary
    completed = sum(1 for r in results if r["mission_completed"])
    collisions = sum(1 for r in results if r["outcome"] == "collision")
    print("\n" + "-" * 68)
    print(f"Completion rate  : {completed}/{len(results)}")
    print(f"Collisions       : {collisions}/{len(results)}")
    valid_min = [r["min_obstacle_distance"] for r in results if r["min_obstacle_distance"] is not None]
    if valid_min:
        print(f"Min obstacle dist: {min(valid_min):.3f} m (best across episodes)")
    print("-" * 68)

    full_path = os.path.join(PROJECT_ROOT, output_csv)
    os.makedirs(os.path.dirname(full_path), exist_ok=True)
    with open(full_path, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        writer.writeheader()
        writer.writerows(results)
    print(f"Results written to: {full_path}\n")
    return results


def evaluate_rl_policy(
    model_path: str,
    algo: str = "ppo",
    num_episodes: int = 3,
    seed: int = 42,
) -> None:
    """Optional legacy RL evaluation (requires stable-baselines3)."""
    try:
        from stable_baselines3 import PPO, SAC
    except ImportError:
        print("[INFO] stable-baselines3 not installed; RL evaluation unavailable.")
        return

    from envs.drone_env import DroneEnv

    model = SAC.load(model_path) if algo.lower() == "sac" else PPO.load(model_path)
    env = DroneEnv(render_mode=None, max_episode_steps=600)
    print(f"Evaluating RL model {model_path} ({algo}) over {num_episodes} episodes...")
    for ep in range(1, num_episodes + 1):
        obs, _ = env.reset(seed=seed + ep)
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            done = terminated or truncated
        print(
            f"Ep {ep}: reached={info.get('target_reached')} collision={info.get('collision')}"
        )
    env.close()


def parse_args():
    parser = argparse.ArgumentParser(description="ACMFDS scenario benchmark (non-RL default).")
    parser.add_argument("--scenario", choices=SCENARIO_NAMES, default="normal")
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", type=str, default="results/evaluation_results.csv")
    parser.add_argument("--max-duration", type=float, default=None)
    parser.add_argument("--rl-model", type=str, default=None,
                        help="Optional legacy RL policy path (requires stable-baselines3)")
    parser.add_argument("--algo", choices=["ppo", "sac"], default="ppo")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.rl_model:
        evaluate_rl_policy(args.rl_model, algo=args.algo, num_episodes=args.episodes, seed=args.seed)
    else:
        evaluate_scenario(
            scenario_name=args.scenario,
            num_episodes=args.episodes,
            seed=args.seed,
            output_csv=args.output,
            max_duration=args.max_duration,
        )
