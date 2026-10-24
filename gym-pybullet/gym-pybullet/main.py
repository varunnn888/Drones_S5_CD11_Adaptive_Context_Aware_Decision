"""Main entry point — ACMFDS-driven autonomous quadcopter demonstration.

Default execution launches the Adaptive Context-Aware Multi-Objective Flight
Decision System (ACMFDS) flying a full takeoff -> hover -> cruise -> land
mission in real PyBullet physics. The default path is non-RL and requires
only pybullet and numpy.

Usage:
    python main.py                          # GUI demo, 'normal' scenario
    python main.py --headless               # headless run, same scenario
    python main.py --scenario obstacle      # obstacle ahead (Scenario B)
    python main.py --scenario low-battery   # Scenario C
    python main.py --scenario strong-wind   # Scenario D
    python main.py --scenario combined      # Scenario E
    python main.py --scenario normal --episodes 3 --seed 7

Optional legacy modes (clearly labelled, not part of the default path):
    python main.py --mode manual            # keyboard direct control (GUI)
"""

import os
import sys
import argparse

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from scenarios import SCENARIOS, SCENARIO_NAMES, get_scenario


def run_scenario(args, scenario_name: str, episode: int) -> None:
    """Run one scenario episode and print its report."""
    from simulation.acmfds_mission import run_acmfds_mission
    from telemetry.logger import format_summary

    scenario = get_scenario(scenario_name)
    csv_path = os.path.join("results", f"acmfds_log_{scenario_name}_{episode}.csv")
    if args.output:
        csv_path = args.output

    print("\n" + "=" * 68)
    print(f"SCENARIO {scenario_name.upper()} (episode {episode}, seed {args.seed})")
    print("=" * 68)
    print(scenario.description)
    print(f"GUI={not args.headless} | goal={scenario.goal} | "
          f"wind={'on' if scenario.wind.enabled else 'off'} "
          f"({scenario.wind.base_speed} m/s) | battery start="
          f"{scenario.battery.initial_charge_percent}%")
    print("=" * 68 + "\n")

    report = run_acmfds_mission(
        scenario,
        gui=not args.headless,
        seed=args.seed,
        csv_path=csv_path,
        realtime=args.realtime,
        max_duration=args.max_duration,
    )

    if report.summary is not None:
        print("\n" + format_summary(report.summary))
    print(f"Termination reason : {report.termination_reason}")
    print(f"Flight log (CSV)   : {report.csv_path}")
    print(f"Outcome            : {report.outcome.upper()}"
          + ("  [mission completed]" if report.mission_completed else "  [mission NOT completed]"))
    return report


def run_manual_mode() -> None:
    """Legacy manual keyboard control (optional feature, GUI only)."""
    print("\n[LEGACY MODE] Manual direct keyboard control (no ACMFDS).")
    from legacy_modes import run_mode_1_manual

    run_mode_1_manual(gui=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description="ACMFDS autonomous quadcopter demonstration (non-RL default).",
    )
    parser.add_argument(
        "--scenario",
        choices=SCENARIO_NAMES,
        default="normal",
        help="Test scenario: normal | obstacle | low-battery | strong-wind | combined",
    )
    parser.add_argument("--headless", action="store_true",
                        help="Run without GUI (DIRECT mode)")
    parser.add_argument("--episodes", type=int, default=1,
                        help="Number of repeated episodes (deterministic seed offset)")
    parser.add_argument("--seed", type=int, default=42, help="Base random seed")
    parser.add_argument("--max-duration", type=float, default=None,
                        help="Mission time limit in seconds (default: scenario default)")
    parser.add_argument("--realtime", action="store_true",
                        help="Pace GUI playback near real time (slower)")
    parser.add_argument("--output", type=str, default=None,
                        help="Custom CSV output path for the flight log")
    parser.add_argument("--list-scenarios", action="store_true",
                        help="Print scenario descriptions and exit")
    parser.add_argument("--mode", choices=["acmfds", "manual"], default="acmfds",
                        help="Execution mode: acmfds (default) or manual (legacy GUI keyboard control)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.list_scenarios:
        for name in SCENARIO_NAMES:
            sc = SCENARIOS[name]
            print(f"\n--- {name} ---\n{sc.description}")
        return 0

    if args.mode == "manual":
        run_manual_mode()
        return 0

    results = []
    for ep in range(1, args.episodes + 1):
        report = run_scenario(args, args.scenario, ep)
        results.append(report)

    if len(results) > 1:
        completed = sum(1 for r in results if r.mission_completed)
        print("\n" + "=" * 68)
        print(f"ACROSS {len(results)} EPISODES: {completed} completed, "
              f"{len(results) - completed} did not")
        print("=" * 68)

    # Non-zero exit if the single requested mission failed (useful for CI/tests).
    if len(results) == 1 and not results[0].mission_completed:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
