"""Train Reinforcement Learning agents (PPO or SAC) in the Drone Environment.

Uses Stable-Baselines3 for continuous actor-critic control.
Supports:
- PPO (Proximal Policy Optimization)
- SAC (Soft Actor-Critic)
- Deterministic seeding
- Obstacle randomization
- Periodic model checkpointing and CSV metric logging
"""

import os
import sys
import argparse
import time
from typing import Optional
import numpy as np

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from envs.drone_env import DroneEnv

try:
    from stable_baselines3 import PPO, SAC
    from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
    from stable_baselines3.common.vec_env import DummyVecEnv
except ImportError:
    raise ImportError("stable-baselines3 is required to run train.py. Install via pip install stable-baselines3.")


class TrainingMetricsCallback(BaseCallback):
    """Callback for tracking training metrics, successes, and collisions."""

    def __init__(self, log_freq: int = 1000, verbose: int = 1) -> None:
        super().__init__(verbose)
        self.log_freq = log_freq
        self.episode_rewards = []
        self.episode_lengths = []
        self.successes = []
        self.collisions = []

    def _on_step(self) -> bool:
        # Check infos for episode completions
        for info in self.locals.get("infos", []):
            if "episode" in info:
                self.episode_rewards.append(info["episode"]["r"])
                self.episode_lengths.append(info["episode"]["l"])
            if info.get("target_reached", False):
                self.successes.append(1)
            elif info.get("collision", False):
                self.collisions.append(1)

        if self.n_calls % self.log_freq == 0 and len(self.episode_rewards) > 0:
            avg_rew = np.mean(self.episode_rewards[-20:])
            avg_len = np.mean(self.episode_lengths[-20:])
            total_succ = sum(self.successes[-50:]) if self.successes else 0
            total_coll = sum(self.collisions[-50:]) if self.collisions else 0
            if self.verbose > 0:
                print(
                    f"Step: {self.n_calls:7d} | "
                    f"Avg Reward (last 20): {avg_rew:7.2f} | "
                    f"Avg Length: {avg_len:5.1f} | "
                    f"Successes (recent): {total_succ:3d} | "
                    f"Collisions: {total_coll:3d}"
                )
        return True


def train(
    algo: str = "ppo",
    total_timesteps: int = 50000,
    seed: int = 42,
    randomize_obstacles: bool = False,
    save_path: Optional[str] = None,
    log_dir: str = "logs",
) -> None:
    """Train continuous RL policy on DroneEnv."""
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(os.path.join(PROJECT_ROOT, "models"), exist_ok=True)

    if save_path is None:
        save_path = os.path.join(PROJECT_ROOT, "models", f"{algo.lower()}_drone.zip")

    print("\n" + "=" * 60)
    print(f"STARTING DRONE RL TRAINING: {algo.upper()}")
    print("=" * 60)
    print(f"Algorithm            : {algo.upper()}")
    print(f"Total Timesteps      : {total_timesteps:,}")
    print(f"Randomize Obstacles  : {randomize_obstacles}")
    print(f"Random Seed          : {seed}")
    print(f"Save Path            : {save_path}")
    print("=" * 60 + "\n")

    # Factory function for vectorized environment
    def make_env():
        return DroneEnv(
            render_mode=None,
            randomize_obstacles=randomize_obstacles,
            max_episode_steps=500,
        )

    env = DummyVecEnv([make_env])

    tb_log = None
    try:
        import tensorboard
        tb_log = os.path.join(log_dir, f"tb_{algo_lower}")
    except ImportError:
        pass

    # Algorithm configuration
    algo_lower = algo.lower()
    if algo_lower == "ppo":
        model = PPO(
            policy="MlpPolicy",
            env=env,
            learning_rate=3e-4,
            n_steps=1024,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.005,
            verbose=1,
            seed=seed,
            tensorboard_log=tb_log,
        )
    elif algo_lower == "sac":
        model = SAC(
            policy="MlpPolicy",
            env=env,
            learning_rate=3e-4,
            buffer_size=100000,
            batch_size=128,
            tau=0.005,
            gamma=0.99,
            ent_coef="auto",
            verbose=1,
            seed=seed,
            tensorboard_log=tb_log,
        )
    else:
        raise ValueError(f"Unsupported algorithm '{algo}'. Choose 'ppo' or 'sac'.")

    metrics_callback = TrainingMetricsCallback(log_freq=2000, verbose=1)

    start_time = time.time()
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=metrics_callback,
            progress_bar=False,
        )
    except KeyboardInterrupt:
        print("\nTraining interrupted by user. Saving current model...")
    finally:
        model.save(save_path)
        env.close()

    elapsed = time.time() - start_time
    print("\n" + "=" * 60)
    print(f"TRAINING COMPLETE IN {elapsed:.1f}s")
    print(f"Model saved to: {save_path}")
    print("=" * 60 + "\n")


def parse_args():
    parser = argparse.ArgumentParser(description="Train RL agent on DroneEnv.")
    parser.add_argument("--algo", type=str, choices=["ppo", "sac"], default="ppo", help="RL Algorithm (ppo or sac)")
    parser.add_argument("--timesteps", type=int, default=50000, help="Total training timesteps")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--randomize-obstacles", action="store_true", help="Randomize obstacles during training")
    parser.add_argument("--save-path", type=str, default=None, help="Custom model save path (.zip)")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    train(
        algo=args.algo,
        total_timesteps=args.timesteps,
        seed=args.seed,
        randomize_obstacles=args.randomize_obstacles,
        save_path=args.save_path,
    )
