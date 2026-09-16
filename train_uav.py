from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

from uav_env import UAVEnv


def main():
    env = UAVEnv(render_mode=None)

    # Catch Gymnasium API issues before spending time training.
    check_env(env, warn=True)

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        ent_coef=0.01,
        seed=42,
    )

    model.learn(total_timesteps=500_000)

    model.save("uav_ppo")

    env.close()

    print("\nTraining complete.")
    print("Saved model: uav_ppo.zip")


if __name__ == "__main__":
    main()
