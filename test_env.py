from uav_env import UAVEnv


def main():
    env = UAVEnv(render_mode="human")

    obs, info = env.reset()

    print("Observation shape:", obs.shape)
    print("Action space:", env.action_space)
    print("Observation space:", env.observation_space)
    print("Initial target:", env.target)
    print("Max episode steps:", env.max_steps)
    print()

    step = 0

    try:
        while True:
            action = env.action_space.sample()

            obs, reward, terminated, truncated, info = env.step(action)
            env.render()

            print(
                f"Step {step:4d} | "
                f"Reward: {reward:8.3f} | "
                f"Distance: {info['distance_to_target']:8.3f} | "
                f"Collision: {info['collision']}"
            )

            step += 1

            if terminated or truncated:
                print()
                print("Episode finished.")
                print("  reached target:", info["reached_target"])
                print("  collision      :", info["collision"])
                print("  steps          :", info["step"])
                break

    finally:
        env.close()


if __name__ == "__main__":
    main()
