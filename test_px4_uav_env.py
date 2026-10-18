import numpy as np

from px4_uav_env import PX4UAVEnv


def main():

    env = PX4UAVEnv()

    try:

        obs, info = env.reset()

        print("RESET")
        print("Observation shape:", obs.shape)
        print("Observation:", obs)
        print("Info:", info)

        for step in range(10):

            # Simple safe action for testing.
            # No turning, low speed.
            action = np.array(
                [0.2, 0.0],
                dtype=np.float32,
            )

            (
                obs,
                reward,
                terminated,
                truncated,
                info,
            ) = env.step(action)

            truth = info["privileged"]
            print(
                f"step={step:02d} "
                f"reward={reward:.3f} "
                f"x={truth['x']:.3f} "
                f"y={truth['y']:.3f} "
                f"z={truth['z']:.3f} "
                f"target_detected={info['target_detected']} "
                f"collision={info['collision']}"
            )

            if terminated or truncated:

                print(
                    "Episode ended."
                )

                break

    finally:

        env.close()


if __name__ == "__main__":
    main()
