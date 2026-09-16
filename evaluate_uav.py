import pygame
from stable_baselines3 import PPO

from uav_env import UAVEnv


def main():

    env = UAVEnv(render_mode="human")
    model = PPO.load("uav_ppo")

    # Initialize window
    env.render()

    total_episodes = 50

    successful_episodes = 0
    collision_episodes = 0
    timeout_episodes = 0

    total_reward_all = 0.0
    total_steps_all = 0

    try:

        # ----------------------------------------------------------
        # Run evaluation episodes
        # ----------------------------------------------------------
        for episode in range(1, total_episodes + 1):

            obs, info = env.reset()

            terminated = False
            truncated = False
            total_reward = 0.0

            while not terminated and not truncated:

                # --------------------------------------------------
                # Pygame events
                # --------------------------------------------------
                for event in pygame.event.get():

                    if event.type == pygame.QUIT:
                        return

                    if (
                        event.type == pygame.KEYDOWN
                        and event.key == pygame.K_ESCAPE
                    ):
                        return

                # --------------------------------------------------
                # PPO action
                # --------------------------------------------------
                action, _ = model.predict(
                    obs,
                    deterministic=True
                )

                # --------------------------------------------------
                # Debug information
                # --------------------------------------------------
                min_lidar_before = env.lidar_readings.min()
                distance_before = env.distance_to_target()

                if min_lidar_before < 60:
                    print(
                        f"WARNING | "
                        f"episode={episode:2d} | "
                        f"step={env.current_step:4d} | "
                        f"min_lidar={min_lidar_before:7.2f} | "
                        f"distance={distance_before:7.2f} | "
                        f"speed={action[0]:6.3f} | "
                        f"turn={action[1]:6.3f}"
                    )

                if env.current_step % 20 == 0:
                    print(
                        f"Episode={episode} "
                        f"Step={env.current_step} "
                        f"X={env.x:.2f} "
                        f"Y={env.y:.2f} "
                        f"theta={env.theta:.2f} "
                        f"speed_action={action[0]:.4f} "
                        f"turn_action={action[1]:.4f}"
                    )

                # --------------------------------------------------
                # Environment step
                # --------------------------------------------------
                obs, reward, terminated, truncated, info = env.step(
                    action
                )

                total_reward += reward

                env.render()

                # --------------------------------------------------
                # Collision debugging
                # --------------------------------------------------
                if info["collision"]:

                    print("\n==============================")
                    print("        COLLISION")
                    print("==============================")

                    print(
                        "Episode:",
                        episode
                    )

                    print(
                        "Step:",
                        env.current_step
                    )

                    print(
                        "Position:",
                        f"({env.x:.2f}, {env.y:.2f})"
                    )

                    print(
                        "Heading:",
                        f"{env.theta:.3f} rad"
                    )

                    print(
                        "Action:",
                        action
                    )

                    print(
                        "Distance to target:",
                        f"{env.distance_to_target():.2f}"
                    )

                    print(
                        "Minimum LiDAR:",
                        f"{env.lidar_readings.min():.2f}"
                    )

                    print("\nLiDAR readings:")
                    print(env.lidar_readings)

                    print("==============================\n")

                    break

            # ------------------------------------------------------
            # Determine episode result
            # ------------------------------------------------------
            reached = info["reached_target"]
            collision = info["collision"]

            if reached:
                successful_episodes += 1
                result = "SUCCESS"

            elif collision:
                collision_episodes += 1
                result = "COLLISION"

            elif truncated:
                timeout_episodes += 1
                result = "TIMEOUT"

            else:
                # This should normally never happen
                timeout_episodes += 1
                result = "UNKNOWN/TIMEOUT"

            total_reward_all += total_reward
            total_steps_all += info["step"]

            # ------------------------------------------------------
            # Episode summary
            # ------------------------------------------------------
            print(
                f"\nEpisode {episode:2d} | "
                f"{result:9s} | "
                f"reward={total_reward:8.2f} | "
                f"steps={info['step']:4d} | "
                f"distance={info['distance_to_target']:7.2f}"
            )

            # ------------------------------------------------------
            # Running statistics
            # ------------------------------------------------------
            success_rate = (
                successful_episodes
                / episode
                * 100.0
            )

            collision_rate = (
                collision_episodes
                / episode
                * 100.0
            )

            timeout_rate = (
                timeout_episodes
                / episode
                * 100.0
            )

            print(
                f"Running statistics after {episode} episodes:"
            )

            print(
                f"  Success rate  : {success_rate:6.2f}%"
            )

            print(
                f"  Collision rate: {collision_rate:6.2f}%"
            )

            print(
                f"  Timeout rate  : {timeout_rate:6.2f}%"
            )

    finally:
        env.close()

    # ==========================================================
    # FINAL RESULTS
    # ==========================================================

    average_reward = (
        total_reward_all / total_episodes
    )

    average_steps = (
        total_steps_all / total_episodes
    )

    success_rate = (
        successful_episodes
        / total_episodes
        * 100.0
    )

    collision_rate = (
        collision_episodes
        / total_episodes
        * 100.0
    )

    timeout_rate = (
        timeout_episodes
        / total_episodes
        * 100.0
    )

    print("\n")
    print("================================================")
    print("              FINAL EVALUATION")
    print("================================================")

    print(
        f"Total episodes   : {total_episodes}"
    )

    print(
        f"Successful       : {successful_episodes}"
    )

    print(
        f"Collisions       : {collision_episodes}"
    )

    print(
        f"Timeouts         : {timeout_episodes}"
    )

    print(
        f"Success rate     : {success_rate:.2f}%"
    )

    print(
        f"Collision rate   : {collision_rate:.2f}%"
    )

    print(
        f"Timeout rate     : {timeout_rate:.2f}%"
    )

    print(
        f"Average reward   : {average_reward:.2f}"
    )

    print(
        f"Average steps    : {average_steps:.2f}"
    )

    print("================================================")


if __name__ == "__main__":
    main()