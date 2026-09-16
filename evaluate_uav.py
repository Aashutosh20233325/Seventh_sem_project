# import pygame
# from stable_baselines3 import PPO

# from uav_env import UAVEnv


# def main():
#     env = UAVEnv(render_mode="human")
#     model = PPO.load("uav_ppo")

#     # Initialize the Pygame window before reading events.
#     env.render()

#     try:
#         for episode in range(10):
#             obs, info = env.reset()

#             terminated = False
#             truncated = False
#             total_reward = 0.0

#             while not terminated and not truncated:
#                 # Keep the window responsive.
#                 for event in pygame.event.get():
#                     if event.type == pygame.QUIT:
#                         return

#                     if (
#                         event.type == pygame.KEYDOWN
#                         and event.key == pygame.K_ESCAPE
#                     ):
#                         return

#                 action, _ = model.predict(
#                     obs,
#                     deterministic=True,
#                 )

#                 min_lidar = env.lidar_readings.min()

#                 if min_lidar < 60:
#                     print(
#                         f"WARNING | step={env.current_step} "
#                         f"min_lidar={min_lidar:.2f} "
#                         f"action=[{action[0]:.3f}, {action[1]:.3f}] "
#                         f"distance={env.distance_to_target():.2f}"
#                     )

#                 obs, reward, terminated, truncated, info = env.step(
#                     action
#                 )

#                 if info["collision"]:
#                     print("\nCOLLISION")
#                     print("Position:", (env.x, env.y))
#                     print("Heading:", env.theta)
#                     print("Action:", action)
#                     print("Minimum LiDAR:", env.lidar_readings.min())
#                     print("LiDAR:", env.lidar_readings)

#                 env.render()

#                 total_reward += reward

#             print(
#                 f"Episode {episode + 1}: "
#                 f"reward={total_reward:.2f}, "
#                 f"steps={info['step']}, "
#                 f"reached={info['reached_target']}, "
#                 f"collision={info['collision']}"
#             )

#     finally:
#         env.close()


# if __name__ == "__main__":
#     main()
import pygame
from stable_baselines3 import PPO

from uav_env import UAVEnv


def main():

    env = UAVEnv(render_mode="human")
    model = PPO.load("uav_ppo")

    # Initialize window
    env.render()

    try:

        # Run 50 evaluation episodes
        for episode in range(1, 51):

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

                # Information BEFORE the action
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
            # Episode result
            # ------------------------------------------------------
            print(
                f"Episode {episode}: "
                f"reward={total_reward:.2f}, "
                f"steps={info['step']}, "
                f"reached={info['reached_target']}, "
                f"collision={info['collision']}"
            )

    finally:
        env.close()


if __name__ == "__main__":
    main()