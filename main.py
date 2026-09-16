import pygame

from uav_env import UAVEnv


def keyboard_action():
    keys = pygame.key.get_pressed()

    speed = 0.0
    angular_velocity = 0.0

    if keys[pygame.K_w]:
        speed = 1.0

    if keys[pygame.K_s]:
        speed = -1.0

    if keys[pygame.K_a]:
        angular_velocity = -1.0

    if keys[pygame.K_d]:
        angular_velocity = 1.0

    return [speed, angular_velocity]


def main():

    env = UAVEnv(render_mode="human")

    obs, info = env.reset()

    # Initialize the Pygame window before reading events.
    env.render()

    running = True

    try:
        while running:

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    running = False

                elif (
                    event.type == pygame.KEYDOWN
                    and event.key == pygame.K_ESCAPE
                ):
                    running = False

            action = keyboard_action()

            obs, reward, terminated, truncated, info = env.step(
                action
            )

            env.render()

            if terminated or truncated:
                obs, info = env.reset()

    finally:
        env.close()


if __name__ == "__main__":
    main()