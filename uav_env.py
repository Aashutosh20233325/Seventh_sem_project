import math

import gymnasium as gym
import numpy as np
import pygame
from gymnasium import spaces


class UAVEnv(gym.Env):
    """
    Step 1 UAV environment.

    Features:
    - 2D UAV navigation
    - rectangular obstacles
    - randomized safe starting position
    - randomized target
    - 49-ray LiDAR
    - collision detection
    - obstacle-avoidance reward shaping
    - Gymnasium reset()/step() API
    - Pygame visualization
    """

    metadata = {"render_modes": ["human"]}

    def __init__(self, render_mode="human"):
        super().__init__()

        # ========================================================
        # SIMULATION
        # ========================================================
        self.width = 900
        self.height = 800

        self.dt = 1.0
        self.max_steps = 1000
        self.current_step = 0

        # ========================================================
        # UAV
        # ========================================================
        self.uav_radius = 12.0

        # Defaults; reset() will randomize these safely.
        self.start_x = 130.0
        self.start_y = 400.0
        self.start_theta = 0.0

        self.x = self.start_x
        self.y = self.start_y
        self.theta = self.start_theta

        self.max_speed = 3.0
        self.max_angular_speed = 0.055

        # ========================================================
        # LiDAR
        # ========================================================
        self.num_rays = 49
        self.lidar_max_distance = 220.0

        self.lidar_readings = np.full(
            self.num_rays,
            self.lidar_max_distance,
            dtype=np.float32,
        )
        self.lidar_points = []
        self.lidar_hit_types = []
        self.lidar_object_labels = np.zeros(
            self.num_rays,
            dtype=np.float32,
        )

        # Target information derived ONLY from LiDAR detection.
        self.target_detected = False
        self.target_lidar_distance = 0.0
        self.target_lidar_angle = 0.0

        # ========================================================
        # TARGET
        # ========================================================
        self.target_radius = 12.0
        self.target = np.array(
            [780.0, 180.0],
            dtype=np.float32,
        )

        # ========================================================
        # OBSTACLES
        # ========================================================
        self.obstacles = [
            pygame.Rect(260, 120, 150, 90),
            pygame.Rect(520, 250, 210, 90),
            pygame.Rect(180, 380, 150, 120),
            pygame.Rect(430, 500, 180, 90),
            pygame.Rect(700, 530, 130, 130),
        ]

        # ========================================================
        # GYMNASIUM SPACES
        # ========================================================
        # [normalized speed, normalized angular velocity]
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32,
        )

        # Observation:
        #   49 LiDAR rays × 2 values:
        #       normalized distance
        #       object label
        #   3 target-sensor values:
        #       target_detected
        #       target distance measured by LiDAR
        #       target angle measured by the detecting ray
        #   2 previous-action values
        #
        # Total = 49*2 + 3 + 2 = 103

        self.observation_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(103,),
            dtype=np.float32,
        )

        self.previous_action = np.zeros(
            2,
            dtype=np.float32,
        )

        # ========================================================
        # RENDERING
        # ========================================================
        self.screen = None
        self.clock = None
        self.font = None
        self.small_font = None
        self.title_font = None

        self.render_mode = render_mode

    # ============================================================
    # GEOMETRY / LIDAR
    # ============================================================

  
    @staticmethod
    def ray_rectangle_intersection(
        origin_x,
        origin_y,
        direction_x,
        direction_y,
        rect,
        max_distance,
    ):
        """
        Return the nearest ray/rectangle intersection distance,
        or None if the ray does not intersect the rectangle.
        """

        xmin = rect.left
        xmax = rect.right
        ymin = rect.top
        ymax = rect.bottom

        t_values = []

        # Intersections with vertical sides
        if abs(direction_x) > 1e-9:
            t = (xmin - origin_x) / direction_x
            if 0 <= t <= max_distance:
                y_hit = origin_y + t * direction_y
                if ymin <= y_hit <= ymax:
                    t_values.append(t)

            t = (xmax - origin_x) / direction_x
            if 0 <= t <= max_distance:
                y_hit = origin_y + t * direction_y
                if ymin <= y_hit <= ymax:
                    t_values.append(t)

        # Intersections with horizontal sides
        if abs(direction_y) > 1e-9:
            t = (ymin - origin_y) / direction_y
            if 0 <= t <= max_distance:
                x_hit = origin_x + t * direction_x
                if xmin <= x_hit <= xmax:
                    t_values.append(t)

            t = (ymax - origin_y) / direction_y
            if 0 <= t <= max_distance:
                x_hit = origin_x + t * direction_x
                if xmin <= x_hit <= xmax:
                    t_values.append(t)

        if not t_values:
            return None

        return min(t_values)

    @staticmethod
    def ray_circle_intersection(
        origin_x,
        origin_y,
        direction_x,
        direction_y,
        center_x,
        center_y,
        radius,
        max_distance,
    ):
        """
        Return the nearest intersection distance between
        a ray and a circle.

        Returns None if there is no intersection.
        """

        # Vector from ray origin to circle center
        oc_x = center_x - origin_x
        oc_y = center_y - origin_y

        # Projection of oc onto ray direction
        t_ca = (
            oc_x * direction_x
            + oc_y * direction_y
        )

        # Circle is behind the ray
        if t_ca < 0:
            return None

        # Squared distance from circle center to ray
        d2 = (
            oc_x * oc_x
            + oc_y * oc_y
            - t_ca * t_ca
        )

        radius_sq = radius * radius

        if d2 > radius_sq:
            return None

        # Distance from projection point to intersection
        t_hc = math.sqrt(
            max(0.0, radius_sq - d2)
        )

        # Nearest intersection
        t0 = t_ca - t_hc

        if t0 < 0:
            t0 = t_ca + t_hc

        if 0 <= t0 <= max_distance:
            return t0

        return None

    # def lidar_scan(self):
    #     """Perform a 49-ray 360-degree LiDAR scan."""

    #     readings = []
    #     hit_points = []
    #     hit_obstacle = []

    #     angle_step = 2.0 * math.pi / self.num_rays

    #     for i in range(self.num_rays):
    #         angle = self.theta + i * angle_step

    #         dx = math.cos(angle)
    #         dy = math.sin(angle)

    #         closest_distance = self.lidar_max_distance

    #         # Check obstacles.
    #         for obstacle in self.obstacles:
    #             distance = self.ray_rectangle_intersection(
    #                 self.x,
    #                 self.y,
    #                 dx,
    #                 dy,
    #                 obstacle,
    #                 self.lidar_max_distance,
    #             )

    #             if distance is not None:
    #                 closest_distance = min(
    #                     closest_distance,
    #                     distance,
    #                 )

    #         # Check simulation boundaries.
    #         if dx > 1e-9:
    #             closest_distance = min(
    #                 closest_distance,
    #                 (self.width - self.x) / dx,
    #             )
    #         elif dx < -1e-9:
    #             closest_distance = min(
    #                 closest_distance,
    #                 (0.0 - self.x) / dx,
    #             )

    #         if dy > 1e-9:
    #             closest_distance = min(
    #                 closest_distance,
    #                 (self.height - self.y) / dy,
    #             )
    #         elif dy < -1e-9:
    #             closest_distance = min(
    #                 closest_distance,
    #                 (0.0 - self.y) / dy,
    #             )

    #         closest_distance = max(
    #             0.0,
    #             closest_distance,
    #         )

    #         hit = (
    #             closest_distance
    #             < self.lidar_max_distance - 1.0
    #         )

    #         end_x = self.x + dx * closest_distance
    #         end_y = self.y + dy * closest_distance

    #         readings.append(closest_distance)
    #         hit_points.append((end_x, end_y))
    #         hit_obstacle.append(hit)

    #     self.lidar_readings = np.asarray(
    #         readings,
    #         dtype=np.float32,
    #     )
    #     self.lidar_points = hit_points
    #     self.lidar_hits = hit_obstacle

    #     return self.lidar_readings
    def lidar_scan(self):
        """
        Perform a 49-ray 360-degree LiDAR scan.

        For every ray we store:
            - distance to the first object/boundary
            - object label

        Object label:
            0.0 = obstacle / boundary / no target
            1.0 = target

        Target information is exposed separately ONLY when one or
        more LiDAR rays actually detect the target.
        """

        readings = []
        object_labels = []
        hit_points = []
        hit_types = []

        # Reset target detection for this scan.
        self.target_detected = False
        self.target_lidar_distance = 0.0
        self.target_lidar_angle = 0.0

        angle_step = 2.0 * math.pi / self.num_rays

        # Keep the best (nearest) target detection if multiple rays
        # intersect the target.
        best_target_distance = None
        best_target_angle = 0.0

        for i in range(self.num_rays):

            # Ray 0 is aligned with the UAV heading.
            relative_angle = i * angle_step

            # Convert the ray angle into [-pi, pi].
            if relative_angle > math.pi:
                relative_angle -= 2.0 * math.pi

            angle = self.theta + i * angle_step

            dx = math.cos(angle)
            dy = math.sin(angle)

            # ----------------------------------------------------
            # Closest obstacle distance
            # ----------------------------------------------------
            obstacle_distance = self.lidar_max_distance

            for obstacle in self.obstacles:

                distance = self.ray_rectangle_intersection(
                    self.x,
                    self.y,
                    dx,
                    dy,
                    obstacle,
                    self.lidar_max_distance,
                )

                if distance is not None:
                    obstacle_distance = min(
                        obstacle_distance,
                        distance,
                    )

            # ----------------------------------------------------
            # Boundary distance
            # ----------------------------------------------------
            boundary_distance = self.lidar_max_distance

            if dx > 1e-9:
                boundary_distance = min(
                    boundary_distance,
                    (self.width - self.x) / dx,
                )

            elif dx < -1e-9:
                boundary_distance = min(
                    boundary_distance,
                    (0.0 - self.x) / dx,
                )

            if dy > 1e-9:
                boundary_distance = min(
                    boundary_distance,
                    (self.height - self.y) / dy,
                )

            elif dy < -1e-9:
                boundary_distance = min(
                    boundary_distance,
                    (0.0 - self.y) / dy,
                )

            # ----------------------------------------------------
            # Target intersection
            # ----------------------------------------------------
            target_distance = self.ray_circle_intersection(
                self.x,
                self.y,
                dx,
                dy,
                float(self.target[0]),
                float(self.target[1]),
                self.target_radius,
                self.lidar_max_distance,
            )

            # The first thing along the ray is what the LiDAR sees.
            closest_distance = min(
                obstacle_distance,
                boundary_distance,
            )

            object_label = 0.0
            hit_type = "obstacle"

            # Target is detectable only if it is not occluded by
            # an obstacle or the environment boundary.
            if (
                target_distance is not None
                and target_distance < closest_distance
            ):
                closest_distance = target_distance
                object_label = 1.0
                hit_type = "target"

                if (
                    best_target_distance is None
                    or target_distance < best_target_distance
                ):
                    best_target_distance = target_distance
                    best_target_angle = relative_angle

            # ----------------------------------------------------
            # Save ray information
            # ----------------------------------------------------
            closest_distance = max(
                0.0,
                closest_distance,
            )

            end_x = self.x + dx * closest_distance
            end_y = self.y + dy * closest_distance

            readings.append(closest_distance)
            object_labels.append(object_label)
            hit_points.append((end_x, end_y))
            hit_types.append(hit_type)

        # --------------------------------------------------------
        # Store LiDAR results
        # --------------------------------------------------------
        self.lidar_readings = np.asarray(
            readings,
            dtype=np.float32,
        )

        self.lidar_object_labels = np.asarray(
            object_labels,
            dtype=np.float32,
        )

        self.lidar_points = hit_points
        self.lidar_hit_types = hit_types

        # --------------------------------------------------------
        # Store target information ONLY if LiDAR detected it
        # --------------------------------------------------------
        if best_target_distance is not None:
            self.target_detected = True
            self.target_lidar_distance = float(
                best_target_distance
            )
            self.target_lidar_angle = float(
                best_target_angle
            )

        return self.lidar_readings

    # ============================================================
    # TARGET / COLLISION
    # ============================================================

    def distance_to_target(self):
        return math.hypot(
            float(self.target[0]) - self.x,
            float(self.target[1]) - self.y,
        )

    def target_relative_angle(self):
        """Angle from UAV heading to target in [-pi, pi]."""

        dx = float(self.target[0]) - self.x
        dy = float(self.target[1]) - self.y

        target_angle = math.atan2(dy, dx)
        relative = target_angle - self.theta

        while relative > math.pi:
            relative -= 2.0 * math.pi

        while relative < -math.pi:
            relative += 2.0 * math.pi

        return relative

    def is_safe_position(self, x, y, min_clearance=80.0):
        """Check clearance from obstacles and simulation boundaries."""

        if (
            x < self.uav_radius + min_clearance
            or x > self.width - self.uav_radius - min_clearance
            or y < self.uav_radius + min_clearance
            or y > self.height - self.uav_radius - min_clearance
        ):
            return False

        for obstacle in self.obstacles:
            closest_x = max(
                obstacle.left,
                min(x, obstacle.right),
            )
            closest_y = max(
                obstacle.top,
                min(y, obstacle.bottom),
            )

            distance = math.hypot(
                x - closest_x,
                y - closest_y,
            )

            if distance < min_clearance:
                return False

        return True

    def generate_start_position(self):
        """Generate a random UAV start with 80 px clearance."""

        for _ in range(1000):
            x = float(
                self.np_random.uniform(
                    self.uav_radius + 80.0,
                    self.width - self.uav_radius - 80.0,
                )
            )

            y = float(
                self.np_random.uniform(
                    self.uav_radius + 80.0,
                    self.height - self.uav_radius - 80.0,
                )
            )

            if self.is_safe_position(
                x,
                y,
                min_clearance=80.0,
            ):
                return x, y

        # Known safe fallback.
        return self.start_x, self.start_y

    def generate_target(self):
        """Generate a random target with obstacle clearance."""

        for _ in range(1000):
            tx = float(
                self.np_random.uniform(
                    self.uav_radius + 40.0,
                    self.width - self.uav_radius - 40.0,
                )
            )

            ty = float(
                self.np_random.uniform(
                    self.uav_radius + 40.0,
                    self.height - self.uav_radius - 40.0,
                )
            )

            # Avoid a trivial target near the start.
            if math.hypot(tx - self.x, ty - self.y) < 250.0:
                continue

            if self.is_safe_position(
                tx,
                ty,
                min_clearance=40.0,
            ):
                return np.array(
                    [tx, ty],
                    dtype=np.float32,
                )

        return np.array(
            [780.0, 180.0],
            dtype=np.float32,
        )

    def check_collision(self):
        """
        UAV is modeled as a circle of radius self.uav_radius.

        Collision occurs when its center is <= that radius from
        an obstacle or the outer boundary.
        """

        for obstacle in self.obstacles:
            closest_x = max(
                obstacle.left,
                min(self.x, obstacle.right),
            )
            closest_y = max(
                obstacle.top,
                min(self.y, obstacle.bottom),
            )

            dx = self.x - closest_x
            dy = self.y - closest_y

            distance = math.hypot(dx, dy)

            if distance <= self.uav_radius:
                return True

        if (
            self.x <= self.uav_radius
            or self.x >= self.width - self.uav_radius
            or self.y <= self.uav_radius
            or self.y >= self.height - self.uav_radius
        ):
            return True

        return False

    # ============================================================
    # OBSERVATION
    # ============================================================

    def get_observation(self):
        """
        Return the 103-dimensional observation.

        LiDAR:
            49 rays × [normalized distance, object label]

        Target information:
            - target_detected: 0 or 1
            - target distance: ONLY from a detecting LiDAR ray
            - target angle: ONLY from the detecting LiDAR ray

        If the target is not detected:
            target_detected = 0
            target_distance = 0
            target_angle = 0

        Plus the previous 2D action.
        """

        lidar_distance_norm = np.clip(
            self.lidar_readings / self.lidar_max_distance,
            0.0,
            1.0,
        )

        observation = []

        # 49 × [distance, object label]
        for distance, object_label in zip(
            lidar_distance_norm,
            self.lidar_object_labels,
        ):
            observation.append(float(distance))
            observation.append(float(object_label))

        # --------------------------------------------------------
        # Target information from LiDAR only
        # --------------------------------------------------------
        if self.target_detected:
            target_detected = 1.0

            target_distance_norm = np.clip(
                self.target_lidar_distance
                / self.lidar_max_distance,
                0.0,
                1.0,
            )

            target_angle_norm = (
                self.target_lidar_angle / math.pi
            )
        else:
            target_detected = 0.0
            target_distance_norm = 0.0
            target_angle_norm = 0.0

        observation.extend(
            [
                target_detected,
                target_distance_norm,
                target_angle_norm,
            ]
        )

        # Previous action
        observation.extend(
            self.previous_action.astype(np.float32)
        )

        observation = np.asarray(
            observation,
            dtype=np.float32,
        )

        return observation

    # ============================================================
    # GYMNASIUM API
    # ============================================================

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self.current_step = 0

        # Randomize safe starting position and orientation.
        self.x, self.y = self.generate_start_position()

        self.theta = float(
            self.np_random.uniform(
                -math.pi,
                math.pi,
            )
        )

        self.previous_action = np.zeros(
            2,
            dtype=np.float32,
        )

        # Randomize target after start position is known.
        self.target = self.generate_target()

        self.lidar_scan()

        observation = self.get_observation()

        info = {
            "distance_to_target": self.distance_to_target(),
            "target_angle": self.target_relative_angle(),
            "target_detected": self.target_detected,
            "target_lidar_distance": self.target_lidar_distance,
            "target_lidar_angle": self.target_lidar_angle,
            "min_lidar": float(np.min(self.lidar_readings)),
            "step": self.current_step,
        }

        return observation, info

    def step(self, action):
        action = np.asarray(
            action,
            dtype=np.float32,
        )
        action = np.clip(
            action,
            self.action_space.low,
            self.action_space.high,
        )

        # --------------------------------------------------------
        # Measurements before movement
        # --------------------------------------------------------
        old_distance = self.distance_to_target()
        old_min_lidar = float(
            np.min(self.lidar_readings)
        )

        # --------------------------------------------------------
        # Convert normalized action to physical values
        # --------------------------------------------------------
        velocity = (
            float(action[0])
            * self.max_speed
        )

        angular_velocity = (
            float(action[1])
            * self.max_angular_speed
        )

        # --------------------------------------------------------
        # Move
        # --------------------------------------------------------
        self.theta += (
            angular_velocity
            * self.dt
        )

        self.x += (
            velocity
            * math.cos(self.theta)
            * self.dt
        )

        self.y += (
            velocity
            * math.sin(self.theta)
            * self.dt
        )

        self.previous_action = action.copy()
        self.current_step += 1

        # --------------------------------------------------------
        # Check state after movement
        # --------------------------------------------------------
        collision = self.check_collision()

        new_distance = self.distance_to_target()

        reached = (
            new_distance <= self.target_radius
        )

        # Update LiDAR.
        self.lidar_scan()

        new_min_lidar = float(
            np.min(self.lidar_readings)
        )

        # # --------------------------------------------------------
        # # Reward
        # # --------------------------------------------------------
        # reward = -0.05  # small time penalty

        # # Target progress.
        # reward += (
        #     old_distance
        #     - new_distance
        # )

        # # --------------------------------------------------------
        # # Obstacle avoidance shaping
        # # --------------------------------------------------------
        # SAFE_DISTANCE = 70.0

        # if new_min_lidar < SAFE_DISTANCE:
        #     danger = (
        #         SAFE_DISTANCE
        #         - new_min_lidar
        #     )

        #     # Penalize getting too close.
        #     reward -= 0.8 * danger

        #     # Reward increasing clearance when already near
        #     # an obstacle.
        #     lidar_improvement = (
        #         new_min_lidar
        #         - old_min_lidar
        #     )

        #     reward += 0.5 * lidar_improvement

        # # Collision is strongly penalized.
        # if collision:
        #     reward -= 100.0

        # # Successful arrival.
        # if reached:
        #     reward += 100.0
       
        # --------------------------------------------------------
        # Reward
        # --------------------------------------------------------

        # Time penalty: encourages completing the task quickly
        reward = -0.1

        # Reward getting closer to the target
        reward += old_distance - new_distance

        # Reward forward movement
        if velocity > 0:
            reward += 0.5

        # Strong collision penalty
        if collision:
            reward -= 100.0

        # Strong target reward
        if reached:
            reward += 100.0

        terminated = (
            collision
            or reached
        )

        truncated = (
            self.current_step
            >= self.max_steps
        )

        observation = self.get_observation()

        info = {
            "distance_to_target": new_distance,
            "collision": collision,
            "reached_target": reached,
            "min_lidar": new_min_lidar,
            "step": self.current_step,
        }

        return (
            observation,
            float(reward),
            terminated,
            truncated,
            info,
        )

    # ============================================================
    # RENDERING
    # ============================================================

    def _ensure_rendering(self):
        if self.screen is None:
            pygame.init()

            self.screen = pygame.display.set_mode(
                (self.width + 300, self.height)
            )

            pygame.display.set_caption(
                "UAV Search Environment - LiDAR Target Detection"
            )

            self.clock = pygame.time.Clock()

            self.font = pygame.font.SysFont(
                "consolas",
                18,
            )
            self.small_font = pygame.font.SysFont(
                "consolas",
                15,
            )
            self.title_font = pygame.font.SysFont(
                "consolas",
                22,
                bold=True,
            )

    def draw_grid(self):
        grid_color = (210, 214, 220)
        grid_size = 50

        for gx in range(
            0,
            self.width,
            grid_size,
        ):
            pygame.draw.line(
                self.screen,
                grid_color,
                (gx, 0),
                (gx, self.height),
                1,
            )

        for gy in range(
            0,
            self.height,
            grid_size,
        ):
            pygame.draw.line(
                self.screen,
                grid_color,
                (0, gy),
                (self.width, gy),
                1,
            )

    def draw_lidar(self):
        lidar_clear = (80, 170, 230)
        lidar_hit = (230, 100, 70)
        lidar_closest = (180, 50, 200)
        lidar_target = (20, 180, 70)

        if len(self.lidar_readings) == 0:
            return

        closest_index = int(
            np.argmin(self.lidar_readings)
        )

        for i, (point, hit_type) in enumerate(
            zip(
                self.lidar_points,
                self.lidar_hit_types,
            )
        ):
            if hit_type == "target":
                color = lidar_target
                width = 4
            elif i == closest_index:
                color = lidar_closest
                width = 4
            elif hit_type == "obstacle":
                color = lidar_hit
                width = 2
            else:
                color = lidar_clear
                width = 1

            pygame.draw.line(
                self.screen,
                color,
                (self.x, self.y),
                point,
                width,
            )

            if hit_type in ("target", "obstacle"):
                pygame.draw.circle(
                    self.screen,
                    color,
                    (
                        int(point[0]),
                        int(point[1]),
                    ),
                    3,
                )

    def draw_obstacles(self):
        obstacle_color = (80, 85, 90)
        obstacle_border = (45, 48, 52)

        for obstacle in self.obstacles:
            pygame.draw.rect(
                self.screen,
                obstacle_color,
                obstacle,
            )
            pygame.draw.rect(
                self.screen,
                obstacle_border,
                obstacle,
                3,
            )

    def draw_target(self):
        target_color = (25, 180, 70)
        target_border = (10, 110, 40)

        target_pos = (
            int(self.target[0]),
            int(self.target[1]),
        )

        pygame.draw.circle(
            self.screen,
            target_color,
            target_pos,
            int(self.target_radius),
        )

        pygame.draw.circle(
            self.screen,
            target_border,
            target_pos,
            int(self.target_radius),
            2,
        )

    def draw_uav(self):
        uav_color = (40, 90, 220)
        uav_border = (20, 45, 110)

        size = 18

        front = (
            self.x
            + size * math.cos(self.theta),
            self.y
            + size * math.sin(self.theta),
        )

        left = (
            self.x
            + size * math.cos(
                self.theta + 2.5
            ),
            self.y
            + size * math.sin(
                self.theta + 2.5
            ),
        )

        right = (
            self.x
            + size * math.cos(
                self.theta - 2.5
            ),
            self.y
            + size * math.sin(
                self.theta - 2.5
            ),
        )

        points = [
            front,
            left,
            right,
        ]

        pygame.draw.polygon(
            self.screen,
            uav_color,
            points,
        )

        pygame.draw.polygon(
            self.screen,
            uav_border,
            points,
            2,
        )

        heading_length = 30

        heading_end = (
            self.x
            + heading_length * math.cos(self.theta),
            self.y
            + heading_length * math.sin(self.theta),
        )

        pygame.draw.line(
            self.screen,
            uav_border,
            (self.x, self.y),
            heading_end,
            3,
        )

    def draw_panel(self):
        panel_x = self.width

        panel_color = (250, 250, 250)
        panel_border = (180, 185, 190)
        text_color = (30, 30, 30)

        pygame.draw.rect(
            self.screen,
            panel_color,
            (
                panel_x,
                0,
                300,
                self.height,
            ),
        )

        pygame.draw.line(
            self.screen,
            panel_border,
            (panel_x, 0),
            (panel_x, self.height),
            2,
        )

        title = self.title_font.render(
            "UAV STATUS",
            True,
            text_color,
        )

        self.screen.blit(
            title,
            (panel_x + 20, 25),
        )

        pygame.draw.line(
            self.screen,
            panel_border,
            (panel_x + 20, 60),
            (self.width + 280, 60),
            1,
        )

        lines = [
            f"X       : {self.x:7.1f}",
            f"Y       : {self.y:7.1f}",
            (
                "Heading : "
                f"{math.degrees(self.theta) % 360:7.1f}°"
            ),
            "",
            "TARGET SENSOR",
            (
                "Detected: "
                f"{'YES' if self.target_detected else 'NO'}"
            ),
            (
                "Distance: "
                f"{self.target_lidar_distance if self.target_detected else 0.0:7.1f}"
            ),
            (
                "Angle   : "
                f"{math.degrees(self.target_lidar_angle) if self.target_detected else 0.0:7.1f}°"
            ),
            "",
            "LiDAR",
            f"Rays    : {self.num_rays}",
            (
                "Min dist: "
                f"{np.min(self.lidar_readings):7.1f}"
            ),
            (
                "Max dist: "
                f"{np.max(self.lidar_readings):7.1f}"
            ),
            "",
            "EPISODE",
            f"Step    : {self.current_step}",
            f"Limit   : {self.max_steps}",
            "",
            "CONTROLS",
            "W : forward",
            "S : backward",
            "A : rotate left",
            "D : rotate right",
            "ESC : quit",
        ]

        current_y = 85

        for line in lines:
            rendered = self.small_font.render(
                line,
                True,
                text_color,
            )

            self.screen.blit(
                rendered,
                (panel_x + 20, current_y),
            )

            current_y += 28

    def render(self):
        if self.render_mode != "human":
            return

        self._ensure_rendering()

        self.screen.fill(
            (235, 238, 242)
        )

        self.draw_grid()
        self.draw_lidar()
        self.draw_obstacles()
        self.draw_target()
        self.draw_uav()
        self.draw_panel()

        pygame.display.flip()
        self.clock.tick(60)

    def close(self):
        if self.screen is not None:
            pygame.quit()
            self.screen = None
            self.clock = None
