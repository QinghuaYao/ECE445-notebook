import gymnasium
from gymnasium import spaces
import pygame
import cv2
import numpy as np
from pupil_apriltags import Detector
import math
# import tensorflow as tf
import time
import logging
import random
import os

FEET_TO_PIXELS = 50
INCHES_TO_PIXELS = FEET_TO_PIXELS / 12
WIDTH, HEIGHT = int(16 * FEET_TO_PIXELS), int(16 * FEET_TO_PIXELS)
WALL_THICKNESS = int(0.25 * FEET_TO_PIXELS)
FPS = 60

BLUE_BOT_WIDTH = 16
BLUE_BOT_LENGTH = 16
RED_BOT_WIDTH = 16
RED_BOT_LENGTH = 16
RED_BOT_SIZE = (int(RED_BOT_WIDTH * INCHES_TO_PIXELS), int(RED_BOT_LENGTH * INCHES_TO_PIXELS))
HAMMER_LENGTH = int(14 * INCHES_TO_PIXELS)
BLUE_BOT_SIZE = (int(BLUE_BOT_WIDTH * INCHES_TO_PIXELS), int(BLUE_BOT_LENGTH * INCHES_TO_PIXELS))

WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
GRAY = (100, 100, 100)

pygame.font.init()
DEFAULT_FONT = pygame.font.SysFont("Arial", 24)

SPEED = 16
ROTATION_SPEED = 16
FRICTION = 0.5

FULL_DETECTION_INTERVAL = 10  # Run full detection every 10 frames
ROI_MARGIN = 50  # Extra pixels around the detected tag for ROI
LK_PARAMS = dict(winSize=(15, 15), maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
DECISION_MARGIN_THRESHOLD = 20  # Reject weak detections
MAX_MOVEMENT_THRESHOLD = 50  # Max allowed movement between frames to filter outliers

logging.basicConfig(filename='battlebots.log', level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s')


def create_detector(quad_decimate):
    return Detector(families='tag16h5',
                    nthreads=4,
                    quad_decimate=quad_decimate,
                    quad_sigma=0.8,
                    refine_edges=True)


# Video capture setup
cap = cv2.VideoCapture(3)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 60)

tags = (0, 1)

def calibrate_camera():
    img_points = np.array([
        [0, 0],
        [800, 0],
        [0, 800],
        [800, 800],
        [400, 400],
        [0, 400],
        [400, 0],
        [400, 800],
        [800, 400],
    ], dtype=np.float32)

    obj_points = np.array([
        [0.292, 0.687, 1.161],
        [-0.875, -0.093, 1.588],
        [1.681, -0.317, 2.241],
        [0.246, -1.144, 2.446],
        [0.277, -0.213, 1.859],
        [1.054, 0.291, 1.736],
        [-0.390, 0.336, 1.316],
        [1.046, -0.847, 2.474],
        [-0.417, -0.648, 1.997],
    ], dtype=np.float32)

    fx, fy, cx, cy = 850, 850, 740, 425
    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0,  0,  1]], dtype=np.float32)
    dist_coeffs = np.zeros(5)

    success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
    if not success:
        raise RuntimeError("PnP failed")

    return rvec, tvec, camera_matrix, dist_coeffs

def fallback_calibration():
    img_points = np.array([
        [0, 0],
        [800, 0],
        [0, 800],
        [800, 800],
        [400, 400],
        [0, 400],
        [400, 0],
        [400, 800],
        [800, 400],
    ], dtype=np.float32)

    obj_points = np.array([
        [900, 995],
        [324, 315],
        [1462, 389],
        [933, 60],
        [915, 360],
        [1235, 618],
        [573, 580],
        [1154, 200],
        [687, 173]
    ], dtype=np.float32)

    obj_points = np.hstack([obj_points, np.zeros((obj_points.shape[0], 1))])

    fx, fy, cx, cy = 850, 850, 740, 425
    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0,  0,  1]], dtype=np.float32)
    dist_coeffs = np.zeros(5)

    success, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
    if not success:
        raise RuntimeError("PnP failed")

    return rvec, tvec, camera_matrix, dist_coeffs

rvec, tvec, cam_mat, dist = calibrate_camera()
rvec_fallback, tvec_fallback, cam_mat_fallback, dist_fallback = fallback_calibration()


class BattlebotsEnv(gymnasium.Env):
    """
    A custom Gymnasium environment for a Battlebots simulation.
    The red robot is controlled by an agent; the blue robot acts as the target.
    Observations include positions, velocities, angles, angle differences, distance, and strike status.
    Actions are a continuous 3D array: [left drive, right drive, hammer input].
    """

    def __init__(self):
        super().__init__()

        self.action_space = spaces.Box(low=np.array([-1, -1, -1]),
                                       high=np.array([1, 1, 1]),
                                       dtype=np.float32)

        max_distance = math.sqrt(WIDTH ** 2 + HEIGHT ** 2)
        self.observation_space = spaces.Box(
            low=np.array([0, 0, -20, -20, 0, 0, 0, 0, -180, 0, 0]),
            high=np.array([WIDTH, HEIGHT, 20, 20, 360, WIDTH, HEIGHT, 360, 180, max_distance, 1]),
            dtype=np.float32
        )

        self.red_robot = {
            "rect": pygame.Rect((WIDTH / 2) - WALL_THICKNESS, (HEIGHT / 2) - WALL_THICKNESS, *RED_BOT_SIZE),
            "angle": 0,
            "velocity": [0, 0],
            "hammer_cooldown": 0,
            "hammer_active": False,
            "striking": False
        }
        self.blue_robot = {
            "rect": pygame.Rect(1 / 4 * WIDTH - WALL_THICKNESS - BLUE_BOT_SIZE[0],
                                WALL_THICKNESS - BLUE_BOT_SIZE[1],
                                *BLUE_BOT_SIZE),
            "angle": 180,
            "velocity": [0, 0]
        }

        self.blue_controls = {pygame.K_a: (0, 1), pygame.K_d: (1, 0)}
        self.blue_input = [0, 0]

        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Battlebots gymnasium")
        self.clock = pygame.time.Clock()
        self.timer = 0.0
        self.actions = []
        self.start_time = time.perf_counter()

        self.previous_distance = None
        self.reward = 0

        self.quad_dec = 0.5
        self.at_detector = create_detector(self.quad_dec)

        # cap.set(cv2.CAP_PROP_ZOOM, 2.0)

        # Tracking variables
        self.tags_dict = {}  # Store tag positions: {tag_id: (x, y)}
        self.tag_Rs = {}
        self.poses = {}
        self.frame_count = 0  # Track frame number
        self.prev_gray = None  # Previous grayscale frame for Optical Flow
        self.camera_matrix = camera_matrix = np.array([
            [850, 0, 740],
            [0, 850, 425],
            [0,   0,   1]
        ], dtype=np.float32)

        self.end_full_pass = 0
        self.start_full_pass = 0

    def _april_tags(self):
        ret, frame = cap.read()
        if not ret:
            return

        start_time = time.perf_counter()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        # gray = clahe.apply(gray)
        # gray = cv2.medianBlur(gray, 3)
        # Full detection every FULL_DETECTION_INTERVAL frames or if we have no tracked tags.
        if self.frame_count % FULL_DETECTION_INTERVAL == 0 or not self.tags_dict:
            detected_tags = self.at_detector.detect(gray, estimate_tag_pose=True,
                                                    camera_params=(850, 850, 740, 425), tag_size=0.1294)
            # If no tags detected, try increasing quad_decimate (faster detection on downsampled image)
            if len(detected_tags) == 0 and self.quad_dec == 0.5:
                self.quad_dec = 2.0
                self.at_detector = create_detector(self.quad_dec)
            elif len(detected_tags) > 0 and self.quad_dec != 0.5:
                # If we detect tags with the higher resolution detector, revert back.
                self.quad_dec = 0.5
                self.at_detector = create_detector(self.quad_dec)

            # Clear tracked tags and update from full detection.
            self.tags_dict.clear()
            self.poses.clear()
            for tag in detected_tags:
                # Optionally, you can check decision margin here if needed.
                # print(f"Tag ID {tag.tag_id} — margin: {tag.decision_margin:.2f}")
                if tag.decision_margin >= DECISION_MARGIN_THRESHOLD:
                    self.tags_dict[tag.tag_id] = tag.center.astype(int)
                    self.tag_Rs[tag.tag_id] = tag.pose_R
                    self.poses[tag.tag_id] = tag.pose_t
        else:
            # Use Optical Flow to track previous tag positions
            if self.prev_gray is not None and self.tags_dict:
                prev_pts = np.array(list(self.tags_dict.values()), dtype=np.float32).reshape(-1, 1, 2)
                new_pts, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, prev_pts, None, **LK_PARAMS)
                # Update tag positions if tracking is successful and movement is reasonable
                for i, (new, st) in enumerate(zip(new_pts, status)):
                    if st == 1:
                        tag_id = list(self.tags_dict.keys())[i]
                        new_pos = tuple(new.ravel().astype(int))
                        old_pos = self.tags_dict[tag_id]
                        if np.linalg.norm(np.array(new_pos) - np.array(old_pos)) < MAX_MOVEMENT_THRESHOLD:
                            self.tags_dict[tag_id] = new_pos
        end_time = time.perf_counter()
        self.prev_gray = gray.copy()  # Save current frame for next Optical Flow calculation

        # Draw detected & tracked tags on frame
        for tag_id, center in self.tags_dict.items():
            x, y = center
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(frame, f'ID: {tag_id}', (x - 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            # Optional: define ROI based on tag center (not used for detection here)

        # Display frame (create window once outside loop in a production system)
        cv2.namedWindow("AprilTag Detector", cv2.WINDOW_NORMAL)
        cv2.imshow("AprilTag Detector", frame)
        cv2.resizeWindow("AprilTag Detector", 600, 400)

        self.frame_count += 1

        # print(f"April Execution Time: {(end_time - start_time) * 1e3:.2f} ms")

    def pose_t_to_pixel_with_heading(self, pose_t, pose_R):
        """
        Projects 3D pose_t and returns pixel location + heading (0/90/180/270) from pose_R.
        """
        # Project point to pixel
        world_point = np.array(pose_t, dtype=np.float32).reshape(1, 1, 3)
        pixel_coords, _ = cv2.projectPoints(world_point, rvec, tvec, self.camera_matrix, None)
        px, py = tuple(pixel_coords[0][0])

        # Extract yaw
        yaw_rad = np.arctan2(pose_R[1, 0], pose_R[0, 0])
        yaw_deg = (np.degrees(yaw_rad)) + 40 % 360

        return (px, py), ((360 - yaw_deg + 90) % 360)

    # def fallback_pos_calculation(self, tag_center):


    def _handle_user_input(self):
        keys = pygame.key.get_pressed()
        self.blue_input = [0, 0]
        for key, (left, right) in self.blue_controls.items():
            if keys[key]:
                self.blue_input[0] += left
                self.blue_input[1] += right

    def step(self, action):
        """
        Execute one step of the environment.
        Action is expected as a 3D numpy array: [left_drive, right_drive, hammer_input].
        """

        self.end_full_pass = time.perf_counter()
        # print(f"Full Pass Execution Time: {(self.end_full_pass - self.start_full_pass) * 1e3:.2f} ms")
        self.start_full_pass = time.perf_counter()
        start_time = time.perf_counter()
        if not isinstance(action, np.ndarray) or action.shape != (3,):
            raise ValueError("Action must be a 3D numpy array of shape (3,)")

        self._april_tags()
        if 11 in self.poses:
            blue_pos, blue_ang = self.pose_t_to_pixel_with_heading(self.poses[11], self.tag_Rs[11])
            self.blue_robot["rect"].center = blue_pos
            self.blue_robot["angle"] = blue_ang
        else:
            if 11 in self.tags_dict:
                tag_2d = self.tags_dict[11]  # shape: (2,)
                tag_3d = np.array([[tag_2d[0], tag_2d[1], 0]], dtype=np.float32)  # shape: (1, 3)
                pixel_coords, _ = cv2.projectPoints(tag_3d, rvec_fallback, tvec_fallback, self.camera_matrix, None)
                self.blue_robot["rect"].center = pixel_coords
        if 6 in self.poses:
            red_pos, red_ang = self.pose_t_to_pixel_with_heading(self.poses[6], self.tag_Rs[6])
            self.red_robot["rect"].center = red_pos
            self.red_robot["angle"] = (red_ang - 90) % 360
        else:
            if 6 in self.tags_dict:
                tag_2d = self.tags_dict[6]  # shape: (2,)
                tag_3d = np.array([[tag_2d[0], tag_2d[1], 0]], dtype=np.float32)  # shape: (1, 3)
                pixel_coords, _ = cv2.projectPoints(tag_3d, rvec_fallback, tvec_fallback, self.camera_matrix, None)
                self.red_robot["rect"].center = pixel_coords

        # action = np.clip(action, self.action_space.low, self.action_space.high)
        # self.actions = action
        # self._handle_user_input()
        # left_drive, right_drive, fire = action
        # logging.debug(
        #     f"Step started. Time since episode start: {(time.perf_counter() - self.start_time):.2f}s, Action: {action}")
        #
        # forward_speed = (left_drive + right_drive) * 10
        # turn_speed = (right_drive - left_drive) * 10
        #
        # self.red_robot["angle"] = (self.red_robot["angle"] + turn_speed) % 360
        # self.red_robot["velocity"] = [
        #     forward_speed * math.cos(math.radians(self.red_robot["angle"])),
        #     forward_speed * -math.sin(math.radians(self.red_robot["angle"]))
        # ]

        # if (self.cpu_Human):
        #     blue_turn_speed = (self.blue_input[1] - self.blue_input[0]) * ROTATION_SPEED
        #     self.blue_robot["angle"] = (self.blue_robot["angle"] + blue_turn_speed) % 360
        #     forward_blue = (self.blue_input[0] + self.blue_input[1]) / 2
        #     self.blue_robot["velocity"] = [
        #         forward_blue * SPEED * math.cos(math.radians(self.blue_robot["angle"])),
        #         forward_blue * SPEED * -math.sin(math.radians(self.blue_robot["angle"]))
        #     ]

        # self._apply_physics(self.red_robot)
        # self._apply_physics(self.blue_robot)

        # if fire > 0.9 and self.red_robot["hammer_cooldown"] == 0:
        #     self.red_robot["hammer_active"] = True
        #     self.red_robot["hammer_cooldown"] = FPS * 3 // 2
        # elif self.red_robot["hammer_cooldown"] > 0:
        #     self.red_robot["hammer_cooldown"] -= 1
        #     if self.red_robot["hammer_cooldown"] <= FPS * 1.1:
        #         self.red_robot["hammer_active"] = False

        # self._handle_collision(self.red_robot, self.blue_robot)

        # self.blue_robot["velocity"] = [
        #     4 * math.cos(math.radians(self.blue_robot["angle"])),
        #     4 * math.sin(math.radians(self.blue_robot["angle"]))
        # ]

        obs = self._get_observation()

        red_center = np.array(self.red_robot["rect"].center)
        blue_center = np.array(self.blue_robot["rect"].center)
        red_angle = self.red_robot["angle"] % 360
        # reward = self._calculate_reward(red_center, blue_center, red_angle, left_drive, right_drive)
        reward = 0
        # self.reward = reward

        # if self.red_robot["hammer_active"] and self.red_robot["striking"]:
        #     done = False
        # else:
        #     done = False
        done = False

        elapsed_time = time.perf_counter() - self.start_time
        truncated = elapsed_time >= self.TRUNCATION_TIME

        end_time = time.perf_counter()
        # print(f"Step Execution Time: {(end_time - start_time) * 1e3:.2f} ms")
        self.clock.tick(FPS)
        return obs, reward, done, truncated, {}

    def _is_blue_facing_red(self, blue_robot, red_robot, tolerance_degrees=45):
        """Returns True if the blue robot is facing the red robot within the given tolerance."""
        blue_center = np.array(blue_robot["rect"].center)
        red_center = np.array(red_robot["rect"].center)
        blue_angle = blue_robot["angle"] % 360

        angle_to_red = math.degrees(math.atan2(-(red_center[1] - blue_center[1]), red_center[0] - blue_center[0]))
        angle_to_red = (angle_to_red + 360) % 360

        angle_difference = abs(blue_angle - angle_to_red)
        angle_difference = min(angle_difference, 360 - angle_difference)
        return angle_difference <= tolerance_degrees

    def _calculate_reward(self, red_center, blue_center, red_angle, left_drive, right_drive):
        distance = np.linalg.norm(red_center - blue_center)
        if self.previous_distance is None:
            self.previous_distance = distance
        if distance < self.previous_distance:
            proximity_reward = 0.1
        else:
            proximity_reward = -0.1
        self.previous_distance = distance

        if left_drive < -0.5 and right_drive < -0.5:
            proximity_reward = -2

        angle_to_blue = math.degrees(math.atan2(blue_center[1] - red_center[1], blue_center[0] - red_center[0]))
        angle_to_blue = (angle_to_blue + 360) % 360
        angle_difference = (self.red_robot["angle"] - angle_to_blue + 180) % 360 - 180

        if abs(angle_difference) < 20:
            angle_reward = 0.5
        elif abs(angle_difference) > 90:
            angle_reward = -0.2
        else:
            angle_reward = -0.05 * abs(angle_difference) / 20

        if self.red_robot["hammer_active"]:
            if self.red_robot["striking"]:
                hammer_reward = 1000
            else:
                hammer_reward = -1
        else:
            hammer_reward = 0

        if self.red_robot["rect"].colliderect(self.blue_robot["rect"]):
            if self.red_robot["striking"]:
                collision_reward = 10
            elif self._is_blue_facing_red(self.blue_robot, self.red_robot):
                collision_reward = -10
            else:
                collision_reward = 0
        else:
            collision_reward = 0

        if np.array(self.blue_robot["rect"].center)[0] - (BLUE_BOT_LENGTH / 2) < WALL_THICKNESS or \
                np.array(self.blue_robot["rect"].center)[0] + (BLUE_BOT_LENGTH / 2) > WIDTH - WALL_THICKNESS:
            collision_reward += 0.1
        elif np.array(self.blue_robot["rect"].center)[1] - (BLUE_BOT_WIDTH / 2) < WALL_THICKNESS or \
                np.array(self.blue_robot["rect"].center)[1] + (BLUE_BOT_WIDTH / 2) > HEIGHT - WALL_THICKNESS:
            collision_reward += 0.1

        if np.linalg.norm(self.red_robot["velocity"]) < 0.1:
            movement_reward = -0.1
        else:
            movement_reward = 0.05

        return proximity_reward + angle_reward + hammer_reward + collision_reward + movement_reward

    def reset(self, randomize=False, truncation_time=100000, cpu_Human=False):
        self.red_robot = {
            "rect": pygame.Rect((WIDTH / 2) - WALL_THICKNESS, (HEIGHT / 2) - WALL_THICKNESS, *RED_BOT_SIZE),
            "angle": 0,
            "velocity": [0, 0],
            "hammer_cooldown": 0,
            "hammer_active": False,
            "striking": False
        }
        self.previous_distance = None
        self.TRUNCATION_TIME = truncation_time
        self.min_x = WALL_THICKNESS
        self.min_y = WALL_THICKNESS
        self.max_x = WIDTH - WALL_THICKNESS
        self.max_y = HEIGHT - WALL_THICKNESS
        self.cpu_Human = cpu_Human
        if randomize:
            max_iterations = 100
            iterations = 0
            while iterations < max_iterations:
                self.blue_pos_start = [random.randint(self.min_x, self.max_x), random.randint(self.min_y, self.max_y)]
                self.blue_pos_end = [random.randint(self.min_x, self.max_x), random.randint(self.min_y, self.max_y)]
                self.blue_angle = math.degrees(math.atan2(self.blue_pos_end[0] - self.blue_pos_start[0],
                                                          self.blue_pos_end[0] - self.blue_pos_start[1]))
                # print(self.blue_pos_start, self.blue_pos_end, self.blue_angle)
                blue_rect = pygame.Rect(self.blue_pos_start[0], self.blue_pos_start[1], *BLUE_BOT_SIZE)
                if (abs(self.blue_angle - math.degrees(
                        math.atan2(self.blue_pos_start[1] - (HEIGHT / 2) - WALL_THICKNESS,
                                   self.blue_pos_start[0] - (WIDTH / 2) - WALL_THICKNESS))) > 45 and
                        abs(self.blue_angle - math.degrees(
                            math.atan2(self.blue_pos_start[1] - (HEIGHT / 2) - WALL_THICKNESS,
                                       self.blue_pos_start[0] - (WIDTH / 2) - WALL_THICKNESS))) < 120 and
                        not self.red_robot["rect"].colliderect(blue_rect)):
                    break
                iterations += 1
            if iterations == max_iterations:
                self.blue_pos_start = [WIDTH - BLUE_BOT_SIZE[0] - WALL_THICKNESS, WALL_THICKNESS]
                self.blue_angle = 270
        else:
            self.blue_pos_start = [WIDTH - BLUE_BOT_SIZE[0] - WALL_THICKNESS, WALL_THICKNESS]
            self.blue_angle = 270
        self.blue_robot = {
            "rect": pygame.Rect(self.blue_pos_start[0],
                                self.blue_pos_start[1],
                                *BLUE_BOT_SIZE),
            "angle": self.blue_angle,
            "velocity": [4 * math.cos(self.blue_angle * math.pi / 180), 4 * math.sin(self.blue_angle * math.pi / 180)]
        }
        logging.debug("Environment reset.")
        self.timer = 0.0
        self.start_time = time.perf_counter()
        return self._get_observation()

    def render(self, path=None):
        try:
            self.screen.fill(WHITE)
            self._draw_arena()
            self._draw_bot(self.red_robot, RED, self.red_robot["hammer_active"])
            self._draw_bot(self.blue_robot, BLUE, False)
            if (path is not None) and (len(path) > 1):
                pygame.draw.lines(self.screen, "green", False, path, 3)
            pygame.display.flip()
        except pygame.error as e:
            logging.error(f"Pygame error in render: {e}")
            pygame.quit()
            raise

    def close(self):
        pygame.quit()

    def _apply_physics(self, bot):
        bot["rect"].x += bot["velocity"][0]
        bot["rect"].y += bot["velocity"][1]
        bot["velocity"][0] *= FRICTION
        bot["velocity"][1] *= FRICTION
        bot["rect"].clamp_ip(pygame.Rect(WALL_THICKNESS, WALL_THICKNESS,
                                         WIDTH - 2 * WALL_THICKNESS, HEIGHT - 2 * WALL_THICKNESS))
        if bot is self.red_robot:
            rotated_center_x, rotated_center_y = self.red_robot["rect"].center
            hammer_end_x = rotated_center_x + HAMMER_LENGTH * math.cos(math.radians(self.red_robot["angle"]))
            hammer_end_y = rotated_center_y - HAMMER_LENGTH * math.sin(math.radians(self.red_robot["angle"]))
            self.red_robot["striking"] = self.blue_robot["rect"].collidepoint((hammer_end_x, hammer_end_y))

    def _handle_collision(self, bot1, bot2):
        if bot1["rect"].colliderect(bot2["rect"]):
            dx = bot1["rect"].centerx - bot2["rect"].centerx
            dy = bot1["rect"].centery - bot2["rect"].centery
            dist = math.hypot(dx, dy)
            if dist == 0:
                return
            nx, ny = dx / dist, dy / dist
            overlap = bot1["rect"].width / 2 + bot2["rect"].width / 2 - dist
            bot1["rect"].x += nx * overlap / 2
            bot1["rect"].y += ny * overlap / 2
            bot2["rect"].x -= nx * overlap / 2
            bot2["rect"].y -= ny * overlap / 2

    def _get_observation(self):
        red_center = np.array(self.red_robot["rect"].center)
        red_velocity = self.red_robot["velocity"]
        red_angle = self.red_robot["angle"] % 360
        blue_angle = self.blue_robot["angle"] % 360
        blue_center = np.array(self.blue_robot["rect"].center)
        red_center = np.clip(red_center, [WALL_THICKNESS, WALL_THICKNESS],
                             [WIDTH - WALL_THICKNESS, HEIGHT - WALL_THICKNESS])
        blue_center = np.clip(blue_center, [WALL_THICKNESS, WALL_THICKNESS],
                              [WIDTH - WALL_THICKNESS, HEIGHT - WALL_THICKNESS])
        front_x = math.cos(math.radians(blue_angle))
        front_y = -math.sin(math.radians(blue_angle))
        target_x = np.clip(blue_center[0] - (BLUE_BOT_WIDTH / 2) * front_x, 0, 800)
        target_y = np.clip(blue_center[1] - (BLUE_BOT_WIDTH / 2) * front_y, 0, 800)
        angle_to_blue = math.degrees(math.atan2((target_x) - (red_center[0]), target_y - red_center[1]))
        angle_to_blue = (angle_to_blue - 90) % 360

        angle_difference = (self.red_robot["angle"] - angle_to_blue + 180) % 360 - 180
        distance_between_robots = np.linalg.norm(red_center - blue_center)
        # print (red_angle, angle_to_blue, angle_difference, distance_between_robots)
        return np.array([
            red_center[0], red_center[1],
            red_velocity[0], red_velocity[1],
            red_angle,
            blue_center[0], blue_center[1],
            blue_angle, angle_difference, distance_between_robots, float(self.red_robot["striking"])
        ], dtype=np.float32)

    def _draw_arena(self):
        pygame.draw.rect(self.screen, GRAY, (0, 0, WIDTH, WALL_THICKNESS))
        pygame.draw.rect(self.screen, GRAY, (0, 0, WALL_THICKNESS, HEIGHT))
        pygame.draw.rect(self.screen, GRAY, (0, HEIGHT - WALL_THICKNESS, WIDTH, WALL_THICKNESS))
        pygame.draw.rect(self.screen, GRAY, (WIDTH - WALL_THICKNESS, 0, WALL_THICKNESS, HEIGHT))
        if len(self.actions) == 3:
            left_drive, right_drive, fire = self.actions
            info_text = f"Left: {left_drive:.2f}, Right: {right_drive:.2f}, Fire: {fire:.2f}, Time: {time.perf_counter() - self.start_time:.1f}, Reward: {self.reward:.3f}"
            text_surface = DEFAULT_FONT.render(info_text, True, BLACK)
            self.screen.blit(text_surface, (10, 10))

    def _draw_bot(self, bot, color, fired):
        rotated_bot = pygame.Surface(bot["rect"].size, pygame.SRCALPHA)
        rotated_bot.fill(color)
        center_x, center_y = rotated_bot.get_width() // 2, rotated_bot.get_height() // 2
        front_x = center_x + (bot["rect"].width // 2)
        front_y = center_y
        pygame.draw.line(rotated_bot, BLACK, (center_x, center_y), (front_x, front_y), 3)
        rotated_bot = pygame.transform.rotate(rotated_bot, bot["angle"])
        rect = rotated_bot.get_rect(center=bot["rect"].center)
        if fired:
            rotated_center = rect.center
            hammer_end_x = rotated_center[0] + HAMMER_LENGTH * math.cos(math.radians(bot["angle"]))
            hammer_end_y = rotated_center[1] - HAMMER_LENGTH * math.sin(math.radians(bot["angle"]))
            pygame.draw.line(self.screen, RED, rect.center, (hammer_end_x, hammer_end_y), 3)
        self.screen.blit(rotated_bot, rect.topleft)


if __name__ == "__main__":
    env = BattlebotsEnv()
    env.reset(randomize=False, truncation_time=9999)
    running = True
    while running:
        env.render()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        action = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        # keys = pygame.key.get_pressed()
        # if keys[pygame.K_a]:
        #     action[0] = 1.0
        # if keys[pygame.K_d]:
        #     action[1] = 1.0
        # if keys[pygame.K_SPACE]:
        #     action[2] = 1.0
        obs, reward, done, truncated, _ = env.step(action)
        if done or truncated:
            env.reset(randomize=False)
    env.close()