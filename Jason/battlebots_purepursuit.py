import numpy as np
import gymnasium
import pygame
from battlebots_env import BattlebotsEnv
import time
import serial
import math

def send_pwm_data(left, right, weapon):
    message = f"{left} {right} {weapon}\n"
    # print(f"Sending PWM - Left: {left}, Right: {right}, Weapon: {weapon}")
    ser.write(message.encode())  # Send data via serial
    ser.flush() 

try:
    ser = serial.Serial('COM7', 115200)
except serial.serialutil.SerialException:
    print("No connection to COM7!")
    quit()

left_drive_pwm = 0
right_drive_pwm = 0
weapon_drive_pwm = 0
last_time_fired = 0

# Hammer firing state machine (non-blocking)
class HammerFiring:
    def __init__(self):
        self.active = False
        self.start_time = 0.0

    def start(self):
        if not self.active:
            self.active = True
            self.start_time = time.time()

    def update(self):
        if not self.active:
            return None
        elapsed = time.time() - self.start_time
        if elapsed < 0.4:
            return 2000
        elif elapsed < 0.8:
            return 1500
        elif elapsed < 1.2:
            return 1000
        elif elapsed < 1.5:
            return 1500
        else:
            self.active = False
            return 1500  # revert to neutral

# Instantiate the hammer state machine
hammerFiring = HammerFiring()

class PurePursuitAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        pygame.init()
        self.clock = pygame.time.Clock()
    
    def act(self, state):
        """Compute the action for the red robot to chase the blue robot."""
        blue_heading = state[7]
        red_x, red_y = state[0], state[1]
        blue_x, blue_y = state[5], state[6]
        red_angle = state[4]
        distance_between_robots = state[9]
        striking_flag = state[10]

        front_x = math.cos(math.radians(blue_heading))
        front_y = -math.sin(math.radians(blue_heading)) 

        rel_x = red_x - blue_x
        rel_y = red_y - blue_y
        rel_mag = math.sqrt(rel_x**2 + rel_y**2)
        rel_x /= rel_mag
        rel_y /= rel_mag

        dot_product = front_x * rel_x + front_y * rel_y 
        
        if dot_product > 0.2 and distance_between_robots > 40: 
            angle_to_red = math.degrees(math.atan2(rel_x, rel_y))
            angle_to_red = (angle_to_red - 90) % 360
            blue_angle_diff = (angle_to_red - blue_heading + 180) % 360 - 180
            if(blue_angle_diff < 0):
                # print("targeting right")
                target_x = np.clip(blue_x - (distance_between_robots - 40) * front_y,0,800)
                target_y = np.clip(blue_y + (distance_between_robots - 40) * front_x,0,800)
            elif(blue_angle_diff > 0):
                # print("targeting left")
                target_x = np.clip(blue_x + (distance_between_robots - 40) * front_y,0,800)
                target_y = np.clip(blue_y - (distance_between_robots - 40) * front_x,0,800)
            delta_x = target_x - red_x
            delta_y = target_y - red_y
            # print(blue_x, blue_y, target_x, target_y)
            target_angle = math.degrees(math.atan2(-delta_y, delta_x))
            angle_difference = (red_angle - target_angle + 180) % 360 - 180
        else:
            rel_x = blue_x - red_x
            rel_y = blue_y - red_y
            angle_to_blue = math.degrees(math.atan2(rel_x, rel_y))
            angle_to_blue = (angle_to_blue - 90) % 360
            angle_difference = (red_angle - angle_to_blue + 180) % 360 - 180
        
        if abs(angle_difference) < 10 and distance_between_robots < 70:
            hammerFiring.start()
            
        forward_speed = (distance_between_robots/1600) + 0.4
        left_drive = forward_speed + 1.5 * angle_difference/180
        right_drive = forward_speed - 1.5 * angle_difference/180
        weapon_pwm_override = hammerFiring.update()
        if weapon_pwm_override is not None:
            weapon_drive_pwm = weapon_pwm_override
        else:
            weapon_drive_pwm = 1500  # Neutral value if not firing
        # print (distance_between_robots)
        left_drive_pwm = np.round(np.clip(1500 + left_drive * 500, 1000, 2000))
        right_drive_pwm = np.round(np.clip(1500 + right_drive * 500, 1000, 2000))
        
        send_pwm_data(int(left_drive_pwm), int(right_drive_pwm), int(weapon_drive_pwm))
        
        return np.array([left_drive, right_drive, 1.0 if hammerFiring.active else 0.0], dtype=np.float32)

    def simulate(self, state):
        """Simulate the action for the red robot to chase the blue robot."""
        blue_heading = state[7]
        red_x, red_y = state[0], state[1]
        blue_x, blue_y = state[5], state[6]
        red_angle = state[4]
        distance_between_robots = state[9]
        trajectory = []
        for _ in range(60*2):
            front_x = math.cos(math.radians(blue_heading))
            front_y = -math.sin(math.radians(blue_heading)) 

            rel_x = red_x - blue_x
            rel_y = red_y - blue_y
            rel_mag = math.sqrt(rel_x**2 + rel_y**2)
            rel_x /= rel_mag
            rel_y /= rel_mag

            dot_product = front_x * rel_x + front_y * rel_y 

            if dot_product > 0.2 and distance_between_robots > 40: 
                angle_to_red = math.degrees(math.atan2(rel_x, rel_y))
                angle_to_red = (angle_to_red - 90) % 360
                blue_angle_diff = (angle_to_red - blue_heading + 180) % 360 - 180
                if(blue_angle_diff < 0):
                    # print("targeting right")
                    target_x = np.clip(blue_x - (distance_between_robots - 40) * front_y,0,800)
                    target_y = np.clip(blue_y + (distance_between_robots - 40) * front_x,0,800)
                elif(blue_angle_diff > 0):
                    # print("targeting left")
                    target_x = np.clip(blue_x + (distance_between_robots - 40) * front_y,0,800)
                    target_y = np.clip(blue_y - (distance_between_robots - 40) * front_x,0,800)
                delta_x = target_x - red_x
                delta_y = target_y - red_y
                # print(blue_x, blue_y, target_x, target_y)
                target_angle = math.degrees(math.atan2(-delta_y, delta_x))
                angle_difference = (red_angle - target_angle + 180) % 360 - 180
            else:
                rel_x = blue_x - red_x
                rel_y = blue_y - red_y
                angle_to_blue = math.degrees(math.atan2(rel_x, rel_y))
                angle_to_blue = (angle_to_blue - 90) % 360
                angle_difference = (red_angle - angle_to_blue + 180) % 360 - 180

            forward_speed = (distance_between_robots/1600) + 0.4
            left_drive = forward_speed + 1.5 * angle_difference/180
            right_drive = forward_speed - 1.5 * angle_difference/180

            #recalculate the new state
            red_x = red_x + 2 * (left_drive + right_drive) * math.cos(math.radians(red_angle))
            red_y = red_y - 2 * (left_drive + right_drive) * math.sin(math.radians(red_angle))
            red_angle = (red_angle + 1/2 * (right_drive - left_drive) * 180/(np.pi)) % 360
            distance_between_robots = math.sqrt((red_x - blue_x)**2 + (red_y - blue_y)**2)
            trajectory.append([red_x, red_y])
        # print (trajectory)
        return np.array(trajectory, dtype=np.float32)


env = BattlebotsEnv()
state_size = env.observation_space.shape[0]
action_size = env.action_space.shape[0]
agent = PurePursuitAgent(state_size, action_size)

def run_pure_pursuit():
    while True:
        state = env.reset(randomize=True, cpu_Human=True)
        total_reward = 0
        step = 0
        env_times = []
        sim_times = []
        while True:
            start_time = time.perf_counter()
            env.render(path = agent.simulate(state))
            end_time = time.perf_counter()
            sim_times.append((end_time - start_time) * 1e3)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    env.close()
                    pygame.quit()
                    exit()
            start_time = time.perf_counter()
            action = agent.act(state)
            end_time = time.perf_counter()
            env_times.append((end_time - start_time) * 1e6) 
            next_state, reward, done, truncated, _ = env.step(action)
            total_reward += reward
            state = next_state
            step += 1

            time.sleep(0.01)

            if done or truncated:
                print(f"Pure Pursuit Agent - Score: {total_reward}, Steps: {step}, Maximum Calculation Time: {np.max(env_times):.2f} us, Average Simulation Time: {np.mean(sim_times):.2f} ms")
                break
            
    env.close()

if __name__ == "__main__":
    run_pure_pursuit()
    
