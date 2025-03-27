import serial
import time

# Open the serial connection (adjust the COM port as needed)
ser = serial.Serial('COM7', 115200)

# Function to send PWM values (left, right, weapon)
def send_pwm_data(left, right, weapon):
    message = f"{left} {right} {weapon}\n"
    print(f"Sending PWM - Left: {left}, Right: {right}, Weapon: {weapon}")
    ser.write(message.encode())  # Send data via serial
    ser.flush() 

try:
    left, right, weapon = [1500, 1500, 1500]
    while True:
        # left_pwm = 1500  # Example PWM for left motor
        # right_pwm = 1500  # Example PWM for right motor
        # weapon_pwm = 1500  # Example PWM for weapon motor
        left = 1000 if left >= 2000 else left + 1
        right = 1000 if right >= 2000 else right + 1
        weapon = 1000 if weapon >= 2000 else weapon + 1
        send_pwm_data(left, right, weapon)
        time.sleep(0.016)  # ~60Hz update rate (16ms delay)

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
