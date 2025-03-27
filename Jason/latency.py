import serial
import time

# Open the serial port (update COM port accordingly) with a 2-second timeout for reading
ser = serial.Serial('COM7', 115200, timeout=2)

# Function to measure round-trip latency
def measure_latency():
    # Send timestamp to the Arduino
    timestamp = int(time.time() * 1000)  # Current timestamp in milliseconds
    ser.write(str(timestamp).encode())  # Send timestamp to Arduino

    # Record the time when the message is sent
    start_time = time.time()

    # Wait for the Arduino's response with a timeout protection
    response = ser.readline().decode().strip()  # Wait for the response with the timestamp

    if response:
        end_time = time.time()
        round_trip_time = (end_time - start_time) * 1000  # Convert to milliseconds
        print(f"Round-trip Latency: {(round_trip_time - 1000):.2f} ms")
    else:
        print("No response received. Timeout occurred.")

# Continuously measure latency
while True:
    measure_latency()
    time.sleep(1)  # Wait for 1 second before measuring again
