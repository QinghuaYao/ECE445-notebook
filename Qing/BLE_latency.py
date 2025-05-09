import asyncio
from bleak import BleakClient, BleakScanner
import time
import re

# Set target device name, service UUID, and characteristic UUID
DEVICE_NAME = "ECE 445 Group 43"
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# Regular expression to extract the timestamp from the message (format like "t=123456")
timestamp_regex = re.compile(r"t=(\d+)")

# Notification callback function
def notification_handler(sender, data):
    msg = data.decode("utf-8")
    print("\nReceived BLE message:", msg)
    
    # Try to extract timestamp from the message
    match = timestamp_regex.search(msg)
    if match:
        esp_time = int(match.group(1))  # Timestamp sent by ESP32 (milliseconds)
        pc_time = int(time.time() * 1000)  # Current PC time (milliseconds)
        latency = pc_time - esp_time
        print("Bluetooth latency:", latency, "ms")
    else:
        print("Warning: Timestamp not found in the message.")

# Main function: scan for device, connect, and subscribe to notifications
async def main():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    target = None

    for d in devices:
        if d.name is not None and DEVICE_NAME in d.name:
            target = d
            break

    if not target:
        print(f"No device found with name containing '{DEVICE_NAME}'")
        return

    print(f"Found device: {target.name}, Address: {target.address}, Connecting...")
    async with BleakClient(target.address) as client:
        print("Connected successfully, subscribing to notifications...")
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        
        # Keep running to receive notifications
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("Exiting program...")

if __name__ == "__main__":
    asyncio.run(main())

