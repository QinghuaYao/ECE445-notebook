import websocket
import time
import threading

# Global variables to accumulate latency values and track the 5-second window start
latencies = []
window_start = time.time()

def on_message(ws, message):
    global latencies, window_start
    # Expect message format: "ping:<timestamp>"
    if message.startswith("ping:"):
        try:
            sent_timestamp = float(message.split(":", 1)[1])
            rtt = (time.time() - sent_timestamp) * 1000  # Convert to milliseconds
            latencies.append(rtt)
            
            # If 5 seconds have passed, calculate and print the average latency
            if time.time() - window_start >= 5.0:
                if latencies:
                    avg_latency = sum(latencies) / len(latencies)
                    print(f"Average latency over last 5 s: {avg_latency:.2f} ms")
                # Reset for the next 5-second window
                latencies = []
                window_start = time.time()
        except Exception:
            pass  # Ignore any parsing errors

def on_open(ws):
    def run():
        while True:
            # Send ping message with the current timestamp
            ping_msg = f"ping:{time.time()}"
            ws.send(ping_msg)
            time.sleep(1)  # Send a ping every 1 second
    threading.Thread(target=run, daemon=True).start()

# Replace with your ESP32's IP address and port if necessary.
ws_url = "ws://172.20.10.7:81"
ws = websocket.WebSocketApp(ws_url, on_open=on_open, on_message=on_message)
ws.run_forever()
