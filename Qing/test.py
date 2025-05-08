import websocket
import time
import threading
bytes_received = 0
start_time = time.time()

def on_message(ws, message):
    global bytes_received, start_time
    # print(message)
    # count bytes
    bytes_received += len(message)

    # monitor speed every second
    elapsed = time.time() - start_time
    if elapsed >= 1.0:
        speed_kbps = (bytes_received * 8) / 1000  # converts to kbps
        print(f"speed: {speed_kbps:.2f} kbps")
        bytes_received = 0
        start_time = time.time()

def on_open(ws):
    print("connected to ESP32!!")

ws = websocket.WebSocketApp("ws://172.20.10.7:81",
                            on_message=on_message,
                            on_open=on_open)
def run_ws():
    ws.run_forever()

ws_thread = threading.Thread(target=run_ws)
ws_thread.start()

time.sleep(5)

ws.close()

ws_thread.join()
