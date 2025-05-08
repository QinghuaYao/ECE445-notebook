import asyncio
from bleak import BleakClient, BleakScanner
import time
import re

# 设置目标设备名称、服务 UUID 和特征 UUID
DEVICE_NAME = "ECE 445 Group 43"
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# 正则表达式用于提取消息中的时间戳（格式例如 "t=123456"）
timestamp_regex = re.compile(r"t=(\d+)")

# 通知回调函数
def notification_handler(sender, data):
    msg = data.decode("utf-8")
    print("\n接收到BLE消息：", msg)
    
    # 尝试从消息中提取时间戳
    match = timestamp_regex.search(msg)
    if match:
        esp_time = int(match.group(1))  # ESP32 发送的时间戳（毫秒）
        pc_time = int(time.time() * 1000)  # 当前电脑时间（毫秒）
        latency = pc_time - esp_time
        print("蓝牙延迟：", latency, "毫秒")
    else:
        print("警告：消息中未找到时间戳。")

# 主程序：扫描设备、连接并订阅通知
async def main():
    print("正在扫描 BLE 设备...")
    devices = await BleakScanner.discover()
    target = None

    for d in devices:
        if d.name is not None and DEVICE_NAME in d.name:
            target = d
            break

    if not target:
        print(f"未找到名称包含 '{DEVICE_NAME}' 的设备")
        return

    print(f"找到设备：{target.name}，地址：{target.address}，正在连接...")
    async with BleakClient(target.address) as client:
        print("连接成功，正在订阅通知...")
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        
        # 保持运行状态以接收通知
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("退出程序...")

if __name__ == "__main__":
    asyncio.run(main())
