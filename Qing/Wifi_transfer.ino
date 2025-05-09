#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// GPIO setting
// battery is connected to GPIO 11
const int batPin = 11;
const int SDA_pin = 12;
const int SCL_pin = 13;
// IMU MPU6050
Adafruit_MPU6050 mpu;
// value to store the battery value
const float Vref = 3.3;  // Reference voltage (default 3.3V)
int batRaw = 0;  // Raw ADC value
float batVoltage = 0.0;  // Converted voltage value
// Wi-Fi SSID and Password
const char* ssid = "Qing";
const char* password = "12345678";

//  WebSocket server running on port 81
WebSocketsServer webSocket = WebSocketsServer(81);

void setup() {
  Serial.begin(115200);
  // IMU setup
  Wire.begin(12,13);
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  // connect Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("WiFi connected, IP address: ");
  Serial.println(WiFi.localIP());

  // running webSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server is running, port: 81");
}

// Handle received WebSocket messages
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_TEXT:
      {
        String msg = String((char*)payload);
        Serial.printf("Received message from client: %s\n", payload);
        // If message starts with "ping", echo it back for latency measurement.
        if (msg.startsWith("ping")) {
          webSocket.sendTXT(num, msg);
        } else {
          // Otherwise, handle normally.
          webSocket.sendTXT(num, "ESP32 receive: " + msg);
        }
      }
      break;
    default:
      break;
  }
}

void loop() {
  webSocket.loop();
  // read MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // String data = "Accel: X=" + String(a.acceleration.x) +
  //               ",Y=" + String(a.acceleration.y) +
  //               ",Z=" + String(a.acceleration.z) +
  //               " | Gyro: X=" + String(g.gyro.x) +
  //               ",Y=" + String(g.gyro.y) +
  //               ",Z=" + String(g.gyro.z) +
  //               " | Temp: " + String(temp.temperature) + " C";
  String data = "A1B2C3";
  // send data
  webSocket.broadcastTXT(data);

  //Serial.println(data);
  delay(5);
}
