#include <WiFi.h>
#include <esp_now.h>

// Data structure to send PWM values
typedef struct {
  int left;
  int right;
  int weapon;
} pwm_data_t;

pwm_data_t pwmData;

// Replace with your Robot ESP32S3 MAC address - d8:3b:da:46:d6:84
uint8_t robotMAC[] = {0xD8, 0x3B, 0xDA, 0x46, 0xD6, 0x84};

// Callback when data is sent via ESP-NOW
// void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
// }

// Function to read PWM values from the serial port
void readSerialData() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');  // Read the line from the serial
    int left, right, weapon;
    if (sscanf(line.c_str(), "%d %d %d", &left, &right, &weapon) == 3) {
      pwmData.left = left;
      pwmData.right = right;
      pwmData.weapon = weapon;
    }

    // char buffer[20];
    // line.toCharArray(buffer, 20);
    // esp_now_send(robotMAC, (uint8_t *)buffer, sizeof(pwm_data_t));
  }
}

void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  Serial.write(data, len);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  // ESP-NOW works in STA mode

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Add peer (robot) ESP32S3
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, robotMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  readSerialData();  // Read data from serial
  // Send PWM data via ESP-NOW
  esp_now_send(robotMAC, (uint8_t *)&pwmData, sizeof(pwm_data_t));
  
  delay(16);  // ~60Hz update rate (16ms delay)
}
