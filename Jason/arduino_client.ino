#include <WiFi.h>
#include <esp_now.h>
#include "driver/mcpwm.h"
#include "esp_wifi.h"

// Define motor PWM output pins (adjust these as needed)
#define LEFT_DRIVE_PIN 1
#define RIGHT_DRIVE_PIN 2
#define WEAPON_DRIVE_PIN 3

// Define pins to read PWM signals
#define LEFT_PWM_PIN 7
#define RIGHT_PWM_PIN 8
#define WEAPON_PWM_PIN 9
#define SENSOR_PIN_1 43
#define SENSOR_PIN_2 44

// Set desired PWM frequency (Hz)
#define PWM_FREQ 60    // 60Hz (period ≈ 16667 µs)
#define SIGNAL_TIMEOUT 100000 

// Server MAC address
uint8_t serverMAC[] = {0xD8, 0x3B, 0xDA, 0xA4, 0x4B, 0xE4};

// PWM data structure
typedef struct {
  int left;
  int right;
  int weapon;
} pwm_data_t;

pwm_data_t pwmData;

// Non-blocking PWM measurement variables
unsigned long leftStart = 0, leftPulse = 0;
unsigned long rightStart = 0, rightPulse = 0;
unsigned long weaponStart = 0, weaponPulse = 0;
unsigned long modeSelectStart = 0, modeSelectPulse = 0;
unsigned long weaponSafetyStart = 0, weaponSafetyPulse = 0;

namespace robotStatus {
  enum modeSelect {
    MANUAL_CONTROL = -1,
    ROBOT_OFF = 0,
    AUTONOMOUS_CONTROL = 1
  };
  modeSelect currentMode = ROBOT_OFF;
  bool weaponActivated = false;
}

// Timing variable for PWM reading
unsigned long lastReadTime = 0;
#define READ_INTERVAL 5 // Read every 5 ms (adjust as necessary)

// Helper: convert pulse width (µs) to duty percentage
float pulseToDutyPercent(int pulse_us) {
  float period_us = 1000000.0 / PWM_FREQ; // ~16667 µs for 60Hz
  return ((float)pulse_us / period_us) * 100.0;
}

// Function to initialize MCPWM
void setupMCPWM() {
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LEFT_DRIVE_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, RIGHT_DRIVE_PIN);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, WEAPON_DRIVE_PIN);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = PWM_FREQ;
  pwm_config.cmpr_a = 0;
  pwm_config.cmpr_b = 0;
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
}

void setLeftPWM(int pulse_us) {
  float duty = 0;  
  switch (robotStatus::currentMode) {
    case robotStatus::AUTONOMOUS_CONTROL: // autonomous control
      duty = pulseToDutyPercent(pulse_us);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
      break;
    case robotStatus::MANUAL_CONTROL: // manual passthrough
      duty = pulseToDutyPercent(leftPulse);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
      break;
    default: // ROBOT_OFF
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
      break;
  }
}

void setRightPWM(int pulse_us) {
  float duty = 0;  
  switch (robotStatus::currentMode) {
    case robotStatus::AUTONOMOUS_CONTROL: // autonomous control
      duty = pulseToDutyPercent(pulse_us);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty);
      break;
    case robotStatus::MANUAL_CONTROL: // manual passthrough
      duty = pulseToDutyPercent(rightPulse);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty);
      break;
    default: // ROBOT_OFF
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
      break;
  }
}

void setWeaponPWM(int pulse_us) {
  float duty = 0;  
  if (robotStatus::weaponActivated == true) {
    switch (robotStatus::currentMode) {
      case robotStatus::AUTONOMOUS_CONTROL: // autonomous control
        duty = pulseToDutyPercent(pulse_us);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
        break;
      case robotStatus::MANUAL_CONTROL: // manual passthrough
        duty = pulseToDutyPercent(weaponPulse);
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
        break;
      default: // ROBOT_OFF
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
        break;
    }
  } else {
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
  }
}

hw_timer_t *timer = NULL;
bool ledState = true;
void ARDUINO_ISR_ATTR checkDeadSignals() {
  unsigned long currentTime = micros();
  
  if (robotStatus::currentMode == robotStatus::MANUAL_CONTROL) ledState = !ledState;
  if (currentTime - leftStart > SIGNAL_TIMEOUT) leftPulse = 0;
  if (currentTime - rightStart > SIGNAL_TIMEOUT) rightPulse = 0;
  if (currentTime - weaponStart > SIGNAL_TIMEOUT) weaponPulse = 0;
  if (currentTime - modeSelectStart > SIGNAL_TIMEOUT) modeSelectPulse = 0;
  if (currentTime - weaponSafetyStart > SIGNAL_TIMEOUT) weaponSafetyPulse = 0;
  digitalWrite(LED_BUILTIN, ledState);
}

void IRAM_ATTR leftISR() {
  if (digitalRead(LEFT_PWM_PIN) == HIGH) {
    leftStart = micros();  // Record rising edge time
  } else {
    leftPulse = micros() - leftStart;  // Compute pulse width on falling edge
  }
}

void IRAM_ATTR rightISR() {
  if (digitalRead(RIGHT_PWM_PIN) == HIGH) {
    rightStart = micros();
  } else {
    rightPulse = micros() - rightStart;
  }
}

void IRAM_ATTR weaponISR() {
  if (digitalRead(WEAPON_PWM_PIN) == HIGH) {
    weaponStart = micros();
  } else {
    weaponPulse = micros() - weaponStart;
  }
}

void IRAM_ATTR modeSelectISR() {
  if (digitalRead(SENSOR_PIN_1) == HIGH) {
    modeSelectStart = micros();
  } else {
    modeSelectPulse = micros() - modeSelectStart;
  }
}

void IRAM_ATTR weaponSafetyISR() {
  if (digitalRead(SENSOR_PIN_2) == HIGH) {
    weaponSafetyStart = micros();
  } else {
    weaponSafetyPulse = micros() - weaponSafetyStart;
  }
}

// ESP-NOW data receive callback
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(pwm_data_t)) {
    memcpy(&pwmData, data, sizeof(pwm_data_t));

    // Update PWM outputs
    setLeftPWM(pwmData.left);
    setRightPWM(pwmData.right);
    setWeaponPWM(pwmData.weapon);
  }
}

// Task to handle WiFi and ESP-NOW on Core 0
void wifiTask(void *pvParameters) {
  WiFi.mode(WIFI_STA);  // ESP-NOW works in STA mode
  WiFi.setSleep(false);  // Disable power-saving mode
  esp_wifi_set_ps(WIFI_PS_NONE);  // Fully disable power-saving
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20); // Use 20MHz channel width
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    vTaskDelete(NULL);  // Terminate the task if initialization fails
  }

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, serverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    vTaskDelete(NULL);
  }

  while (true) {
    vTaskDelay(1);  // Yield task to allow other tasks to run
  }
}

void setup() {
  Serial.begin(115200);
  setupMCPWM();

  pinMode(LEFT_PWM_PIN, INPUT_PULLDOWN);
  pinMode(RIGHT_PWM_PIN, INPUT_PULLDOWN);
  pinMode(WEAPON_PWM_PIN, INPUT_PULLDOWN);
  pinMode(SENSOR_PIN_1, INPUT_PULLDOWN);
  pinMode(SENSOR_PIN_2, INPUT_PULLDOWN);
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize pins for reading PWM
  attachInterrupt(LEFT_PWM_PIN, leftISR, CHANGE);
  attachInterrupt(RIGHT_PWM_PIN, rightISR, CHANGE);
  attachInterrupt(WEAPON_PWM_PIN, weaponISR, CHANGE);
  attachInterrupt(SENSOR_PIN_1, modeSelectISR, CHANGE);
  attachInterrupt(SENSOR_PIN_2, weaponSafetyISR, CHANGE);

  // Create a task for handling WiFi and ESP-NOW on core 0
  xTaskCreatePinnedToCore(
    wifiTask,    // Task function
    "wifiTask",  // Name of task
    4096,        // Stack size
    NULL,        // Task input parameter
    1,           // Priority
    NULL,        // Task handle
    0            // Core 0
  );

  timer = timerBegin(20000);
  timerAttachInterrupt(timer, &checkDeadSignals);
  timerAlarm(timer, 1000, true, 0);
}

bool withinRange(int pulse, int low, int high){ // include low, exclude high
  if (pulse < high && pulse >= low) return true;
  else return false;
}

void loop() {
  if(withinRange(modeSelectPulse, 1250, 1750)){ // turn OFF 
    robotStatus::currentMode = robotStatus::ROBOT_OFF;
    digitalWrite(LED_BUILTIN, true);
  } else if (withinRange(modeSelectPulse, 1750, 2100)){ // turn on autonomous control
    robotStatus::currentMode = robotStatus::AUTONOMOUS_CONTROL;
    digitalWrite(LED_BUILTIN, false);
  } else if (withinRange(modeSelectPulse, 900, 1250)){ //turn on manual control
    robotStatus::currentMode = robotStatus::MANUAL_CONTROL;
  }
  else {
    robotStatus::currentMode = robotStatus::ROBOT_OFF; //invalid input / signal lost, turn off robot
    robotStatus::weaponActivated = false;
    digitalWrite(LED_BUILTIN, true);
  }

  if(withinRange(weaponSafetyPulse, 900, 1800)) robotStatus::weaponActivated = false; //turn off hammer
  else if (withinRange(weaponSafetyPulse, 1800, 2100)) robotStatus:: weaponActivated = true; //turn on hammer
  else {
    robotStatus::currentMode = robotStatus::ROBOT_OFF; //invalid input / signal lost, turn off robot
    robotStatus::weaponActivated = false;
  }
  delay(10);  // Small delay to allow for updates
}
