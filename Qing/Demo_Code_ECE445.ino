/*
    ECE 445 group 43 project
    breadboard demo code based on the Arduino ESP32 BLE Sever example
    Reference:
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
    MPU 6050 connection:
    https://randomnerdtutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
// GPIO setting
// battery is connected to GPIO 11
const int batPin = 17;
const int SDA_pin = 19;
const int SCL_pin = 20;
// IMU MPU6050
Adafruit_MPU6050 mpu;
// value to store the battery value
const float Vref = 3.3;  // Reference voltage (default 3.3V)
int batRaw = 0;  // Raw ADC value
float batVoltage = 0.0;  // Converted voltage value
BLECharacteristic *pCharacteristic;
void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

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


  BLEDevice::init("ECE 445 Group 43");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic =
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);


  pCharacteristic->setValue("Hello!!");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  unsigned long startTime = 0;
  unsigned long lastPrintTime = 0;
  size_t totalBytesSent = 0;
  unsigned long currentTime = millis();
  if (startTime == 0) startTime = currentTime;

  // put your main code here, to run repeatedly:
  batRaw = analogRead(batPin);
  batVoltage = (batRaw / 4095.0) * Vref;
  String BLE_msg = String(batVoltage,2) + "V";  // Convert int to string
  Serial.println("Battery Level: " + BLE_msg);

  // IMU reading
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  BLE_msg = "t=" + String(currentTime) +
                   " | Bat=" + String(batVoltage, 2) + "V" +
                   " | Accel: X=" + String(a.acceleration.x, 2) +
                   ",Y=" + String(a.acceleration.y, 2) +
                   ",Z=" + String(a.acceleration.z, 2) +
                   " | Gyro: X=" + String(g.gyro.x, 2) +
                   ",Y=" + String(g.gyro.y, 2) +
                   ",Z=" + String(g.gyro.z, 2) +
                   " | Temp=" + String(temp.temperature, 2) + " C";
  pCharacteristic->setValue(BLE_msg.c_str());  // Update BLE characteristic
  pCharacteristic->notify();  // Send updated value

  
  /* Print out the values */
  Serial.println(BLE_msg);

  delay(200);
}
