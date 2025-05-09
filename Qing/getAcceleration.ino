/**
 * @file getAcceleration.ino
 * @brief Read acceleration on XYZ from LIS331HH over I2C on ESP32-S3, output in m/s^2
 */

#include <Wire.h>
#include <DFRobot_LIS.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Pin definitions for ESP32-S3
#define SDA_PIN    11     // IO11
#define SCL_PIN    12     // IO12

// I2C address of LIS331HH: 0x18 if SA0→GND, 0x19 if SA0→VCC
#define LIS331HH_I2C_ADDR  0x18

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;
// Conversion: 1 mg = 0.001 g, and 1 g = 9.80665 m/s²
static const float MG_TO_MS2 = 9.80665f * 0.001f;

// Create I2C object
DFRobot_LIS331HH_I2C acce(&Wire, LIS331HH_I2C_ADDR);

// battery is connected to GPIO 10
const int batPin = 9;
// value to store the battery value
const float Vref = 3.3;  // Reference voltage (default 3.3V)
int batRaw = 0;  // Raw ADC value
float batVoltage = 0.0;  // Converted voltage value
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  Serial.print("Initializing LIS331HH...");
  while (!acce.begin()) {
    Serial.println(" failed. Check wiring & address!");
    delay(1000);
  }
  Serial.println(" OK");

  Serial.print("LIS331HH chip ID: 0x");
  Serial.println(acce.getID(), HEX);

  acce.setRange(DFRobot_LIS::eLis331hh_6g);
  acce.setAcquireRate(DFRobot_LIS::eNormal_50HZ);

  Serial.println("Acceleration reads in m/s^2:");


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
  // Read raw acceleration (mg)
  long ax_mg = acce.readAccX();
  long ay_mg = acce.readAccY();
  long az_mg = acce.readAccZ();

  // Convert to m/s^2
  float ax = ax_mg * MG_TO_MS2/3;
  float ay = ay_mg * MG_TO_MS2/3;
  float az = az_mg * MG_TO_MS2/3;

  // Print with 3 decimal places
  Serial.print("X: "); Serial.print(ax, 3); Serial.print(" m/s²\t");
  Serial.print("Y: "); Serial.print(ay, 3); Serial.print(" m/s²\t");
  Serial.print("Z: "); Serial.print(az, 3); Serial.println(" m/s²");
  batRaw = analogRead(batPin);
  batVoltage = (batRaw / 4095.0) * Vref;
  String Batmsg = String(batVoltage,2) + "V";
  Serial.print("Analog value:" + Batmsg);


  pCharacteristic->setValue(Batmsg.c_str());  // Update BLE characteristic
  pCharacteristic->notify();  // Send updated value
  delay(300);
}
