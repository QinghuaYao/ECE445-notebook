// battery is connected to GPIO 10
const int batPin = 10;
// value to store the battery value
int batVal = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  // reading battery value
  batVal = analogRead(batPin);
  Serial.print("Analog value:");
  Serial.println(batVal);
  delay(500);
}
