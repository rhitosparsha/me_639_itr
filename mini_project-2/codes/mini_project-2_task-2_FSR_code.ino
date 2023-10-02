int fsrPin = A0;
int fsrReading; // the analog reading from the FSR resistor divider

void setup(void) {
  Serial.begin(115200);
}

void loop(void) {
  fsrReading = analogRead(fsrPin);  // Read analog input from fsrPin
  Serial.print(fsrReading);     // print the raw analog reading
  Serial.println();
  delay(10);
}