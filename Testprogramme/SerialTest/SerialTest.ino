#include <HardwareSerial.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // set the data rate for the HardwareSerial port
  Serial2.begin(115200);
}

void loop() {
  if (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  if (Serial.available()) {
    Serial2.write(Serial.read());
  }
}
