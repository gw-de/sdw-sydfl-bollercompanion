#include <ESP8266VESC.h>
#include <HardwareSerial.h>

/*
 * 27.08.2020
 * Der ESP32 ist in der Lage zuverlässig den Motor zu steuern und Messdaten zu lesen. 
 * 
 * Nächste Schritte:
 * - Anbindung an App
 * - Implementierung von Steuerungsfunktionen wie Lenken, ...
 */
 
//Instanz "Serial": Rx = GPIO3, Tx = GPIO1, aber doppelbelegt mit USB!
//Instanz "Serial2": Rx = GPIO16, Tx = GPIO17
//Instanz "Serial1" nicht benutzen, doppelbelegt mit Flash-Speicher!

//Define global variables
ESP8266VESC esp8266VESC1 = ESP8266VESC(Serial2); //VESC 1

void setup() {
  //Debug output
  Serial.begin(115200);
  delay(500);
  
  //Setup serial connection to VESC 1
  Serial2.begin(115200);
  delay(500);
}


void loop() {
  //Write to VESC
  Serial.println("Setting to 2000 RPM");
  esp8266VESC1.setRPM(2000);
  delay(2000);

  Serial.println("Release Engine");
  esp8266VESC1.releaseEngine();
  delay(2000);

  Serial.println("Setting Current to 1 A");
  esp8266VESC1.setCurrent(1.0);
  delay(2000);

  Serial.println("Brake!");
  esp8266VESC1.fullBreaking();
  delay(2000);

  
  //Read from VESC
  VESCValues vescValues;
  
  if (esp8266VESC1.getVESCValues(vescValues) == true)
  {
    Serial.println("Temperature Mosfet #1 = " + String(vescValues.temperatureMosfet1) + " degree");
    Serial.println("Temperature Mosfet #2 = " + String(vescValues.temperatureMosfet2) + " degree");
    Serial.println("Temperature Mosfet #3 = " + String(vescValues.temperatureMosfet3) + " degree");
    Serial.println("Temperature Mosfet #4 = " + String(vescValues.temperatureMosfet4) + " degree");
    Serial.println("Temperature Mosfet #5 = " + String(vescValues.temperatureMosfet5) + " degree");
    Serial.println("Temperature Mosfet #6 = " + String(vescValues.temperatureMosfet6) + " degree");
    Serial.println("Temperature PCB = " + String(vescValues.temperaturePCB) + " degree");

    Serial.println("Average motor current = " + String(vescValues.avgMotorCurrent) + "A");
    Serial.println("Average battery current = " + String(vescValues.avgInputCurrent) + "A");
    Serial.println("Duty cycle = " + String(vescValues.dutyCycleNow) + "%");
        
    Serial.println("rpm = " + String(vescValues.rpm) + "rpm");
    Serial.println("Battery voltage = " + String(vescValues.inputVoltage) + "V");
        
    Serial.println("Drawn energy (mAh) = " + String(vescValues.ampHours) + "mAh");
    Serial.println("Charged energy (mAh) = " + String(vescValues.ampHoursCharged) + "mAh");

    Serial.println("Drawn energy (Wh) = " + String(vescValues.wattHours) + "Wh");
    Serial.println("Charged energy (Wh) = " + String(vescValues.wattHoursCharged) + "Wh");
    
    Serial.println("tachometer = " + String(vescValues.tachometer));
    Serial.println("tachometerAbs = " + String(vescValues.tachometerAbs));
  }
  else
  {
    Serial.println("The VESC values could not be read!");
  }
  
  Serial.println();
  delay(1000);
}

/*
 * VESC library methods:
 * esp8266VESC.setDutyCycle(const float dutyValue);
 * esp8266VESC.setCurrent(const float currentInAmpere);
 * esp8266VESC.setCurrentBrake(const float currentInAmpere);
 * esp8266VESC.setRPM(const int32_t rpmValue);
 * esp8266VESC.releaseEngine();
 * esp8266VESC.fullBreaking();
 * esp8266VESC.getVESCValues(VESCValues &vescValues);
 * esp8266VESC.debugPrintArray(uint8_t data[], int length);
 * 
 * VESCValues variables:
 * float temperatureMosfet1 = 0.0f;
 * float temperatureMosfet2 = 0.0f;
 * float temperatureMosfet3 = 0.0f;
 * float temperatureMosfet4 = 0.0f;
 * float temperatureMosfet5 = 0.0f;
 * float temperatureMosfet6 = 0.0f;
 * float temperaturePCB = 0.0f;
 * float avgMotorCurrent = 0.0f;
 * float avgInputCurrent = 0.0f;
 * float dutyCycleNow = 0.0f;
 * int32_t rpm = 0;
 * float inputVoltage = 0.0f;
 * float ampHours = 0.0f;
 * float ampHoursCharged = 0.0f;
 * float wattHours = 0.0f;
 * float wattHoursCharged = 0.0f;
 * int32_t tachometer = 0;
 * int32_t tachometerAbs = 0;
 */
