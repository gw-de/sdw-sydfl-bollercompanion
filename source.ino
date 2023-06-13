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
ESP8266VESC esp8266VESC2 = ESP8266VESC(Serial); //VESC 2

void setup() {
  //Setup serial connection to VESC 1
  Serial.begin(115200);
  delay(500);
  
  //Setup serial connection to VESC 1
  Serial2.begin(115200);
  delay(500);
}


void loop() {
  //Write to VESC
  esp8266VESC1.setRPM(5000);
  esp8266VESC2.setRPM(5000);
  delay(2000);

  esp8266VESC1.releaseEngine();
  esp8266VESC2.releaseEngine();
  delay(2000);

  esp8266VESC1.setCurrent(5.0);
  esp8266VESC2.setCurrent(5.0);
  delay(2000);

  esp8266VESC1.fullBreaking();
  esp8266VESC2.fullBreaking();
  delay(2000);

  //Read from VESC
  VESCValues vescValues1;
  VESCValues vescValues2;
  
  if (esp8266VESC1.getVESCValues(vescValues1) == true)
  {
    /*vescValues1.temperatureMosfet1
    vescValues1.temperatureMosfet2
    vescValues1.temperatureMosfet3
    vescValues1.temperatureMosfet4
    vescValues1.temperatureMosfet5
    vescValues1.temperatureMosfet6
    vescValues1.temperaturePCB

    vescValues1.avgMotorCurrent
    vescValues1.avgInputCurrent
    vescValues1.dutyCycleNow
        
    vescValues1.rpm
    vescValues1.inputVoltage
        
    vescValues1.ampHours
    vescValues1.ampHoursCharged

    vescValues1.wattHours
    vescValues1.wattHoursCharged
    
    vescValues1.tachometer
    vescValues1.tachometerAbs
    */
  }
  else
  {
    //unable to read...
  }

  if (esp8266VESC2.getVESCValues(vescValues2) == true)
  {
    /*vescValues2.temperatureMosfet1
    vescValues2.temperatureMosfet2
    vescValues2.temperatureMosfet3
    vescValues2.temperatureMosfet4
    vescValues2.temperatureMosfet5
    vescValues2.temperatureMosfet6
    vescValues2.temperaturePCB

    vescValues2.avgMotorCurrent
    vescValues2.avgInputCurrent
    vescValues2.dutyCycleNow
        
    vescValues2.rpm
    vescValues2.inputVoltage
        
    vescValues2.ampHours
    vescValues2.ampHoursCharged

    vescValues2.wattHours
    vescValues2.wattHoursCharged
    
    vescValues2.tachometer
    vescValues2.tachometerAbs
    */
  }
  else
  {
    //unable to read...
  }
  
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
