/*
 * 04.09.2020
 * Der ESP32 ist in der Lage zuverlässig den Motor zu steuern und Messdaten zu lesen. 
 * Das Display zeigt die aktuelle Geschwindigkeit an. Außerdem gibt das Display eine Warnung bei Übertemperatur aus. 
 * Danach wurde ein Sicherheitsmechanismus implementiert, bei dem ein Schalter geschlossen bleiben muss, anderenfalls wird der Motor gebremst. 
 */

 /*
  * Pin Belegung:
  * Serial (VESC 1): RX = RX, TX = TX
  * Serial2 (VESC 2): RX = 16, TX = 17
  * I2C: SDA = 21, SCL = 22, GND = GND, VCC = 5V/3.3V
  * Totmannschalter = 18
  */

//Include libraries for VESC
#include <HardwareSerial.h>
#include <ESP8266VESC.h>

//Include libraries for display
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

//Define display size
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

//Define GPIO pin for button
#define BUTTON 18

//Global variables
boolean openswitch;
boolean vescunavailable;
boolean overheating1;
boolean overheating2;

//Create required class instances
ESP8266VESC esp8266VESC1 = ESP8266VESC(Serial2); //VESC 1
ESP8266VESC esp8266VESC2 = ESP8266VESC(Serial); //VESC 2
VESCValues vescValues1;
VESCValues vescValues2;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); //Display


//if any of the MOSFET temperatures is above 100, or the PCB temperature above 80 then return true
boolean overtemperature (VESCValues vescValues) {
  if (vescValues.temperatureMosfet1 > 100.0 || vescValues.temperatureMosfet2 > 100.0 || vescValues.temperatureMosfet3 > 100.0 || vescValues.temperatureMosfet4 > 100.0 || vescValues.temperatureMosfet5 > 100.0 || vescValues.temperatureMosfet6 > 100.0 || vescValues.temperaturePCB > 80.0) {
    return true;
  }
  else {
    return false;
  }
}


//read sensor values from both VESCs and return true if successful
boolean readVESC() {
  boolean vesc1 = esp8266VESC1.getVESCValues(vescValues1);
  boolean vesc2 = esp8266VESC2.getVESCValues(vescValues2);
  
  if ((vesc1 == false) || (vesc2 == false)) {
    return false;
  }
  else {
    return true;
  }
}

//Read all sensors function
void readSensors() {
  //Read safety switch 
  if(digitalRead(BUTTON)){
    openswitch = true;
  }
  else {
    openswitch = false;
  }

  //Read VESC sensors
  vescunavailable = !readVESC();

  //Check for overheating after reading VESC sensors
  overheating1 = overtemperature(vescValues1);
  overheating2 = overtemperature(vescValues2);
}


//Control motor function
void drive(int rpm) {
  if (overtemperature(vescValues1) || overtemperature(vescValues2) || vescunavailable) {
    esp8266VESC1.releaseEngine();
    esp8266VESC2.releaseEngine();
  }
  else {
    if (openswitch) {
      //Brake
      esp8266VESC1.fullBreaking();
      esp8266VESC2.fullBreaking();
    }
    else {
      esp8266VESC1.setRPM(rpm);
      esp8266VESC2.setRPM(rpm);
    }
  }
}


//Write on display function
void showOnDisplay () {
  //clear display from previous values
  display.clearDisplay();

  //set cursor to position to keep image stable
  display.setCursor(0, 0);

  //Show message depending on current variable states
  if (vescunavailable) {
    display.setTextSize(1);
    display.println("ERROR: Connection    failure to one or    both Motorcontrollers");
  }
  else {
    if (openswitch) {
      //Show message
      display.setTextSize(1);
      display.println("Please close security mechanism");    
    }
    else {
      if (overheating1 && overheating2) {
        display.setTextSize(2);
        display.println("WARNING: Both VESCs overheat! Driving blocked for safety");
      }
      else if (overheating1) {
        display.setTextSize(2);
        display.println("WARNING: VESC 1 overheats! Driving blocked for safety");
      }
      else if (overheating2) {
        display.setTextSize(2);
        display.println("WARNING: VESC 2 overheats! Driving blocked for safety");
      }
      else {
        //display average velocity on display
        //string rpm1 = string.toString(vescValues1.rpm);
        int velocity1 = (int)((vescValues1.rpm * 254) / 60000);
        int velocity2 = (int)((vescValues2.rpm * 254) / 60000);
        int velocity = (velocity1 + velocity2) / 2;
    
        display.setTextSize(2);
        display.print(vescValues1.rpm);
        display.print(" km/h");
        display.println();
      }    
    }
  }

  //show message on display
  display.display(); 
}


//Initialization
void setup() {
  //Setup serial connection to VESC 1
  Serial.begin(115200);
  delay(50);
  
  //Setup serial connection to VESC 1
  Serial2.begin(115200);
  delay(50);

  //Setup GPIO pin with button for interrupt
  pinMode(BUTTON, INPUT_PULLUP);
  //attachInterrupt(BUTTON, ISR, CHANGE);  

  //Initialize display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(50);
  display.clearDisplay();
  display.setTextColor(WHITE);

  //Show successful Bootup on display for 2s
  display.setTextSize(2);
  display.println("Bootup    successful");
  display.display();
  delay(2000);
  display.clearDisplay();
}


//Program routine
void loop() {  
  //Read all sensors
  readSensors();

  //Control motor
  drive(0);
  
  //Write on display
  showOnDisplay();
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
