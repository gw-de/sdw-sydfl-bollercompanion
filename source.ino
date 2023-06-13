/*
 * 17.09.2020
 * Der ESP32 ist in der Lage zuverlässig den Motor zu steuern und Messdaten zu lesen. 
 * Das Display zeigt die aktuelle Geschwindigkeit an. Außerdem gibt das Display Meldungen bei Fehlern aus. 
 * Es ist ein Sicherheitsmechanismus implementiert, bei dem ein Schalter geschlossen bleiben muss, ansonsten wird der Motor gebremst. 
 * Mit dem Throttle kann die Geschwindigkeit gesteuert werden. 
 * Wird der Throttle nicht genutzt wird die Geschwindigkeit über den Winkel der Achse gesteuert. 
 */

 /*
  * Pin Belegung:
  * Serial (VESC 1): RX = RX, TX = TX
  * Serial2 (VESC 2): RX = 16, TX = 17
  * I2C: SDA = 21, SCL = 22, GND = GND, VCC = 5V/3.3V
  * Safety Switch = 18
  * Throttle: red = 3.3V, black = GND, white = 4
  * Gyro ID configuration = 5
  */

/*
 * TO-DO:
 * Werte von den Gyros werden gelesen, es muss noch Winkel und die weitere Berechnung programmiert werden. 
 */

/*
 * Include Libraries
 */
//Enable UART and I2C communication
#include <HardwareSerial.h>
#include <Wire.h>

//Include library for VESC
#include <ESP8266VESC.h>

//Include libraries for display
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

//Include library for gyro
#include <MPU6050.h>

/*
 * Define constants and global variables
 */
//Define display size
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

//Define GPIO pins
#define BUTTON 18
#define THROTTLE 4
#define GYRO 5

//Define global variables
boolean openswitch;
boolean vescunavailable;
boolean overheating1;
boolean overheating2;
int32_t throttleValue;
int32_t driveRPM;

int16_t a1x, a1y, a1z;
int16_t g1x, g1y, g1z;
int16_t a2x, a2y, a2z;
int16_t g2x, g2y, g2z;

/*
 * Create required class instances
 */
ESP8266VESC esp8266VESC1 = ESP8266VESC(Serial2); //VESC 1
ESP8266VESC esp8266VESC2 = ESP8266VESC(Serial); //VESC 2
VESCValues vescValues1;
VESCValues vescValues2;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); //Display
MPU6050 mpu1;
MPU6050 mpu2(0x69);



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
  /*
  boolean vesc1 = esp8266VESC1.getVESCValues(vescValues1);
  boolean vesc2 = esp8266VESC2.getVESCValues(vescValues2);
  
  if ((vesc1 == false) || (vesc2 == false)) {
    return false;
  }
  else {
    return true;
  }
  */

  //Simulation
  vescValues1.temperatureMosfet1 = 0.0;
  vescValues1.temperatureMosfet2 = 0.0;
  vescValues1.temperatureMosfet3 = 0.0;
  vescValues1.temperatureMosfet4 = 0.0;
  vescValues1.temperatureMosfet5 = 0.0;
  vescValues1.temperatureMosfet6 = 0.0;
  vescValues1.temperaturePCB = 0.0;
  vescValues1.avgMotorCurrent = 0.0;
  vescValues1.avgInputCurrent = 0.0;
  vescValues1.dutyCycleNow = 0.0;
  //vescValues1.rpm = 0;
  vescValues1.inputVoltage = 0.0;
  vescValues1.ampHours = 0.0;
  vescValues1.ampHoursCharged = 0.0;
  vescValues1.wattHours = 0.0;
  vescValues1.wattHoursCharged = 0.0;
  vescValues1.tachometer = 0;
  vescValues1.tachometerAbs = 0;

  vescValues2.temperatureMosfet1 = 0.0;
  vescValues2.temperatureMosfet2 = 0.0;
  vescValues2.temperatureMosfet3 = 0.0;
  vescValues2.temperatureMosfet4 = 0.0;
  vescValues2.temperatureMosfet5 = 0.0;
  vescValues2.temperatureMosfet6 = 0.0;
  vescValues2.temperaturePCB = 0.0;
  vescValues2.avgMotorCurrent = 0.0;
  vescValues2.avgInputCurrent = 0.0;
  vescValues2.dutyCycleNow = 0.0;
  //vescValues2.rpm = 0;
  vescValues2.inputVoltage = 0.0;
  vescValues2.ampHours = 0.0;
  vescValues2.ampHoursCharged = 0.0;
  vescValues2.wattHours = 0.0;
  vescValues2.wattHoursCharged = 0.0;
  vescValues2.tachometer = 0;
  vescValues2.tachometerAbs = 0;

  vescValues1.rpm = driveRPM;
  vescValues2.rpm = driveRPM;

  return true;
}


//Read gyro and calculate angle between them 
void readGyro() {

  
  //Calculate rpm value from sensor values
}


//Read throttle and convert value to useful range
void readThrottle() {
  int32_t throttleValue = analogRead(THROTTLE);
  throttleValue -= 900; //offset of ~840

  //if negative due to offset reset to 0
  if(throttleValue < 0) {
    throttleValue = 0;
  }
  //otherwise set rpm value to throttle input
  else {
    driveRPM = throttleValue;
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

  //read gyro, function controls desired speed
  readGyro();

  //read throttle, if used it overwrites speed value by gyro
  readThrottle();
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
        int32_t velocity1 = ((vescValues1.rpm * 254) / 60000);
        int32_t velocity2 = ((vescValues2.rpm * 254) / 60000);
        int32_t velocity = (velocity1 + velocity2) / 2;
    
        display.setTextSize(2);
        display.print(velocity);
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

  //Setup first gyro
  mpu1.initialize();

  //Setup second gyro
  pinMode(GYRO, OUTPUT);
  digitalWrite(GYRO, HIGH);
  mpu2.initialize();
  
  //Initialize display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(50);
  display.clearDisplay();
  display.setTextColor(WHITE);

  if (mpu1.testConnection() || mpu2.testConnection) {
    display.setTextSize(2);
    display.println("Connection to one or both position sensors failed");
    display.display();
    delay(2000);
    display.clearDisplay();
  }

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
