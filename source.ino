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

//Define ids for gyros
#define MPU6050_ADRESS1 0x68
#define MPU6050_ADRESS2 0x69

//Gyro constants
const int ACCEL_OFFSET   = 200;
const int GYRO_OFFSET    = 151;  // 151
const int GYRO_SENSITITY = 131;  // 131 is sensivity of gyro from data sheet
const float GYRO_SCALE   = 0.2; //  0.02 by default - tweak as required
const float LOOP_TIME    = 0.15; // 0.1 = 100ms

//Global variables
int accValue1[3], accAngle1[3], gyroValue1[3], temperature1, accCorr1;
float gyroAngle1[3], gyroCorr1;
int accValue2[3], accAngle2[3], gyroValue2[3], temperature2, accCorr2;
float gyroAngle2[3], gyroCorr2;
boolean openswitch;
boolean vescunavailable;
boolean overheating1;
boolean overheating2;
int32_t throttleValue;
int32_t driveRPM;

/*
 * Create required class instances
 */
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

  /*
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
  */
}


//Read gyro and calculate angle between them 
void readGyro() {
  //Read first gyro
  Wire.beginTransmission(MPU6050_ADRESS1);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU6050_ADRESS1, 7*2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  for(byte i=0; i<3; i++) {
    accValue1[i] = Wire.read()<<8 | Wire.read(); // reading registers: ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT
  }
  temperature1 = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  for(byte i=0; i<3; i++) {
    gyroValue1[i] = Wire.read()<<8 | Wire.read(); // reading registers: GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT
  }

  //Read second gyro
  Wire.beginTransmission(MPU6050_ADRESS2);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU6050_ADRESS2, 7*2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  for(byte i=0; i<3; i++) {
    accValue2[i] = Wire.read()<<8 | Wire.read(); // reading registers: ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT
  }
  temperature2 = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  for(byte i=0; i<3; i++) {
    gyroValue2[i] = Wire.read()<<8 | Wire.read(); // reading registers: GYRO_XOUT, GYRO_YOUT, GYRO_ZOUT
  }

  //Calculate angle for first gyro
  for(byte i=0; i<3; i++) {
    accCorr1 = accValue1[i] - ACCEL_OFFSET;
    accCorr1 = map(accCorr1, -16800, 16800, -90, 90);
    accAngle1[i] = constrain(accCorr1, -90, 90);
  }

   for(byte i=0; i<3; i++) {
    gyroCorr1 = (float)((gyroValue1[i]/GYRO_SENSITITY) - GYRO_OFFSET);
    gyroAngle1[i] = (gyroCorr1 * GYRO_SCALE) * -LOOP_TIME;
  }

  //Calculate angle for second gyro
  for(byte i=0; i<3; i++) {
    accCorr2 = accValue2[i] - ACCEL_OFFSET;
    accCorr2 = map(accCorr2, -16800, 16800, -90, 90);
    accAngle2[i] = constrain(accCorr2, -90, 90);
  }

   for(byte i=0; i<3; i++) {
    gyroCorr2 = (float)((gyroValue2[i]/GYRO_SENSITITY) - GYRO_OFFSET);
    gyroAngle2[i] = (gyroCorr2 * GYRO_SCALE) * -LOOP_TIME;
  }
  
  //Calculate rpm value from sensor values
  //driveRPM = gyroAngle1[1] - gyroAngle2[1];
}


//Read throttle and convert value to useful range
void readThrottle() {
  int32_t throttleValue = analogRead(THROTTLE);
  throttleValue -= 900; //offset of ~840

  //scale remaining range of 0-2000 to rpm value
  //throttleValue *= 4;

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
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADRESS1); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //Setup second gyro
  pinMode(GYRO, OUTPUT);
  digitalWrite(GYRO, HIGH);
  delay(100);
  Wire.begin();
  Wire.beginTransmission(MPU6050_ADRESS2); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
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
  drive(driveRPM);
  
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
