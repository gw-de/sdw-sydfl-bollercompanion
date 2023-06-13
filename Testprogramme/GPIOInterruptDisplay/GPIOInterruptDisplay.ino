#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//Define GPIO pin for button
#define BUTTON 18

//Variable for interrupt
boolean open;

//Interrupt Service Routine for GPIO pin, motor brakes as long as the button is open
void IRAM_ATTR ISR() {    
  delay(100);

  if(digitalRead(BUTTON)){
    open = false;
  }
  else {
    open = true;
  }
}


//Initialization
void setup() {
  //Setup serial connection to VESC 1
  Serial.begin(115200);
  delay(50);
  
  //Setup GPIO pin with button for interrupt
  pinMode(BUTTON, INPUT_PULLUP);
  attachInterrupt(BUTTON, ISR, CHANGE);  

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

  Serial.println("Setup");
}


//Program routine
void loop() {
  if (open) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Interrupt");
    display.display();
    Serial.println("Interrupt");    
  }
  else {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Loop");
    display.display();    
    Serial.println("Loop");  
  }
  
  delay(2000);

}
