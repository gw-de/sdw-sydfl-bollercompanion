//Define GPIO pin for button
#define BUTTON 18

//Interrupt Service Routine for GPIO pin, motor brakes as long as the button is open
void IRAM_ATTR ISR() {
  Serial.println("Interrupt");
  delay(2000);
}


//Initialization
void setup() {
  //Setup serial connection to VESC 1
  Serial.begin(115200);
  delay(50);

  //Setup GPIO pin with button for interrupt
  pinMode(BUTTON, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(BUTTON), ISR, HIGH);
  attachInterrupt(BUTTON, ISR, HIGH);  

  Serial.println("Setup");
}


//Program routine
void loop() {  
}
