#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include "FT817.h"

//Declare pin functions on the Sparkfun Easydriver
#define stp 36     // step pin
#define dir 37     // direction pin
#define MS1 38     // MS1 pin
#define MS2 39     // MS2 pin
#define EN  35     // enable pin
#define RST 34     // reset pin
#define SLP 28     // sleep pin

#define   cw_home_switch 31      // Pin 31 connected to home micro switch (this will be the 0 position)
#define  ccw_home_switch 30      // Pin 30 connected to initial homing micro switch (this will be variable)
int cwHomeSwitchState  = LOW;
int ccwHomeSwitchState = LOW;

const int rxPin              = 52;       // These are the transmit and receive pins so that the 
const int txPin              = 53;       // Arduino can talk to the FT-817

const int tunePin            = 45;       // Pushbutton pin for the Tune button
const int slow_manualTunePin = 44;       // Pushbutton pin for the Slow Manual button
const int tune_upPin         = 43;       // Pushbutton pin for the Tune Up button
const int tune_downPin       = 42;       // Pushbutton pin for the Tune Down button

char line0[17];      // definition for LCD line one of four
char line1[17];      // definition for LCD line two of four
char line2[17];      // definition for LCD line three of four
char line3[17];      // definition for LCD line four of four

// Stepper control variables
int  move_finished=1;        // Used to check if move is completed
bool  direction=HIGH;        // HIGH=clockwise; LOW=counter clockwise

const int  PinA = 2;         // Used for generating interrupts using CLK signal
const int  PinB = 4;         // Used for reading DT signal
const int PinSW = 8;         // Used for the push button switch

long time = 0;               // the last time the output pin was toggled
long debounce = 200;         // the debounce time, increase if the output flickers

int tuneButtonState = 0;     // initialization of the Tune button.
int slowManualTuneButtonState = 0;  // initialization of the Slow Tune button
int tuneUpButtonState = 0;   // initialization of the Tune Up button
int tuneDownButtonState = 0; // initialization of the Tune Down button

int analogPinFwd = A14;      // forward voltage from the SWR bridge is set to pin A14
int analogPinRev = A15;      // reverse voltage from the SWR bridge is set to pin A15

bool  pttFlag = LOW;         // initialization of the PTT Flag
double    fwd = 0;           // variable to store the forward voltage value from the SWR bridge.
double    rev = 0;           // variable to store the reverse voltage value from the SWR bridge.
float     swr = 0.0;         // The calculated SWR will be a float
int radio_swr = 0;           // the FT-817 will report the values of the TX meter.
                             // The SWR will be reported as an integer number.
                             // From my experience this integer correlates to the number of bars
                             // that are displayed on the FT-817's SWR meter.  
                             
unsigned long current_freq;  // variable that contains the FT-817's current frequency. 
byte mode;                   // This is a variable to store the FT-817's mode.

LiquidCrystal lcd(51, 50, 49, 48, 47, 46); // rs,e,d4,d5,d6,d7 - The setup of the LCD

const int numRows = 4;       // I am using a 4x16 LCD
const int numCols = 16;      // 

FT817 radio;                 // The object will be referred to as radio.

// ------------------------------------------------------------------
//     **************   Interrupt Service Routine ************** 
//  If the rotary encoder is moved an interrupt will trigger
//  and this routine will service that interrupt.
//  If turned, the direction is determined and the stepper
//  will be moved slowly in the direction that the rotary
//  encoder is moved. 
// ------------------------------------------------------------------
void isr ()  {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  cwHomeSwitchState = digitalRead(cw_home_switch);
  ccwHomeSwitchState = digitalRead(ccw_home_switch);
  
  // If interrupts come faster than 5ms, assume it's a bounce and ignore
  if (interruptTime - lastInterruptTime > 5) {
    if (digitalRead(PinB) == LOW)
    {
      if (ccwHomeSwitchState == HIGH){
        digitalWrite(EN, LOW); //Pull enable pin low to allow motor control
        moveStepper(5,LOW);
      }
    }
    else {
      if (cwHomeSwitchState == HIGH){
        digitalWrite(EN, LOW); //Pull enable pin low to allow motor control
        moveStepper(5,HIGH);
      }
    }
    // Keep track of when we were here last (no more than every 5ms)
    lastInterruptTime = interruptTime;
  }
}

// ------------------------------------------------------------------
//  Initial Setup routine
//    - initialize the Arduino Mega reference voltage
//    - Setup the modes for the Easydriver, the limit switches,
//        the rotary encoder, the interrupt service routine,
//        the serial communication with the FT-817nd, and the
//        the four control switches
// ------------------------------------------------------------------
void setup() {
  analogReference(INTERNAL2V56);        // sets the analog reference voltage to 2.56V (Arduino Mega only)

  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(EN, OUTPUT);

  
  resetEDPins(); //Set step, direction, microstep and enable pins to default states
  pinMode(ccw_home_switch, INPUT_PULLUP);
  pinMode( cw_home_switch, INPUT_PULLUP);
  pinMode(ccw_home_switch, INPUT);
  pinMode( cw_home_switch, INPUT);
  
  // Just whilst we debug, view output on serial monitor
  Serial.begin(9600);

  // Rotary encoder pulses are INPUTs
  pinMode( PinA, INPUT);
  pinMode( PinB, INPUT);
  pinMode(PinSW, INPUT_PULLUP);   // Switch is floating use the in-built PULLUP resistor

  digitalWrite(PinA,  HIGH);      // turn on pull-up resistor
  digitalWrite(PinB,  HIGH);      // turn on pull-up resistor

  pinMode(           tunePin, INPUT);  
  pinMode(slow_manualTunePin, INPUT);
  pinMode(        tune_upPin, INPUT);
  pinMode(      tune_downPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(PinA), isr, LOW);    // Attach the routine to service the interrupts
  delay(100);
  SoftwareSerial mySerial(rxPin,txPin); // Setup the serial communication of the Arduino to the FT-817.
  radio.assignSerial(mySerial);         // Assign the setup to my radio.
  radio.begin(9600);                    // My FT-817 is set to be controlled at 9600 baud (menu #14: CAT RATE)
  lcd.begin(numCols, numRows);          // Setup the LCD
  lcd.setCursor(0, 0);                  // Set the LCD's cursor to column 0 row 0.
  lcd.print("817 CAT Control");         // Display this message on the LCD.
  current_freq = 1417500;               // Set the current frequency to the middle of the 20 meter band.
  radio.setFreq(current_freq);          // Set the FT-817's frequency to current_freq.

  // Ready to go!
  Serial.println("Start");
  delay(100);
}

// ------------------------------------------------------------------
// Main loop
// ------------------------------------------------------------------
void loop() {
  cwHomeSwitchState         = digitalRead(cw_home_switch);
  ccwHomeSwitchState        = digitalRead(ccw_home_switch);
  tuneButtonState           = digitalRead(tunePin);
  slowManualTuneButtonState = digitalRead(slow_manualTunePin);
  tuneUpButtonState         = digitalRead(tune_upPin);
  tuneDownButtonState       = digitalRead(tune_downPin);

  lcdPrint(current_freq, fwd, rev, swr, radio_swr); // output the information to the LCD display.

  // Is someone pressing the rotary switch?
  if ((!digitalRead(PinSW))) {
      // virtualPosition = 50;
    while (!digitalRead(PinSW))
      delay(10);
    Serial.println("Reset");
  }

  // Auto tune functionality. Switch to FM mode, power to low, PTT
  // read the SWR and adjust the stepper motor to tune the capacitor.
  if (tuneButtonState==LOW) {
    // Serial.println("Tune Button has been pressed!");
    if(!pttFlag)
      mode = radio.getMode();               // Get the FT-817's current mode
    if(mode != FT817_MODE_FM){            // If the mode is not set to FM then set it to FM
      radio.setMode(FT817_MODE_FM);  
    }
    radio.setPTTOn();
    pttFlag = HIGH;
  }
  else if(pttFlag==HIGH && tuneButtonState==HIGH) {
    // Serial.println("Tune Button has been released!");
    radio.setPTTOff();
    delay(10);
    radio.setMode(mode);
    pttFlag = LOW;  
  }

  // Manual tune functionality.
  // Pushing the Tune Up button will go one way
  // Pushing the Tune Down button will go the other way.
  // Pushing the Slow Manual Tune button will do nothing
  // unless it is pushed with the Tune Up or Tune Down 
  // buttons then the stepper will tune slowly.
  if (slowManualTuneButtonState == HIGH) {
    if (tuneUpButtonState == HIGH) {
      
    } else {
      Serial.println("Only the Tune Up Button has been pressed!");
      if (cwHomeSwitchState == HIGH){
        digitalWrite(EN, LOW); //Pull enable pin low to allow motor control
        moveStepper(50,HIGH);
      }
      else {
        Serial.println("CW Limit switch is ENGAGED - the stepper will not move CW.");
      }
    }
    if (tuneDownButtonState == HIGH) {
      
    } else {
      Serial.println("Only the Tune Down Button has been pressed!");
      if (ccwHomeSwitchState == HIGH){
        digitalWrite(EN, LOW); //Pull enable pin low to allow motor control
        moveStepper(50,LOW);
      }
      else {
        Serial.println("CCW Limit switch is ENGAGED - the stepper will not move CCW."); 
      }

    }
  } else {
    if (tuneUpButtonState == HIGH) {

    } else {
      Serial.println("Slow Manual Tune && Tune Up Button have been pressed!");
      if (cwHomeSwitchState == HIGH){
        digitalWrite(EN, LOW); //Pull enable pin low to allow motor control
        moveStepper(10,HIGH);   
      }
      else {
        Serial.println("CW Limit switch is ENGAGED - the stepper will not move CW.");
      }
    }
    if (tuneDownButtonState == HIGH) {
      
    } else {
      Serial.println("Slow Manual Tune && Tune Down Buttons have been pressed!");
      if (ccwHomeSwitchState == HIGH){
        digitalWrite(EN, LOW); //Pull enable pin low to allow motor control
        moveStepper(10,LOW);
      }
      else {
        Serial.println("CCW Limit switch is ENGAGED - the stepper will not move CCW."); 
      }
    }
  }

  current_freq = radio.getFreqMode("freq"); // Get the FT-817's current frequency
  

  // Is the radio transmitting? 
  // If it is, then read the SWR of the radio and of the bridge.
  // and calculate the SWR from the forward and reverse voltage.
  if (radio.txState2()) {

//    setXmitColor(255, 0, 0);  // red
    
    if(radio.getMode() == FT817_MODE_FM){
      radio_swr = radio.getSWR();            
      fwd = analogRead(analogPinFwd);    // read the input pin
      rev = analogRead(analogPinRev);    // read the input pin
      swr = (fwd+rev)/(fwd-rev);
    }
//    showSWRled(radio_swr);     
  } else {     // if the radio is not transmitting, then set the variables to 0.
//    setXmitColor(0, 255, 0);  // green
//    setSWRColor(220, 220, 0);  // red
    rev = 0;
    fwd = 0;
    swr = 0.0;
    radio_swr = 0;
  }
}

void moveStepper(int speed,bool direction)
{
  int x;
  digitalWrite(MS1, HIGH); //Pull MS1, and MS2 high to set logic to 1/8th microstep resolution
  digitalWrite(MS2, HIGH);

  Serial.println("Moving at eighth-step mode.");
  digitalWrite(dir, direction); //Pull direction pin high to move in "reverse"
  for(x= 1; x < speed; x++)  //Loop the stepping enough times for motion to be visible
  {
    digitalWrite(stp,HIGH); //Trigger one step
    delay(1);
    digitalWrite(stp,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }

}

int lcdPrint(unsigned long current_freq, int fwd, int rev, float swr, int radio_swr){
    char str[7];
    dtostrf((float)current_freq/100000, 7, 3, str);    
    
    lcd.setCursor(0, 0);
    snprintf(line0, sizeof line0, "%s%s%s", "Freq: ", str, "Mhz");
    lcd.print(line0);
    lcd.setCursor(0, 1);
    lcd.print("Fwd:              ");
    lcd.setCursor(4, 1);
    lcd.print(fwd);
    lcd.setCursor(9, 1);
    lcd.print("Rev:");
    lcd.print(rev);
    lcd.setCursor(0, 2);
    lcd.print("Bridge SWR: ");
    lcd.print(swr);
    lcd.setCursor(0, 3);
    lcd.print("FT-817 SWR: ");
    lcd.print(radio_swr);
}

//Reset Easy Driver pins to default states
void resetEDPins()
{
  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(EN, HIGH);
  digitalWrite(SLP,HIGH);
  digitalWrite(RST,HIGH);
}
