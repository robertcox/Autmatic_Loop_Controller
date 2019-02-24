#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include "FT817.h"

const int rxPin = 52;       // These are the transmit and receive pins so that the 
const int txPin = 53;       // Arduino can talk to the FT-817
int tunePin = 45;
int slow_manualTunePin = 44;
int tune_upPin = 43;
int tune_downPin = 42;


//Declare pin functions on Redboard
#define stp 36
#define dir 37
#define MS1 38
#define MS2 39
#define EN  35
#define RST 34
#define SLP 28

#define  cw_home_switch 31 // Pin 31 connected to home micro switch (this will be the 0 position)
#define  ccw_home_switch 30 // Pin 30 connected to initial homing micro switch (this will be variable)
int cwHomeSwitchState  = LOW;
int ccwHomeSwitchState = LOW;

// Stepper Travel Variables
long destination;        // Used to store the X value entered in the Serial Monitor
int  move_finished=1;    // Used to check if move is completed
bool  direction=HIGH;        // HIGH=clockwise; LOW=counter clockwise
long initial_homing=-1;  // Used to home the cw rotation of the stepper at startup
long distanceToGo;
long currentPosition;
long stepsToGo;
long previousStepsToGo;

// Used for generating interrupts using CLK signal
const int PinA = 2;

// Used for reading DT signal
const int PinB = 4;

// Used for the push button switch
const int PinSW = 8;

// Keep track of last rotary value
int lastCount = 50;

// Updated by the ISR (Interrupt Service Routine)
volatile int virtualPosition = 50;

long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers

int tuneButtonState = 0;
int slowManualTuneButtonState = 0;
int tuneUpButtonState = 0;
int tuneDownButtonState = 0;

int analogPinFwd = A14;     // forward voltage from the SWR bridge is set to pin A14
int analogPinRev = A15;     // reverse voltage from the SWR bridge is set to pin A15

bool pttFlag = LOW;
double fwd = 0;             // variable to store the forward voltage value from the SWR bridge.
double rev = 0;             // variable to store the reverse voltage value from the SWR bridge.
float swr = 0.0;            // The calculated SWR will be a float
int radio_swr = 0;          // the FT-817 will report the values of the TX meter.
                            // The SWR will be reported as an integer number.
                            // From my experience this integer correlates to the number of bars
                            // that are displayed on the FT-817's SWR meter.  
unsigned long current_freq; // variable that contains the current frequency. 
byte mode;                  // This is a variable to store the mode that the radio is in.

LiquidCrystal lcd(51, 50, 49, 48, 47, 46); // rs,e,d4,d5,d6,d7 - The setup of the LCD

const int numRows = 4;      // I am using a 4x16 LCD
const int numCols = 16;

FT817 radio;                // The object will be referred to as radio.

// ------------------------------------------------------------------
// INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT     INTERRUPT
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
// SETUP    SETUP    SETUP    SETUP    SETUP    SETUP    SETUP
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
  pinMode(cw_home_switch, INPUT_PULLUP);
  
  // Just whilst we debug, view output on serial monitor
  Serial.begin(9600);

  // Rotary pulses are INPUTs
  pinMode(PinA, INPUT);
  digitalWrite(PinA, HIGH);       // turn on pull-up resistor
  pinMode(PinB, INPUT);
  digitalWrite(PinB, HIGH);       // turn on pull-up resistor

  // Switch is floating so use the in-built PULLUP so we don't need a resistor
  pinMode(PinSW, INPUT_PULLUP);

  // Attach the routine to service the interrupts
  attachInterrupt(digitalPinToInterrupt(PinA), isr, LOW);
  delay(100);
  SoftwareSerial mySerial(rxPin,txPin); // Setup the serial communication of the Arduino to the FT-817.
  radio.assignSerial(mySerial);         // Assign the setup to my radio.
  radio.begin(9600);                    // My FT-817 is set to be controlled at 9600 baud (menu #14: CAT RATE)
  lcd.begin(numCols, numRows);          // Setup the LCD
  lcd.setCursor(0, 0);                  // Set the LCD's cursor to column 0 row 0.
  lcd.print("817 CAT Control");         // Display this message on the LCD.
  radio.setFreq(current_freq);          // Set the FT-817's frequency to current_freq.
//  radio.setMode(FT817_MODE_FM);         // Set the FT-817's mode to FM.
  current_freq = 1417500;               // Set the current frequency to the middle of the 20 meter band.

  pinMode(cw_home_switch,INPUT);
  pinMode(ccw_home_switch,INPUT);
  
  pinMode(tunePin, INPUT);
  pinMode(slow_manualTunePin, INPUT);
  pinMode(tune_upPin, INPUT);
  pinMode(tune_downPin, INPUT);

  // Ready to go!
  Serial.println("Start");
  delay(100);
}

// ------------------------------------------------------------------
// MAIN LOOP     MAIN LOOP     MAIN LOOP     MAIN LOOP     MAIN LOOP
// ------------------------------------------------------------------
void loop() {
  cwHomeSwitchState = digitalRead(cw_home_switch);
  ccwHomeSwitchState = digitalRead(ccw_home_switch);
  tuneButtonState = digitalRead(tunePin);
  slowManualTuneButtonState = digitalRead(slow_manualTunePin);
  tuneUpButtonState = digitalRead(tune_upPin);
  tuneDownButtonState = digitalRead(tune_downPin);


  // Is someone pressing the rotary switch?
  if ((!digitalRead(PinSW))) {
    virtualPosition = 50;
    while (!digitalRead(PinSW))
      delay(10);
    Serial.println("Reset");
  }

  // Auto tune functionality. Switch to FM mode, power to low, PTT
  // read the SWR and adjust the stepper motor to tune the capacitor.
  if (tuneButtonState==LOW) {
    Serial.println("Tune Button has been pressed!");
    if(!pttFlag)
      mode = radio.getMode();               // Get the FT-817's current mode
    if(mode != FT817_MODE_FM){            // If the mode is not set to FM then set it to FM
      radio.setMode(FT817_MODE_FM);  
    }
    radio.setPTTOn();
    pttFlag = HIGH;
  }
  else if(pttFlag==HIGH && tuneButtonState==HIGH) {
    Serial.println("Tune Button has been released!");
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
    radio_swr = radio.getSWR();            
    fwd = analogRead(analogPinFwd);    // read the input pin
    rev = analogRead(analogPinRev);    // read the input pin
    swr = (fwd+rev)/(fwd-rev);
//    showSWRled(radio_swr);     
  } else {     // if the radio is not transmitting, then set the variables to 0.
//    setXmitColor(0, 255, 0);  // green
//    setSWRColor(220, 220, 0);  // red
    rev = 0;
    fwd = 0;
    swr = 0.0;
    radio_swr = 0;
  }
  lcdPrint(current_freq, fwd, rev, swr, radio_swr); // output the information to the LCD display.
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
    lcd.setCursor(0, 0);
    lcd.print("Freq:           ");
    lcd.setCursor(6, 0);
    lcd.print((float)current_freq/100000,3);
    lcd.print("Mhz");
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
