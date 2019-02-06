#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include "FT817.h"

const int rxPin = 52;       // These are the transmit and receive pins so that the 
const int txPin = 53;       // Arduino can talk to the FT-817

int swr_bluePin = 43;
int swr_greenPin = 42;
int swr_redPin = 41;

int xmit_bluePin = 39;
int xmit_greenPin = 38;
int xmit_redPin = 37;

int auto_tunePin = 45;
int slow_manualTunePin = 44;
int tune_upPin = 42;
int tune_downPin = 43;

int autoTuneButtonState = 0;
int slowManualTuneButtonState = 0;
int tuneUpButtonState = 0;
int tuneDownButtonState = 0;

int analogPinFwd = A14;     // forward voltage from the SWR bridge is set to pin A14
int analogPinRev = A15;     // reverse voltage from the SWR bridge is set to pin A15
                       
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

void setup()
{
  analogReference(INTERNAL2V56);        // sets the analog reference voltage to 2.56V (Arduino Mega only)
  Serial.begin(9600);                   // Set the serial communication from the Arduino to my computer to 9600.
  SoftwareSerial mySerial(rxPin,txPin); // Setup the serial communication of the Arduino to the FT-817.
  radio.assignSerial(mySerial);         // Assign the setup to my radio.
  radio.begin(9600);                    // My FT-817 is set to be controlled at 9600 baud (menu #14: CAT RATE)
  lcd.begin(numCols, numRows);          // Setup the LCD
  lcd.setCursor(0, 0);                  // Set the LCD's cursor to column 0 row 0.
  lcd.print("817 CAT Control");         // Display this message on the LCD.
  radio.setFreq(current_freq);          // Set the FT-817's frequency to current_freq.
  radio.setMode(FT817_MODE_FM);         // Set the FT-817's mode to FM.
  current_freq = 1417500;               // Set the current frequency to the middle of the 20 meter band.
  pinMode(swr_redPin, OUTPUT);
  pinMode(swr_greenPin, OUTPUT);
  pinMode(swr_bluePin, OUTPUT);
  pinMode(xmit_redPin, OUTPUT);
  pinMode(xmit_greenPin, OUTPUT);
  pinMode(xmit_bluePin, OUTPUT);
  
  pinMode(auto_tunePin, INPUT);
  pinMode(slow_manualTunePin, INPUT);
  pinMode(tune_upPin, INPUT);
  pinMode(tune_downPin, INPUT);
}

void loop()
{ 
  autoTuneButtonState = digitalRead(auto_tunePin);
  slowManualTuneButtonState = digitalRead(slow_manualTunePin);
  tuneUpButtonState = digitalRead(tune_upPin);
  tuneDownButtonState = digitalRead(tune_downPin);
  // Auto tune functionality. Switch to FM mode, power to low, PTT
  // read the SWR and adjust the stepper motor to tune the capacitor.
  if (autoTuneButtonState == HIGH) {
    
  } else {
    Serial.println("Auto Tune Button has been pressed!");

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
    }
    if (tuneDownButtonState == HIGH) {
      
    } else {
      Serial.println("Only the Tune Down Button has been pressed!");
    }
  } else {
    if (tuneUpButtonState == HIGH) {
      
    } else {
      Serial.println("Slow Manual Tune && Tune Up Button have been pressed!");
    }
    if (tuneDownButtonState == HIGH) {
      
    } else {
      Serial.println("Slow Manual Tune && Tune Down Buttons have been pressed!");
  
    }
  }


  current_freq = radio.getFreqMode("freq"); // Get the FT-817's current frequency
  
  mode = radio.getMode();               // Get the FT-817's current mode

  if(mode != FT817_MODE_FM){            // If the mode is not set to FM then set it to FM
    radio.setMode(FT817_MODE_FM);  
  }

  // Is the radio transmitting? 
  // If it is, then read the SWR of the radio and of the bridge.
  // and calculate the SWR from the forward and reverse voltage.
  if (radio.txState2()) {
    setXmitColor(255, 0, 0);  // red
    radio_swr = radio.getSWR();            
    fwd = analogRead(analogPinFwd);    // read the input pin
    rev = analogRead(analogPinRev);    // read the input pin
    swr = (fwd+rev)/(fwd-rev);
    showSWRled(radio_swr);     
  } else {     // if the radio is not transmitting, then set the variables to 0.
    setXmitColor(0, 255, 0);  // green
    setSWRColor(220, 220, 0);  // red
    rev = 0;
    fwd = 0;
    swr = 0.0;
    radio_swr = 0;
  }
  lcdPrint(current_freq, fwd, rev, swr, radio_swr); // output the information to the LCD display.
}

int showSWRled( int radio_swr ){
  if( radio_swr == 0 ){
    setSWRColor(0, 255, 0);  // green
  } else if (radio_swr == 2 || radio_swr == 3){
    setSWRColor(255, 255, 0);  // yellow    
  } else {
    setSWRColor(255, 0, 0);  // red    
  }
}

void setSWRColor(int red, int green, int blue)
{
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
  digitalWrite(swr_redPin, red);
  digitalWrite(swr_greenPin, green);
  digitalWrite(swr_bluePin, blue);  
}

void setXmitColor(int red, int green, int blue)
{
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
  digitalWrite(xmit_redPin, red);
  digitalWrite(xmit_greenPin, green);
  digitalWrite(xmit_bluePin, blue);  
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

