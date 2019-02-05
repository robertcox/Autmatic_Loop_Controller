#include <SoftwareSerial.h>   // This is the header file for the serial library we will be using
#include <LiquidCrystal.h>    // This is the header file for the LCD that we will be using
#include "FT817.h"            // This is the header file for the FT817 library that we will be using

LiquidCrystal lcd(51, 50, 49, 48, 47, 46); // These are the pins that setup the LCD (rs,e,d4,d5,d6,d7)

int analogPinFwd = A14;     // ** Forward Voltage in from the SWR Bridge
int analogPinRef = A15;     // ** Reflected Voltage in from the SWR Bridge

const int rxPin = 52;   // This is the receive pin from the CAT control cable
const int txPin = 53;   // This is the transmit pin on the CAT control cable
const int numRows = 4;  // The number of rows for our LCD display
const int numCols = 16; // The number of columns for our LCD display
unsigned long current_freq; // definition of the current_freq variable
float swr = 0.0;        // ** This will be the bridge's calculated SWR, change it from a int to a float.
int rig_swr = 0;        // ** This is now the FT-817's internal SWR 
 
String mode = "USB";    // This is the mode string for displaying on the LCD
byte orig_mode = 0;     // This is a variable to store the original mode of the radio.
int xmit = 0;           // This is a transmit flag

FT817 rig;              // This is the rig object for our radio.

void setup() {
    analogReference(INTERNAL2V56); // ** Sets the analog reference voltage to 2.56V (Arduino Mega only)
    Serial.begin(9600); // This sets up the serial interface between the computer and the Arduino
    SoftwareSerial mySerial(rxPin,txPin); // This sets up the serial interface between the Arduino and the FT-817ND
    rig.assignSerial(mySerial);   // The serial setup is now associated with the rig (FT-817ND)
    rig.begin(9600);              // This is the serial CAT RATE of the FT-817ND (menu 14)
    lcd.begin(numCols, numRows);  // This sets up the LCD and defines the rows and columns
    current_freq = 1417500;       // **** This is the our starting frequency
    rig.setMode(FT817_MODE_USB);  // This initially sets the FT-817ND to USB    
    mode = "FM ";                 // change the mode text to FM for display
    rig.setFreq(current_freq);    // ** set the current frequency
    ft817_display(mode, current_freq, rig_swr, swr, xmit); // ** This displays everything
    delay(2000);                  // Delay for two seconds to read the initial values on the screen
}

void loop() {
    orig_mode = rig.getMode();    // Get current mode and store the it in the mode variable
    ft817_display(mode, current_freq, rig_swr, swr, xmit); // ** display everything.

  /* This is a while loop that will increment through the 20 meter  
   *  band and increment the current frequency by 1000 every time  
   *  through the loop. If the current frequency is greater than 
   *  1435000 then we will break out of the loop.
   */

  while (current_freq <= 1402000){
//  while (current_freq <= 1435000){
//    rig.setFreq(current_freq);  // set the current frequency
//    ft817_display(mode, current_freq, rig_swr, swr, xmit); // **
//    delay(2000);                // Wait for two seconds
    rig.setMode(FT817_MODE_FM); // set the FT-817ND to FM
    mode = "FM ";               // change the mode text to FM for display
    rig.setPTTOn();             // Transmit (PTT on)
    xmit = true;                // set the xmit flag to true
    ft817_display(mode, current_freq, rig_swr, swr, xmit); // **
    delay(1000);                // delay for one second
    rig_swr = rig.getSWR();     // ** read the internal SWR from the FT-817ND.
    swr = actual_swr(analogPinFwd, analogPinRef);

    ft817_display(mode, current_freq, rig_swr, swr, xmit); // **
    
    delay(2000);                // delay for two seconds
    rig.setPTTOff();            // Turn off transmit (PTT off)
    xmit = false;               // set the xmit flag to false 
    rig_swr = NULL;
    swr = NULL;
    delay(100);                 // wait for just a very short time
    rig.setMode(orig_mode);     // set the FT-817ND back to the original mode
    mode = "USB";               // change the mode text to USB for display
    ft817_display(mode, current_freq, rig_swr, swr, xmit); // **
    delay(2000);                // wait for two seconds
    current_freq = current_freq + 1000; // increment the frequency.
  }
  dummyload_change();           // This displays the "Change Dummyload" message
  Serial.println("Back from Dummyload change");

  current_freq = 1400000;       // This is the our starting frequency
  xmit = false;               // set the xmit flag to false 
  rig_swr = NULL;
  swr = NULL;
  delay(2000);
}

/*
 * This is a function that will display the information  
 * we are tracking on a 16x4 LCD. 
 */
void ft817_display(String mode, unsigned long current_freq, int rig_swr, float swr, int xmit){   // ** 
  /* First line of the LCD */
  lcd.setCursor(0, 0);          // Start at the top left of the LCD
  lcd.print("817 CAT Control"); // Output this on the first line of the LCD
  /* Second line of the LCD */
  lcd.setCursor(0, 1);          // Skip down a line on the LCD
  lcd.print("M:");              // ** Output this string for the mode
  lcd.print(mode);              // Output the mode
  lcd.setCursor(7, 1);          // ** Move the cursor to the seventh position of the line
  lcd.print("T:");              // ** Output this string for the transmit
  if(xmit)                      // If the radio is transmitting
    lcd.print("X");             // Output this string if the radio is trasmitting
  else                          // Otherwise
    lcd.print("O");             // Output this string if the radio is not transmitting
  lcd.setCursor(12, 1);         // ** Move the cursor to the twelth position of the line
  lcd.print("S:");              // Output this string for the internal SWR
  if(xmit)                      // If the radio is transmitting
    lcd.print(rig_swr);         // ** Output out the SWR
  else                          // Otherwise
    lcd.print("  ");            // Output a blank string
  /* Third line of the LCD */  
  lcd.setCursor(0, 2);          // Skip down to the third line of the LCD
  lcd.print("Freq: ");          // Output this string
  lcd.print((float)current_freq/100000,3);  // This line converts the frequency into a float for displaying
  lcd.print(" Mhz    ");        // Output this string 
  /* Fourth line of the LCD */  
  lcd.setCursor(0, 3);          // Skip down to the fourth line of the LCD
  lcd.print("Actual SWR: ");    // Output this string
  if(xmit)                      // If the radio is transmitting
    lcd.print(swr);         // ** Output out the SWR
  else                          // Otherwise
    lcd.print("    ");            // Output a blank string
}

/*
 * This is a function that will display a message to change the dummyload  
 * and it will give the user 60 seconds to complete the change. 
 * A 60 second countdown timer is also displayed.
 */
void dummyload_change(){
//  int sec = 60;                  // This is a countdown counter initialized to 60 seconds.
  int sec = 5;                  // This is a countdown counter initialized to 60 seconds.
  lcd.setCursor(0, 2);           // Skip down to the fourth line of the LCD
  lcd.print("Dummyload Change"); // Output this string
  lcd.setCursor(0, 3);           // Skip down to the third line of the LCD
  lcd.print("You have: ");       // Output this string
  while(sec > 0){                // While the seconds is greater than zero 
    lcd.setCursor(10, 3);        // Skip down to the third line of the LCD
    lcd.print(sec);              // Output the current number of seconds
    lcd.print("s  ");            // Output the s string for seconds
    delay(1000);                 // Wait for one second
    sec--;                       // Decrement the number of seconds
  }
  lcd.setCursor(0, 2);           // Go to the third line of the LCD
  lcd.print("                "); // Output a blank string of 16 characters to clear the line
  lcd.setCursor(0, 3);           // Go to the fourth line of the LCD
  lcd.print("                "); // Output a blank string of 16 characters to clear the line
}

float actual_swr(int analogPinFwd, int analogPinRef){
  double fwd = 0;           // variable to store the value read
  double ref = 0;
  float swr = 0.0;
  fwd = analogRead(analogPinFwd); // read the input voltage for the forward power. 
  ref = analogRead(analogPinRef); // read the input voltage for the reflected power.

  swr = (fwd+ref)/(fwd-ref);      // Calculate the SWR from the measured voltages.
  Serial.print("FWD: ");
  Serial.print(fwd);
  Serial.print(" - REF: ");
  Serial.print(ref);
  Serial.print(" - SWR: ");
  Serial.println(swr);
 
  return swr;
}
