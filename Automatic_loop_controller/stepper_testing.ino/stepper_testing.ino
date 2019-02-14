#include <AccelStepper.h>
#include <MultiStepper.h>

AccelStepper stepper(4, 36, 37, 38, 39);

// Define the Pins used
#define ccw_home_switch 31 // Pin 31 connected to Home Switch (MicroSwitch)
#define  cw_home_switch 30 // Pin 30 connected to Home Switch (MicroSwitch)

// Stepper Travel Variables
long Travel;  // Used to store the X value entered in the Serial Monitor
int move_finished=1;  // Used to check if move is completed
long initial_homing=-1;  // Used to Home Stepper at startup


void setup() {
  Serial.begin(9600);  // Start the Serial monitor with speed of 9600 Bauds
   
  pinMode(ccw_home_switch, INPUT_PULLUP);
  pinMode(cw_home_switch,  INPUT_PULLUP);
   
  delay(5); 

  homeStepperMotor();
}

void loop() {
  // Print out Instructions on the Serial Monitor at Start
  Serial.println("Enter Travel distance (Positive for CW / Negative for CCW and Zero for back to Home): ");

  while (Serial.available()>0)  { // Check if values are available in the Serial Buffer

  move_finished=0;  // Set variable for checking move of the Stepper
  
  Travel= Serial.parseInt();  // Put numeric value from buffer in Travel variable
  if (Travel < 0 || Travel > 1350) {  // Make sure the position entered is not beyond the HOME or MAX position
    Serial.println("");
    Serial.println("Please enter a value greater than zero and smaller or equal to 1350.....");
    Serial.println("");
  } else {
    Serial.print("Moving stepper into position: ");
    Serial.println(Travel);
  
  stepper.moveTo(Travel);  // Set new moveto position of Stepper
  
  delay(1000);  // Wait 1 seconds before moving the Stepper
  }
  }

  if (Travel >= 0 && Travel <= 1350) {

// Check if the Stepper has reached desired position
  if ((stepper.distanceToGo() != 0)) {
    
    stepper.run();  // Move Stepper into position
    
  }

// If move is completed display message on Serial Monitor
  if ((move_finished == 0) && (stepper.distanceToGo() == 0)) {
    Serial.println("COMPLETED!");
    Serial.println("");
     Serial.println("Enter Travel distance (Positive for CW / Negative for CCW and Zero for back to Home): ");
    move_finished=1;  // Reset move variable
  }
  }
}


void homeStepperMotor()
{
  //  Set Max Speed and Acceleration of each Steppers at startup for homing
  stepper.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(100.0);  // Set Acceleration of Stepper
 

  // Start Homing procedure of Stepper Motor at startup
  Serial.print("Stepper is Homing . . . . . . . . . . . ");

  while (digitalRead(ccw_home_switch)) {  // Make the Stepper move CCW until the switch is activated   
    stepper.moveTo(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    stepper.run();  // Start moving the stepper
    delay(5);
  }

  stepper.setCurrentPosition(0);  // Set the current position as zero for now
  stepper.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(100.0);  // Set Acceleration of Stepper
  initial_homing=1;

  while (!digitalRead(ccw_home_switch)) { // Make the Stepper move CW until the switch is deactivated
    stepper.moveTo(initial_homing);  
    stepper.run();
    initial_homing++;
    delay(5);
  }
  
  stepper.setCurrentPosition(0);
  Serial.println("Homing Completed");
  Serial.println("");
  stepper.setMaxSpeed(1000.0);      // Set Max Speed of Stepper (Faster for regular movements)
  stepper.setAcceleration(1000.0);  // Set Acceleration of Stepper


}
