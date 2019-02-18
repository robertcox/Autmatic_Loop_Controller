#include "AccelStepper.h" 

// AccelStepper Setup
AccelStepper stepper(AccelStepper::FULL4WIRE, 36, 37, 38, 39);   

// Define the Pins used
#define ccw_home_switch 31 // Pin 31 connected to home micro switch (this will be the 0 position)
#define  cw_home_switch 30 // Pin 30 connected to initial homing micro switch (this will be variable)

// Stepper Travel Variables
long destination;        // Used to store the X value entered in the Serial Monitor
int move_finished=1;     // Used to check if move is completed
long initial_homing=-1;  // Used to home the cw rotation of the stepper at startup


void setup() {
   Serial.begin(9600);   // Start the Serial monitor with speed of 9600 Bauds
   
   pinMode(ccw_home_switch, INPUT_PULLUP);
   pinMode(cw_home_switch, INPUT_PULLUP);
   
   //  Set Max Speed and Acceleration of each Steppers at startup for homing
  stepper.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(100.0);  // Set Acceleration of Stepper

  // Start Homing procedure of Stepper Motor at startup

  Serial.print("Stepper is homing clockwise. . . . . . . . . . . ");

  while (digitalRead(cw_home_switch)) {  // Make the Stepper move CCW until the switch is activated   
    stepper.moveTo(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    stepper.run();  // Start moving the stepper
    delay(5);
  }

  stepper.setCurrentPosition(0);  // Set the current position as zero for now
  stepper.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(100.0);  // Set Acceleration of Stepper
  initial_homing=1;

  while (!digitalRead(cw_home_switch)) { // Make the Stepper move CW until the switch is deactivated
    stepper.moveTo(initial_homing);  
    stepper.run();
    initial_homing++;
    delay(5);
  }
  
  stepper.setCurrentPosition(0);
  Serial.println("Clockwise homing completed");
  Serial.println("");
  stepper.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Faster for regular movements)
  stepper.setAcceleration(100.0);  // Set Acceleration of Stepper

  delay(1000);    // Pause for one second before homing the counter clockwise rotation.

  Serial.print("Stepper is homing counter clockwise. . . . . . . . . . . ");

  while (digitalRead(ccw_home_switch)) {  // Make the Stepper move CCW until the switch is activated   
    stepper.moveTo(initial_homing);  // Set the position to move to
    initial_homing++;  // Decrease by 1 for next move if needed
    stepper.run();  // Start moving the stepper
    delay(5);
  }

  stepper.setCurrentPosition(initial_homing);  // Set the current position as zero for now
  stepper.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  stepper.setAcceleration(100.0);  // Set Acceleration of Stepper

  while (!digitalRead(ccw_home_switch)) { // Make the Stepper move CW until the switch is deactivated
    stepper.moveTo(initial_homing);  
    stepper.run();
    initial_homing--;
    delay(5);
  }
  
  stepper.setCurrentPosition(initial_homing);
  Serial.println("Counter clockwise homing Completed");
  Serial.println("");
  Serial.print("Initial Homing: ");
  Serial.println(initial_homing);
  
  stepper.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Faster for regular movements)
  stepper.setAcceleration(100.0);  // Set Acceleration of Stepper

// Print out Instructions on the Serial Monitor at Start
  Serial.print("Enter destination position between 0 and ");
  Serial.println(initial_homing);
}

void loop() {

 while (Serial.available() > 0)  { // Check if values are available in the Serial Buffer

  move_finished=0;  // Set variable for checking move of the Stepper
  
  destination = Serial.parseInt();  // Put numeric value from buffer in destination variable
    Serial.print("Initial value of destination: ");
    Serial.println(destination);
  if (destination < 0 || destination > initial_homing) {  // Make sure the position entered is not beyond the HOME or MAX position
    Serial.println("");
    Serial.print("Please enter a value greater than zero and smaller or equal to ");
    Serial.print(initial_homing);
    Serial.println(".....");
    Serial.println("");
    } else {
    Serial.print("Moving stepper into position: ");
    Serial.println(destination);
    stepper.moveTo(destination);  // ******************* Set new moveto position of Stepper
    Serial.print("The target position is: ");
    Serial.println(stepper.targetPosition());

    stepper.setSpeed(100.0); 
    stepper.setAcceleration(100.0); 
    delay(1000);  // Wait 1 seconds before moving the Stepper
   }
 }

  if (destination >= 0 && destination <= initial_homing) {
    // Check if the Stepper has reached desired position
    if ((stepper.distanceToGo() != 0)) {   // ******************* 
     Serial.print("Current position is: ");
     Serial.print(stepper.currentPosition());
     Serial.print(" - distance to target position: ");
     Serial.println(stepper.distanceToGo());
      
      stepper.run();  // ******************* Move Stepper into position
    }

    // If move is completed display message on Serial Monitor
    if ((move_finished == 0) && (stepper.distanceToGo() == 0)) {
      Serial.println("COMPLETED!");
      Serial.println("");
      Serial.print("Enter destination position between 0 and ");
      Serial.println(initial_homing);
      move_finished=1;  // Reset move variable
    }
  }
}
