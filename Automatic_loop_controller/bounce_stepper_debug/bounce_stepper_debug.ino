//
// Make a single stepper bounce from one limit to another
//
// Copyright (C) 2012 Mike McCauley
// $Id: Random.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
#include <AccelStepper.h>

#define m1_sleep  28
#define m1_reset  34
#define m1_enable 35
#define m1_step   36
#define m1_dir    37
#define m1_ms1    38
#define m1_ms2    39


// AccelStepper Setup
AccelStepper stepper(AccelStepper::DRIVER, m1_step, m1_dir);   // Easy Driver interface


void setup()
{ 
  pinMode(m1_sleep,  OUTPUT);
  pinMode(m1_reset,  OUTPUT);
  pinMode(m1_enable, OUTPUT);
  pinMode(m1_ms1,    OUTPUT);
  pinMode(m1_ms2,    OUTPUT);
   
  Serial.begin(9600);   // Start the Serial monitor with speed of 9600 Bauds
  resetEDPins(); //Set step, direction, microstep and enable pins to default states
  Serial.println("Begin motor control");

  // Change these to suit your stepper if you want
  stepper.setMaxSpeed(5000);
  stepper.setAcceleration(500);
  stepper.moveTo(50);
}

void loop()
{
    // If at the end of travel go to the other end
    if (stepper.distanceToGo() == 0){
      stepper.moveTo(-stepper.currentPosition());
      Serial.println("---------------------- Reversing ---------------------");
    }
    Serial.print("Current position is: ");
    Serial.print(stepper.currentPosition());
    Serial.print(" - distance to target position: ");
    Serial.println(stepper.distanceToGo());
    stepper.run();
}

void resetEDPins()
{
  digitalWrite(m1_step,    LOW);
  digitalWrite(m1_dir,     LOW);
  digitalWrite(m1_ms1,     LOW);
  digitalWrite(m1_ms2,     LOW);
  digitalWrite(m1_enable,  LOW);
  digitalWrite(m1_sleep,  HIGH);
  digitalWrite(m1_reset,  HIGH);
  
}
