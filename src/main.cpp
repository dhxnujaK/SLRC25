#include <Arduino.h>

void setup() {
  // Initialize pin 13 as an output
  pinMode(13, OUTPUT);
}

void loop() {
  // Turn the LED on (HIGH is the voltage level)
  digitalWrite(13, HIGH);
  
  // Wait for 1000 milliseconds (1 second)
  delay(500);
  
  // Turn the LED off by making the voltage LOW
  digitalWrite(13, LOW);
  
  // Wait for 1000 milliseconds (1 second)
  delay(1000);
}