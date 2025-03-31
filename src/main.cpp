#include <Arduino.h>
#include <EEPROM.h>

// Define pins for TCS230
#define S0 46
#define S1 47
#define S2 42
#define S3 44
#define OUT 43
#define BUZZER 9  // Pin for buzzer
#define LED 13   // Pin for LED

// Set color scale (you can adjust these based on your requirements)
#define SCALE 1000  // Adjust this value as necessary

// EEPROM locations to store calibration values
#define EEPROM_WHITE_RED 0
#define EEPROM_WHITE_GREEN 1
#define EEPROM_WHITE_BLUE 2
#define EEPROM_ORANGE_RED 3
#define EEPROM_ORANGE_GREEN 4
#define EEPROM_ORANGE_BLUE 5

// Calibration time in milliseconds (2 minutes = 120000 ms)
#define CALIBRATION_TIME_LIMIT 10000

unsigned long calibrationStartTime;

// Function declarations
void startCalibration();
void calibrateColor(int redFilter, int greenFilter, int blueFilter, int &redValue, int &greenValue, int &blueValue, int samples = 10);
void saveCalibrationToEEPROM(int whiteRed, int whiteGreen, int whiteBlue, int orangeRed, int orangeGreen, int orangeBlue);
void performTask();
void LEDblink();
void LEDnoblink();

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Set the pins as output/input
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  pinMode(BUZZER, OUTPUT);  // Set buzzer pin as output
  pinMode(LED,OUTPUT);

  // Configure frequency scaling
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);

  // Give some time for sensor initialization
  delay(500);

  // Start the calibration process
  startCalibration();
}

void loop() {
  // Check if the 2-minute window for calibration has passed
  bool isRun = false;
  if ((millis() - calibrationStartTime > CALIBRATION_TIME_LIMIT) || !isRun) {
    // After calibration time window, we can no longer calibrate, so we perform the task
    performTask();
    isRun = true;
  }
}

void startCalibration() {
  // Start time measurement for calibration window
  calibrationStartTime = millis(); // Record the starting time

  // Calibration values for White and Orange
  int whiteRed, whiteGreen, whiteBlue;
  int orangeRed, orangeGreen, orangeBlue;

  // Measure white color
  calibrateColor(255, 255, 255, whiteRed, whiteGreen, whiteBlue);
  //LEDblink();
  delay(1000); // Wait for 1 second to stabilize the readings
  // Measure orange color
  calibrateColor(255, 100, 0, orangeRed, orangeGreen, orangeBlue);

  // Save calibrated values to EEPROM
  //saveCalibrationToEEPROM(whiteRed, whiteGreen, whiteBlue, orangeRed, orangeGreen, orangeBlue);

  // Indicate end of calibration
  tone(BUZZER, 1500);  // Play tone at 1500 Hz
  delay(200);  // Beep for 200ms to indicate the end
  noTone(BUZZER);  // Stop the tone

  // Done with calibration, now wait for the next step
  Serial.println("Calibration complete. System will perform the task after 2 minutes.");
}

void calibrateColor(int redFilter, int greenFilter, int blueFilter, int &redValue, int &greenValue, int &blueValue, int samples = 10) {
  // Measure Red intensity (average of multiple samples)
  LEDblink();
  redValue = 0;
  for (int i = 0; i < samples; i++) {
    digitalWrite(S2, redFilter);
    digitalWrite(S3, greenFilter);
    redValue += pulseIn(OUT, HIGH);
    delay(10); // Small delay between readings
  }
  redValue /= samples; // Average

  // Measure Green intensity
  greenValue = 0;
  for (int i = 0; i < samples; i++) {
    digitalWrite(S2, greenFilter);
    digitalWrite(S3, redFilter);
    greenValue += pulseIn(OUT, HIGH);
    delay(10);
  }
  greenValue /= samples;

  // Measure Blue intensity
  blueValue = 0;
  for (int i = 0; i < samples; i++) {
    digitalWrite(S2, blueFilter);
    digitalWrite(S3, redFilter);
    blueValue += pulseIn(OUT, HIGH);
    delay(10);
  }
  blueValue /= samples;
  delay(1000);
  LEDnoblink(); // Turn off LED after calibration
}

void saveCalibrationToEEPROM(int whiteRed, int whiteGreen, int whiteBlue, int orangeRed, int orangeGreen, int orangeBlue) {
  // Save white color values to EEPROM
  EEPROM.write(EEPROM_WHITE_RED, whiteRed);
  EEPROM.write(EEPROM_WHITE_GREEN, whiteGreen);
  EEPROM.write(EEPROM_WHITE_BLUE, whiteBlue);

  // Save orange color values to EEPROM
  EEPROM.write(EEPROM_ORANGE_RED, orangeRed);
  EEPROM.write(EEPROM_ORANGE_GREEN, orangeGreen);
  EEPROM.write(EEPROM_ORANGE_BLUE, orangeBlue);
}

void performTask() {
  // This function will perform the task once calibration is complete and the 2-minute window has passed.
  
  // Retrieve calibrated color values from EEPROM
  int whiteRed = EEPROM.read(EEPROM_WHITE_RED);
  int whiteGreen = EEPROM.read(EEPROM_WHITE_GREEN);
  int whiteBlue = EEPROM.read(EEPROM_WHITE_BLUE);

  int orangeRed = EEPROM.read(EEPROM_ORANGE_RED);
  int orangeGreen = EEPROM.read(EEPROM_ORANGE_GREEN);
  int orangeBlue = EEPROM.read(EEPROM_ORANGE_BLUE);

}

void LEDblink(){
  digitalWrite(LED,HIGH);
}
void LEDnoblink(){
  digitalWrite(LED,LOW);
}