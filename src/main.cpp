#include <Arduino.h>
#include <EEPROM.h>

// Define pins for TCS230
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define OUT 8
#define BUZZER 9  // Pin for buzzer

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
#define CALIBRATION_TIME_LIMIT 120000

unsigned long calibrationStartTime;

// Function declarations
void startCalibration();
void calibrateColor(int redFilter, int greenFilter, int blueFilter, int &redValue, int &greenValue, int &blueValue);
void saveCalibrationToEEPROM(int whiteRed, int whiteGreen, int whiteBlue, int orangeRed, int orangeGreen, int orangeBlue);
void performTask();

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
  if (millis() - calibrationStartTime > CALIBRATION_TIME_LIMIT) {
    // After calibration time window, we can no longer calibrate, so we perform the task
    performTask();
  }
}

void startCalibration() {
  // Indicate start of calibration
  tone(BUZZER, 1000);  // Play tone at 1000 Hz
  delay(200);  // Beep for 200ms to indicate the start
  noTone(BUZZER);  // Stop the tone

  // Start time measurement for calibration window
  calibrationStartTime = millis(); // Record the starting time

  // Calibration values for White and Orange
  int whiteRed, whiteGreen, whiteBlue;
  int orangeRed, orangeGreen, orangeBlue;

  // Measure white color
  calibrateColor(255, 255, 255, whiteRed, whiteGreen, whiteBlue);

  // Measure orange color
  calibrateColor(255, 100, 0, orangeRed, orangeGreen, orangeBlue);

  // Save calibrated values to EEPROM
  saveCalibrationToEEPROM(whiteRed, whiteGreen, whiteBlue, orangeRed, orangeGreen, orangeBlue);

  // Indicate end of calibration
  tone(BUZZER, 1500);  // Play tone at 1500 Hz
  delay(200);  // Beep for 200ms to indicate the end
  noTone(BUZZER);  // Stop the tone

  // Print the calibration results to serial monitor
  Serial.print("White Calibration - R: ");
  Serial.print(whiteRed);
  Serial.print(" G: ");
  Serial.print(whiteGreen);
  Serial.print(" B: ");
  Serial.println(whiteBlue);

  Serial.print("Orange Calibration - R: ");
  Serial.print(orangeRed);
  Serial.print(" G: ");
  Serial.print(orangeGreen);
  Serial.print(" B: ");
  Serial.println(orangeBlue);

  // Done with calibration, now wait for the next step
  Serial.println("Calibration complete. System will perform the task after 2 minutes.");
}

void calibrateColor(int redFilter, int greenFilter, int blueFilter, int &redValue, int &greenValue, int &blueValue) {
  // Set the color filter (S2, S3)
  digitalWrite(S2, redFilter);
  digitalWrite(S3, greenFilter);

  // Measure Red intensity
  redValue = pulseIn(OUT, HIGH);  // Count the pulse width in microseconds

  // Set the color filter for green
  digitalWrite(S2, greenFilter);
  digitalWrite(S3, redFilter);

  // Measure Green intensity
  greenValue = pulseIn(OUT, HIGH);

  // Set the color filter for blue
  digitalWrite(S2, blueFilter);
  digitalWrite(S3, redFilter);

  // Measure Blue intensity
  blueValue = pulseIn(OUT, HIGH);
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

  // Print the retrieved calibration data
  Serial.println("Using calibrated values:");
  Serial.print("White Color - R: ");
  Serial.print(whiteRed);
  Serial.print(" G: ");
  Serial.print(whiteGreen);
  Serial.print(" B: ");
  Serial.println(whiteBlue);

  Serial.print("Orange Color - R: ");
  Serial.print(orangeRed);
  Serial.print(" G: ");
  Serial.print(orangeGreen);
  Serial.print(" B: ");
  Serial.println(orangeBlue);

  // Perform the task using the calibrated data, for example, detect and act based on color
  // Add your task implementation here, like moving motors, triggering actions, etc.
}
