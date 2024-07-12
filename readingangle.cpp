#include <Arduino.h>
#include <ESP32Encoder.h>

// Define the pins for the encoder signals
const int encoderPinA = 18;
const int encoderPinB = 19;

// Create an ESP32Encoder object
ESP32Encoder encoder;

// Number of pulses per revolution of the encoder
const int pulsesPerRevolution = 1000;

void setup() {
  Serial.begin(9600);

  // Set up the encoder pins
  encoder.attachHalfQuad(encoderPinA, encoderPinB);
}

void loop() {
  // Read the encoder position
  long position = encoder.getCount();

  // Convert position to degrees
  float degrees = (position * 360.0) / pulsesPerRevolution;

  // Print the position in degrees to the serial monitor
  Serial.print("Position (degrees): ");
  Serial.println(degrees);

  delay(100);
}