#include <Arduino.h>
#include <ESP32Encoder.h>

// Constants for PID control
const double Kp = 6;  // Proportional gain
const double Ki = 0.0;  // Integral gain
const double Kd = 0;  // Derivative gain

// Pins for encoder channels
#define ENCODER_A_PIN 36
#define ENCODER_B_PIN 39 

// Pins for motor control
#define MOTOR_PIN1 4
#define MOTOR_PIN2 2

ESP32Encoder encoder;

// Variables for PID control
double setpoint = 0.0;  // Desired angle input from the user
double prevError = 0.0;
double integral = 0.0;

// Encoder resolution (steps per revolution)
const int stepsPerRevolution = 44*30;  // Replace with your encoder's resolution

// Interrupt service routine for encoder channel A
void IRAM_ATTR encoderA_ISR() {
  if (digitalRead(ENCODER_B_PIN) == HIGH) {
    encoder.getCount();
  } else {
    encoder.getCount() -1;
  }
}

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Initialize encoder
  encoder.attachFullQuad(ENCODER_A_PIN, ENCODER_B_PIN);

  // Initialize motor control pins
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);

  // Attach interrupt for encoder channel A
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderA_ISR, CHANGE);
}

void loop() {
  if (Serial.available() == 1) {
    // Prompt user to input desired angle
    Serial.println("Enter desired angle:");
    setpoint = Serial.parseFloat();  // Read the user input
  }

  int32_t currentPosition = encoder.getCount();

  // Calculate the angle based on the encoder count
  double currentAngle = (currentPosition * 360.0) / (stepsPerRevolution);

  // Compute the error (difference between setpoint and current angle)
  double error = setpoint - currentAngle;

  // Compute PID terms
  double proportionalTerm = Kp * error;
  integral += Ki * error;
  double derivativeTerm = Kd * (error - prevError);

  // Compute the desired motor speed
  double motorSpeed = proportionalTerm + integral + derivativeTerm;
  Serial.print("Speed: ");
  Serial.print(motorSpeed);

  // Set the motor speed using PWM
  if (motorSpeed > 255) {
    motorSpeed = 255;
  } else if (motorSpeed < -255) {
    motorSpeed = -255;
  }

  if (motorSpeed > 0) {
    analogWrite(MOTOR_PIN1, motorSpeed);
    analogWrite(MOTOR_PIN2, 0);
  } else {
    analogWrite(MOTOR_PIN1, 0);
    analogWrite(MOTOR_PIN2, -motorSpeed);
  }

  // Update previous error for the next iteration
  prevError = error;

  // Display current angle and setpoint
  Serial.print(" Current Angle: ");
  Serial.print(currentAngle);
  Serial.print(" degrees. Setpoint: ");
  Serial.print(setpoint);
  Serial.println(" degrees.");

  delay(10);
}