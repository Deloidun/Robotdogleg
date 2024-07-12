#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PIDControl.h>

// Constants for PID control
const double Kp = 25;  // Proportional gain
const double Ki = 0.0;  // Integral gain
const double Kd = 200;  // Derivative gain

// Pins for encoder channels
#define ENCODER_A_PIN1 18
#define ENCODER_B_PIN1 19 
#define ENCODER_A_PIN2 20
#define ENCODER_B_PIN2 21 
#define ENCODER_A_PIN3 22
#define ENCODER_B_PIN3 23 

// Pins for motor control
#define MOTOR_PIN1_1 33
#define MOTOR_PIN1_2 32
#define MOTOR_PIN2_1 25
#define MOTOR_PIN2_2 26
#define MOTOR_PIN3_1 27
#define MOTOR_PIN3_2 28

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;

PIDController pidController1(Kp, Ki, Kd);
PIDController pidController2(Kp, Ki, Kd);
PIDController pidController3(Kp, Ki, Kd);

// Encoder resolution (steps per revolution)
const int stepsPerRevolution = 22*30*2;  // Replace with your encoder's resolution

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Initialize encoders
  encoder1.attachFullQuad(ENCODER_A_PIN1, ENCODER_B_PIN1);
  encoder2.attachFullQuad(ENCODER_A_PIN2, ENCODER_B_PIN2);
  encoder3.attachFullQuad(ENCODER_A_PIN3, ENCODER_B_PIN3);

  // Initialize motor control pins
  pinMode(MOTOR_PIN1_1, OUTPUT);
  pinMode(MOTOR_PIN1_2, OUTPUT);
  pinMode(MOTOR_PIN2_1, OUTPUT);
  pinMode(MOTOR_PIN2_2, OUTPUT);
  pinMode(MOTOR_PIN3_1, OUTPUT);
  pinMode(MOTOR_PIN3_2, OUTPUT);
}

void loop() {
  if (Serial.available() == 1) {
    // Prompt user to input desired angles
    Serial.println("Enter desired angles (separated by spaces):");
    double setpoint1 = Serial.parseFloat();  // Read the user input for motor 1
    double setpoint2 = Serial.parseFloat();  // Read the user input for motor 2
    double setpoint3 = Serial.parseFloat();  // Read the user input for motor 3
    
    pidController1.setSetpoint(setpoint1);
    pidController2.setSetpoint(setpoint2);
    pidController3.setSetpoint(setpoint3);
  }

  int32_t currentPosition1 = encoder1.getCount();
  int32_t currentPosition2 = encoder2.getCount();
  int32_t currentPosition3 = encoder3.getCount();

  // Calculate the angles based on the encoder counts
  double currentAngle1 = (currentPosition1 * 360.0) / (stepsPerRevolution);
  double currentAngle2 = (currentPosition2 * 360.0) / (stepsPerRevolution);
  double currentAngle3 = (currentPosition3 * 360.0) / (stepsPerRevolution);
  double motorSpeed1 = pidController1.compute(currentAngle1);
  double motorSpeed2 = pidController2.compute(currentAngle2);
  double motorSpeed3 = pidController3.compute(currentAngle3);

  Serial.print("Speeds: ");
  Serial.print(motorSpeed1);
  Serial.print(", ");
  Serial.print(motorSpeed2);
  Serial.print(", ");
  Serial.print(motorSpeed3);

  // Set the motor speeds using PWM
  if (motorSpeed1 > 255) {
    motorSpeed1 = 255;
  } else if (motorSpeed1 < -255) {
    motorSpeed1 = -255;
  }

  if (motorSpeed2 > 255) {
    motorSpeed2 = 255;
  } else if (motorSpeed2 < -255) {
    motorSpeed2 = -255;
  }

  if (motorSpeed3 > 255) {
    motorSpeed3 = 255;
  } else if (motorSpeed3 < -255) {
    motorSpeed3 = -255;
  }

  if (motorSpeed1 > 0) {
    analogWrite(MOTOR_PIN1_1, motorSpeed1);
    analogWrite(MOTOR_PIN1_2, 0);
  } else {
    analogWrite(MOTOR_PIN1_1, 0);
    analogWrite(MOTOR_PIN1_2, -motorSpeed1);
  }

  if (motorSpeed2 > 0) {
    analogWrite(MOTOR_PIN2_1, motorSpeed2);
    analogWrite(MOTOR_PIN2_2, 0);
  } else {
    analogWrite(MOTOR_PIN2_1, 0);
    analogWrite(MOTOR_PIN2_2, -motorSpeed2);
  }

  if (motorSpeed3 > 0) {
    analogWrite(MOTOR_PIN3_1, motorSpeed3);
    analogWrite(MOTOR_PIN3_2, 0);
  } else {
    analogWrite(MOTOR_PIN3_1, 0);
    analogWrite(MOTOR_PIN3_2, -motorSpeed3);
  }

  // Display current angles and setpoints
  Serial.print(" Current Angles: ");
  Serial.print(currentAngle1);
  Serial.print(" degrees, ");
  // Serial.print(currentAngle2);
  // Serial.print(" degrees, ");
  // Serial.print(currentAngle3);
  // Serial.println(" degrees.");
  

  delay(10);
}