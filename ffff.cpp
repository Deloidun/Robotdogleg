#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PIDControl.h>

// Constants for PID control
const double Kp = 25;  // Proportional gain
const double Ki = 0.0;  // Integral gain
const double Kd = 200;  // Derivative gain

// Pins for encoder channels
#define ENCODER_A_PIN 18
#define ENCODER_B_PIN 19

// Pins for motor control
#define MOTOR_PIN_1 33
#define MOTOR_PIN_2 32

ESP32Encoder encoder;
PIDController pidController(Kp, Ki, Kd);

// Encoder resolution (steps per revolution)
const int stepsPerRevolution = 22 * 30 * 2;  // Replace with your encoder's resolution

volatile int32_t currentPosition = 0;

void IRAM_ATTR encoderISR() {
  currentPosition += encoder.getCount();
}

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Initialize encoder
  encoder.attachFullQuad(ENCODER_A_PIN, ENCODER_B_PIN);
  
  // Attach interrupt service routine for the encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderISR, CHANGE);

  // Initialize motor control pins
  pinMode(MOTOR_PIN_1, OUTPUT);
  pinMode(MOTOR_PIN_2, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    // Prompt user to input desired angle
    Serial.println("Enter desired angle:");
    double setpoint = Serial.parseFloat();  // Read the user input

    pidController.setSetpoint(setpoint);
  }

  // Calculate the angle based on the encoder count
  double currentAngle = (double)currentPosition * 360.0 / stepsPerRevolution;

  double motorSpeed = pidController.compute(currentAngle);


  Serial.print("Angle: ");
  Serial.print(currentAngle);
  Serial.print(", Speed: ");
  Serial.println(motorSpeed);

  // Set the motor speed using PWM
  analogWrite(MOTOR_PIN_1, abs(motorSpeed));
  analogWrite(MOTOR_PIN_2, 0);
  digitalWrite(MOTOR_PIN_1, motorSpeed >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_PIN_2, motorSpeed >= 0 ? LOW : HIGH);
  
  Serial.print("Encoder count: ");
  Serial.println(currentPosition);
  Serial.print("PID Output: ");
  Serial.println(motorSpeed);
  Serial.println("-------------");

}