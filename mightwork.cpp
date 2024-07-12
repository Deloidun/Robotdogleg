#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>

// Constants for PID control for each motor
const double Kp1 = 6;  // Proportional gain for motor 1
const double Ki1 = 0.0;  // Integral gain for motor 1
const double Kd1 = 0;  // Derivative gain for motor 1

const double Kp2 = 6;  // Proportional gain for motor 2
const double Ki2 = 2;  // Integral gain for motor 2
const double Kd2 = 0.2;  // Derivative gain for motor 2

const double Kp3 = 10;  // Proportional gain for motor 3
const double Ki3 = 0.0;  // Integral gain for motor 3
const double Kd3 = 0;  // Derivative gain for motor 3

// Variables for PID control
double setpoint1 = 0.0, setpoint2 = 0.0, setpoint3 = 0.0;
double currentAngle1 = 0.0, currentAngle2 = 0.0, currentAngle3 = 0.0;
double motorSpeed1 = 0.0, motorSpeed2 = 0.0, motorSpeed3 = 0.0;
double prevError1 = 0.0, prevError2 = 0.0, prevError3 = 0.0;
double integral1 = 0.0, integral2 = 0.0, integral3 = 0.0;

// PID controllers
PID pid1(&currentAngle1, &motorSpeed1, &setpoint1, Kp1, Ki1, Kd1, DIRECT);
PID pid2(&currentAngle2, &motorSpeed2, &setpoint2, Kp2, Ki2, Kd2, DIRECT);
PID pid3(&currentAngle3, &motorSpeed3, &setpoint3, Kp3, Ki3, Kd3, DIRECT);

// Pins for encoder channels
#define ENCODER_A_PIN1 36
#define ENCODER_B_PIN1 39
#define ENCODER_A_PIN2 32
#define ENCODER_B_PIN2 33
#define ENCODER_A_PIN3 34
#define ENCODER_B_PIN3 35

// Pins for motor control
#define MOTOR_PIN1_1 4
#define MOTOR_PIN1_2 2
#define MOTOR_PIN2_1 27
#define MOTOR_PIN2_2 13
#define MOTOR_PIN3_1 18
#define MOTOR_PIN3_2 19

ESP32Encoder encoder1, encoder2, encoder3;

// Encoder resolution (steps per revolution)
const int stepsPerRevolution = 44*30;  // Replace with your encoder's resolution







// Interrupt service routines for encoder channels
void IRAM_ATTR encoderA_ISR1() {
  if (digitalRead(ENCODER_B_PIN1) == HIGH) {
    encoder1.setCount(encoder1.getCount() + 1);
  } else {
    encoder1.setCount(encoder1.getCount() - 1);
  }
}

void IRAM_ATTR encoderA_ISR2() {
  if (digitalRead(ENCODER_B_PIN2) == HIGH) {
    encoder2.setCount(encoder2.getCount() + 1);
  } else {
    encoder2.setCount(encoder2.getCount() - 1);
  }
}

void IRAM_ATTR encoderA_ISR3() {
  if (digitalRead(ENCODER_B_PIN3) == HIGH) {
    encoder3.setCount(encoder3.getCount() + 1);
  } else {
    encoder3.setCount(encoder3.getCount() - 1);
  }
}

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

  pinMode(ENCODER_A_PIN1, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN1, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN2, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN2, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN3, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN3, INPUT_PULLUP);

  // Attach interrupts for encoder channels
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN1), encoderA_ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN2), encoderA_ISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN3), encoderA_ISR3, CHANGE);

  // Set PID parameters
  pid1.SetMode(AUTOMATIC);
  pid1.SetSampleTime(10);
  pid1.SetOutputLimits(-255, 255);

  pid2.SetMode(AUTOMATIC);
  pid2.SetSampleTime(10);
  pid2.SetOutputLimits(-255, 255);

  pid3.SetMode(AUTOMATIC);
  pid3.SetSampleTime(10);
  pid3.SetOutputLimits(-255, 255);
}


void setMotorSpeed(int pin1, int pin2, double speed) {
  if (speed > 0) {
    analogWrite(pin1, speed);
    analogWrite(pin2, 0);
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, -speed);
  }
}


void loop() {
  if (Serial.available() == 1) {
    // Read desired angles from Serial input
    setpoint1 = Serial.parseFloat();
    setpoint2 = Serial.parseFloat();
    setpoint3 = Serial.parseFloat();
  }

  // Calculate current angles from encoders
  currentAngle1 = (encoder1.getCount()  * 360.0) / stepsPerRevolution;
  currentAngle2 = (encoder2.getCount()  * 360.0) / stepsPerRevolution;
  currentAngle3 = (encoder3.getCount()  * 360.0) / stepsPerRevolution;

  // // Normalize encoder count
  // encoder1.setCount(encoder1.getCount() % stepsPerRevolution);
  // encoder2.setCount(encoder2.getCount() % stepsPerRevolution);
  // encoder3.setCount(encoder3.getCount() % stepsPerRevolution);

  // Update PID inputs
  pid1.Compute();
  pid2.Compute();
  pid3.Compute();

  // Set the motor speeds using PWM
  setMotorSpeed(MOTOR_PIN1_1, MOTOR_PIN1_2, constrain(motorSpeed1, -255, 255));
  setMotorSpeed(MOTOR_PIN2_1, MOTOR_PIN2_2, constrain(motorSpeed2, -255, 255));
  setMotorSpeed(MOTOR_PIN3_1, MOTOR_PIN3_2, constrain(motorSpeed3, -255, 255));

  // Display current angles and setpoints
  Serial.print("Angle1: ");
  Serial.print(currentAngle1);
  Serial.print(" degrees, Setpoint1: ");
  Serial.print(setpoint1);
  Serial.print(" | Angle2: ");
  Serial.print(currentAngle2);
  Serial.print(" degrees, Setpoint2: ");
  Serial.print(setpoint2);
  Serial.print(" | Angle3: ");
  Serial.print(currentAngle3);
  Serial.print(" degrees, Setpoint3: ");
  Serial.println(setpoint3);
}

