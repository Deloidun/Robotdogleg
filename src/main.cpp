#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <Wire.h>
#include <MPU6050.h>

int i, j;
float random_num;
int numberPos = 0; // this will be changed with MATLAB
int rows = 3;
bool isArrayPopulated = false;
const int toler = 5;
int pGain = 10.5;
// Constants for PID control for each motor
const double Kp1 = pGain;  // Proportional gain for motor 1
const double Ki1 = 0.0;  // Integral gain for motor 1
const double Kd1 = 0.1;  // Derivative gain for motor 1

const double Kp2 = pGain;  // Proportional gain for motor 2
const double Ki2 = 0.0;  // Integral gain for motor 2
const double Kd2 = 0.1;  // Derivative gain for motor 2

const double Kp3 = pGain;  // Proportional gain for motor 3
const double Ki3 = 0.0;  // Integral gain for motor 3
const double Kd3 = 0.1;  // Derivative gain for motor 3

// Variables for PID control
double setpoint1 = 0.0, setpoint2 = 0.0, setpoint3 = 0.0;
double currentAngle1 = 0.0, currentAngle2 = 0.0, currentAngle3 = 0.0;
double motorSpeed1 = 0.0, motorSpeed2 = 0.0, motorSpeed3 = 0.0;

// PID controllers
PID pid1(&currentAngle1, &motorSpeed1, &setpoint1, Kp1, Ki1, Kd1, DIRECT);
PID pid2(&currentAngle2, &motorSpeed2, &setpoint2, Kp2, Ki2, Kd2, DIRECT);
PID pid3(&currentAngle3, &motorSpeed3, &setpoint3, Kp3, Ki3, Kd3, DIRECT);

// Pins for encoder channels
#define ENCODER_A_PIN1 39
#define ENCODER_B_PIN1 36
#define ENCODER_A_PIN2 32
#define ENCODER_B_PIN2 33
#define ENCODER_A_PIN3 35
#define ENCODER_B_PIN3 34

// Pins for motor control
#define MOTOR_PIN1_1 2
#define MOTOR_PIN1_2 4
#define MOTOR_PIN2_1 27
#define MOTOR_PIN2_2 13
#define MOTOR_PIN3_1 19
#define MOTOR_PIN3_2 18

ESP32Encoder encoder1, encoder2, encoder3;

// Encoder resolution (steps per revolution)
const int stepsPerRevolution =  11 * 30;  // Replace with your encoder's resolution

float storepos[3][300];

volatile int encoder1Count = 0;
volatile int encoder2Count = 0;
volatile int encoder3Count = 0;
volatile bool encoder1Flag = false;
volatile bool encoder2Flag = false;
volatile bool encoder3Flag = false;

// Interrupt service routines for encoder channels
void IRAM_ATTR encoderA_ISR1() {
  if (digitalRead(ENCODER_B_PIN1) == HIGH) {
    encoder1Count++;
  } else {
    encoder1Count--;
  }
  encoder1Flag = true;
}

void IRAM_ATTR encoderA_ISR2() {
  if (digitalRead(ENCODER_B_PIN2) == HIGH) {
    encoder2Count++;
  } else {
    encoder2Count--;
  }
  encoder2Flag = true;
}

void IRAM_ATTR encoderA_ISR3() {
  if (digitalRead(ENCODER_B_PIN3) == HIGH) {
    encoder3Count++;
  } else {
    encoder3Count--;
  }
  encoder3Flag = true;
}

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Initialize encoders
  encoder1.attachHalfQuad(ENCODER_A_PIN1, ENCODER_B_PIN1);
  encoder2.attachHalfQuad(ENCODER_A_PIN2, ENCODER_B_PIN2);
  encoder3.attachHalfQuad(ENCODER_A_PIN3, ENCODER_B_PIN3);

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
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN1), encoderA_ISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN2), encoderA_ISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN3), encoderA_ISR3, RISING);

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

std::vector<float> splitStringToFloats(const std::string& input, const std::string& delimiter) {
  std::vector<float> result;
  std::istringstream iss(input);
  std::string token;

  // Parse the input string
  while (getline(iss, token, ',')) {
    // Remove any leading or trailing spaces
    size_t start = token.find_first_not_of(' ');
    size_t end = token.find_last_not_of(' ');
    if (start != std::string::npos && end != std::string::npos) {
      token = token.substr(start, end - start + 1);
    }

    // Convert the string token to float
    float value = std::stof(token);
    result.push_back(value);
  }

  return result;
}

void loop() {
  // Update the current angles
  currentAngle1 = (encoder1Count * 360.0) / stepsPerRevolution;
  currentAngle2 = (encoder2Count * 360.0) / stepsPerRevolution;
  currentAngle3 = (encoder3Count * 360.0) / stepsPerRevolution;

 // Update PID inputs
  pid1.Compute();
  pid2.Compute();
  pid3.Compute();

  // Set the motor speeds using PWM
  setMotorSpeed(MOTOR_PIN1_1, MOTOR_PIN1_2, constrain(motorSpeed1, -255, 255));
  setMotorSpeed(MOTOR_PIN2_1, MOTOR_PIN2_2, constrain(motorSpeed2, -255, 255));
  setMotorSpeed(MOTOR_PIN3_1, MOTOR_PIN3_2, constrain(motorSpeed3, -255, 255));
 
  
  // Debugging: Print current angles
  Serial.print("\n");
  Serial.print("Motor1: " + String(setpoint1) + " | Motor2: " + String(setpoint2) + " | Motor3: " + String(setpoint3));
  Serial.print(" | Angle1: ");
  Serial.print(String(currentAngle1));
  Serial.print(" | Angle2: ");
  Serial.print(String(currentAngle2));
  Serial.print(" | Angle3: ");
  Serial.print(String(currentAngle3));
  Serial.print(" | Pitch: ");
  Serial.print(String(1));
  Serial.print(" | Roll: ");
  Serial.print(String(1.5));


  // Array to store positions
  
  // Initialize the array
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < 300; j++) {
      storepos[i][j] = 0;
    }
  }

  // Read new positions from Serial if available
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    std::vector<float> floats = splitStringToFloats(data.c_str(), ", ");

    for (int i = 0; i < floats.size() - 1; i++) {
      if (i % 3 == 0) {
        storepos[0][i / 3] = floats[i];
        storepos[1][i / 3] = floats[i + 1];
        storepos[2][i / 3] = floats[i + 2];
        numberPos = floats[i + 3];
      }
    }
    
    for (int i = 0; i < numberPos; i++) {
        setpoint1 = storepos[0][i];
        setpoint2 = storepos[1][i];
        setpoint3 = storepos[2][i];

        // Slow down the motors for the last 10 positions
        if (i >= numberPos - 10) {
            pid1.SetOutputLimits(-100, 100);
            pid2.SetOutputLimits(-100, 100);
            pid3.SetOutputLimits(-100, 100);
        } else {
            pid1.SetOutputLimits(-255, 255);
            pid2.SetOutputLimits(-255, 255);
            pid3.SetOutputLimits(-255, 255);
        }
        
    for (int i = 0; i < numberPos; i++) {
        setpoint1 = storepos[0][i];
        setpoint2 = storepos[1][i];
        setpoint3 = storepos[2][i];

    while ( !(setpoint1 - toler <= currentAngle1 && currentAngle1 <= setpoint1 + toler) ||
            !(setpoint2 - toler <= currentAngle2 && currentAngle2 <= setpoint2 + toler) ||
            !(setpoint3 - toler <= currentAngle3 && currentAngle3 <= setpoint3 + toler)) {

        currentAngle1 = (encoder1Count * 360.0) / stepsPerRevolution;
        currentAngle2 = (encoder2Count * 360.0) / stepsPerRevolution;
        currentAngle3 = (encoder3Count * 360.0) / stepsPerRevolution;

        // Update PID inputs
        pid1.Compute();
        pid2.Compute();
        pid3.Compute();

        // Set the motor speeds using PWM
        setMotorSpeed(MOTOR_PIN1_1, MOTOR_PIN1_2, constrain(motorSpeed1, -255, 255));
        setMotorSpeed(MOTOR_PIN2_1, MOTOR_PIN2_2, constrain(motorSpeed2, -255, 255));
        setMotorSpeed(MOTOR_PIN3_1, MOTOR_PIN3_2, constrain(motorSpeed3, -255, 255));

        // Debugging: Print setpoints and current angles
        Serial.print("\n");
        Serial.print("Motor1: " + String(setpoint1) + " | Motor2: " + String(setpoint2) + " | Motor3: " + String(setpoint3));
        Serial.print(" | Angle1: ");
        Serial.print(String(currentAngle1));
        Serial.print(" | Angle2: ");
        Serial.print(String(currentAngle2));
        Serial.print(" | Angle3: ");
        Serial.print(String(currentAngle3));
        Serial.print(" | Pitch: ");
        Serial.print(String(1));
        Serial.print(" | Roll: ");
        Serial.print(String(1.5));
        }
      }
    }
  }
}
