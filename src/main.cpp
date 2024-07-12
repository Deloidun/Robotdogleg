#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <time.h>

//std::vector<float> splitStringToFloats(const std::string& input, const std::string& delimiter);


int i, j;
float random_num;
int numberPos = 3; // this will be change with the matlab
int rows = 3;
bool isArrayPopulated = false;

// Constants for PID control for each motor
const double Kp1 = 30;  // Proportional gain for motor 1
const double Ki1 = 0.0;  // Integral gain for motor 1
const double Kd1 = 0.01;  // Derivative gain for motor 1

const double Kp2 = 30;  // Proportional gain for motor 2
const double Ki2 = 0.0;  // Integral gain for motor 2
const double Kd2 = 0.01;  // Derivative gain for motor 2

const double Kp3 = 30;  // Proportional gain for motor 3
const double Ki3 = 0.0;  // Integral gain for motor 3
const double Kd3 = 0.01;  // Derivative gain for motor 3

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

   while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

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
   // Calculate current angles from encoders
  

  //////////////////////////////////////////////////////
  // if (Serial.available() > 0) {
  //   // Read desired angles from Serial input
  //   String data = Serial.readStringUntil('\n');
  //   std::vector<float> floats = splitStringToFloats(data.c_str(), ", ");

  //   // Keep printing tokens while one of the
  //   // delimiters present in str[].
  //   float theta1, theta2, theta3;
  //   if (floats.size() >= 3) {  // Ensure there are at least three elements
  //       theta1 = floats[0];
  //       theta2 = floats[1];
  //       theta3 = floats[2];
  //   }

  //   Serial.print("\n");
  //   Serial.print("Theta1: " + String(theta1) + " Theta2: " + String(theta2) + " Theta3: " + String(theta3));

  //   setpoint1 = theta1;
  //   setpoint2 = theta2;
  //   setpoint3 = theta3;
  // }
  //////////////////////////////////RECEIVE DATA NEED FIX////////////////////

  Serial.println("Enter the number of columns: ");
  if (Serial.available() == 0) {
      ; // Wait for user input
  }
  numberPos = Serial.parseInt();

  // Allocate memory for the original array
  float **arr = (float **)malloc(rows * sizeof(float *));
  for (int i = 0; i < rows; i++) {
      arr[i] = (float *)malloc(numberPos * sizeof(float));
  }


  /////////////////////////////////////////// For testing only////////////////////////////////////
  srand((unsigned)time(NULL));
  for (int i = 0; i < rows; i++) {
      for (int j = 0; j < numberPos; j++) {
          float random_num = (float)(rand() % 1000) / 100.0; // Random float between 0.00 and 24.99
          arr[i][j] = random_num;
      }
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("Original Array:");
  for (int i = 0; i < rows; i++) {
      for (int j = 0; j < numberPos; j++) {
          Serial.printf("%.2f\t", arr[i][j]);
      }
      Serial.println();
  }

  // test only
  // float arr[3][2];
  
  // Proceed with the rest of the program logic
  Serial.println("Array has been populated. Proceeding with the loop.");

  // Allocate memory for the copy array
  float **storepos = (float **)malloc(rows * sizeof(float *));
  for (int i = 0; i < rows; i++) {
      storepos[i] = (float *)malloc(numberPos * sizeof(float));
  }

  
  // init arr2
  for (int i = 0; i < rows; i++) {
      for (int j = 0; j < numberPos; j++) {
          storepos[i][j] = 360;
      }
  }

  // Copy the original array to the new array
  for (int i = 0; i < rows; i++) {
      for (int j = 0; j < numberPos; j++) {
          storepos[i][j] = arr[i][j];
      }
  }

  // Print the copied array
  Serial.println("\nCopy Array:");
  for (int i = 0; i < rows; i++) {
      for (int j = 0; j < numberPos; j++) {
          Serial.printf("%.2f\t", storepos[i][j]);
      }
      Serial.println();
  }

    // Verify that the array has no uninitialized values (NANs)
  bool allValuesInitialized = true;
  for (int i = 0; i < rows; i++) {
      for (int j = 0; j < numberPos; j++) {
          if (storepos[i][j] >= 360) {
              allValuesInitialized = false;
              break;
          }
      }
  }

  // Set the flag if the array is fully populated
  isArrayPopulated = allValuesInitialized;

// Check if the array has been populated
  if (isArrayPopulated) {
    // Print motor values
    for (int i = 0; i < numberPos; i++) {
      //if(currentAngle1 == setpoint1 && currentAngle2 == setpoint2 && currentAngle3 == setpoint3 ){
        setpoint1 = storepos[0][i];
        setpoint2 = storepos[1][i];
        setpoint3 = storepos[2][i];
      //}
      while (currentAngle1 != setpoint1 || currentAngle2 != setpoint2 || currentAngle3 != setpoint3) {
                if (currentAngle1 < setpoint1) currentAngle1 += 0.01;
                else if (currentAngle1 > setpoint1) currentAngle1 -= 0.01;

                if (currentAngle2 < setpoint2) currentAngle2 += 0.01;
                else if (currentAngle2 > setpoint2) currentAngle2 -= 0.01;

                if (currentAngle3 < setpoint3) currentAngle3 += 0.01;
                else if (currentAngle3 > setpoint3) currentAngle3 -= 0.01;
        
                // Update PID inputs
                pid1.Compute();
                pid2.Compute();
                pid3.Compute();

                // Set the motor speeds using PWM
                setMotorSpeed(MOTOR_PIN1_1, MOTOR_PIN1_2, constrain(motorSpeed1, -255, 255));
                setMotorSpeed(MOTOR_PIN2_1, MOTOR_PIN2_2, constrain(motorSpeed2, -255, 255));
                setMotorSpeed(MOTOR_PIN3_1, MOTOR_PIN3_2, constrain(motorSpeed3, -255, 255));

                Serial.printf("currentAngle 1: %.2f\t", currentAngle1);
                Serial.printf("currentAngle 2: %.2f\t", currentAngle2);
                Serial.printf("currentAngle 3: %.2f\n", currentAngle3);

                Serial.printf("setpoint 1: %.2f\t", setpoint1);
                Serial.printf("setpoint 2: %.2f\t", setpoint2);
                Serial.printf("setpoint 3: %.2f\n", setpoint3);

                delay(10); // Simulate time for the motors to reach the target angles
            }
        
    }

    // Free allocated memory
    for (int i = 0; i < rows; i++) {
        free(arr[i]);
    }
    free(arr);


    for (int i = 0; i < rows; i++) {
        free(storepos[i]);
    }
    free(storepos);

    isArrayPopulated = false;
  } else {
      // Wait until the array receives values
      Serial.println("Not populated");
      delay(500);
  }

  
}

