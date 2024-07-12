#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PIDcontrol.h>

// Motor configuration
struct Motor {
  int encoderAPin;
  int encoderBPin;
  int motorPin1;
  int motorPin2;
  ESP32Encoder encoder;
  PIDControl pidControl;

  // Constructor
  Motor(int encoderAPin, int encoderBPin, int motorPin1, int motorPin2, double Kp, double Ki, double Kd)
      : encoderAPin(encoderAPin),
        encoderBPin(encoderBPin),
        motorPin1(motorPin1),
        motorPin2(motorPin2),
        encoder(),
        pidControl(Kp, Ki, Kd){
  }
};


// Motor objects
Motor motor1(0, 0, 0, 0, 0.0, 0.0, 0.0);
Motor motor2(0, 0, 0, 0, 0.0, 0.0, 0.0);
Motor motor3(0, 0, 0, 0, 0.0, 0.0, 0.0);

unsigned long time_prev = 0; // Variable used for serial monitoring
const int stepsPerRevolution = 44 * 30; 
double setpoint1;
double setpoint2;
double setpoint3;
// Interrupt service routine for encoder channel A
void IRAM_ATTR encoderA_ISR(Motor &motor) {
  if (digitalRead(motor.encoderBPin) == HIGH) {
    motor.encoder.setCount(motor.encoder.getCount()  );  
  } else {
    motor.encoder.setCount(motor.encoder.getCount() - 1);  // Decrement count by 1
  }
}

// Function to set motor speed based on PID control
double setMotorSpeed(Motor &motor) {
  int32_t currentPosition = motor.encoder.getCount();

  // Calculate the angle based on the encoder count
  double currentAngle = (currentPosition * 360.0) / stepsPerRevolution;

  // Compute the desired motor speed using PID control
  double motorSpeed = motor.pidControl.compute(currentAngle);

  // Set the motor speed using PWM
  if (motorSpeed > 255) {
    motorSpeed = 255;
  } else if (motorSpeed < -255) {
    motorSpeed = -255;
  }

  if (motorSpeed > 0) {
    analogWrite(motor.motorPin1, motorSpeed);
    analogWrite(motor.motorPin2, 0);
  } else {
    analogWrite(motor.motorPin1, 0);
    analogWrite(motor.motorPin2, -motorSpeed);
  }

  return currentAngle;
}

void motor1_ISR() {
  encoderA_ISR(motor1);
}

void motor2_ISR() {
  encoderA_ISR(motor2);
}

void motor3_ISR() {
  encoderA_ISR(motor3);
}

void SerialDataPrint(Motor &motor, double currentAngle, double setpoint)
{
  if (micros() - time_prev >= 20000)
  {
    time_prev = micros();
    Serial.print("Motor: ");
    Serial.print(motor.motorPin1);
    Serial.print(" Current Angle: ");
    Serial.print(currentAngle);
    Serial.print(" degrees. Setpoint: ");
    Serial.print(setpoint);
    Serial.println(" degrees.");
  }
}

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Motor 1 configuration
  motor1.encoderAPin = 36;
  motor1.encoderBPin = 39;
  motor1.motorPin1 = 4;
  motor1.motorPin2 = 2;
  motor1.encoder.attachFullQuad(motor1.encoderAPin, motor1.encoderBPin);
  motor1.pidControl = PIDControl(3, 0, 0.0);  // Create an instance of PIDControl
  motor1.encoder.setCount(0);

  // Motor 2 configuration
  motor2.encoderAPin = 32;
  motor2.encoderBPin = 33;
  motor2.motorPin1 = 27;
  motor2.motorPin2 = 13;
  motor2.encoder.attachFullQuad(motor2.encoderAPin, motor2.encoderBPin);
  motor2.pidControl = PIDControl(3, 0, 0.0);  // Create an instance of PIDControl
  motor2.encoder.setCount(0);

  // Motor 3 configuration
  motor3.encoderAPin = 26;
  motor3.encoderBPin = 27;
  motor3.motorPin1 = 13;
  motor3.motorPin2 = 12;  // Corrected motorPin2 value
  motor3.encoder.attachFullQuad(motor3.encoderAPin, motor3.encoderBPin);
  motor3.pidControl = PIDControl(3, 0, 0.0);  // Create an instance of PIDControl
  motor3.encoder.setCount(0);

  //Set encoder pins as INPUT

  pinMode(motor1.encoderAPin, INPUT);
  pinMode(motor1.encoderBPin, INPUT);
  pinMode(motor2.encoderAPin, INPUT);
  pinMode(motor2.encoderBPin, INPUT);
  pinMode(motor3.encoderAPin, INPUT);
  pinMode(motor3.encoderBPin, INPUT);

  // Set motor pins as OUTPUT
  pinMode(motor1.motorPin1, OUTPUT);
  pinMode(motor1.motorPin2, OUTPUT);
  pinMode(motor2.motorPin1, OUTPUT);
  pinMode(motor2.motorPin2, OUTPUT);
  pinMode(motor3.motorPin1, OUTPUT);
  pinMode(motor3.motorPin2, OUTPUT);

  // Attach interrupts for encoder channel A
  attachInterrupt(digitalPinToInterrupt(motor1.encoderAPin), motor1_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor2.encoderAPin), motor2_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motor3.encoderAPin), motor3_ISR, CHANGE);
}

void loop() {
  
    if (Serial.available() == 1) {
    // Prompt user to input desired angles
    Serial.println("Enter desired angles (separated by spaces):");
    setpoint1 = Serial.parseFloat();  // Read the user input for motor 1
    setpoint2 = Serial.parseFloat();  // Read the user input for motor 2
    setpoint3 = Serial.parseFloat();  // Read the user input for motor 3
    
    motor1.pidControl.setSetpoint(setpoint1);
    motor2.pidControl.setSetpoint(setpoint2);
    motor3.pidControl.setSetpoint(setpoint3);
  }

  setMotorSpeed(motor1);
  setMotorSpeed(motor2);
  setMotorSpeed(motor3);
  SerialDataPrint(motor1, setMotorSpeed(motor1),setpoint1);
  
 
}