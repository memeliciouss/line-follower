#include <Arduino.h>

// Motor 1 Pins (Left Motor)
#define ENC_A1 2   // Encoder A for Left Motor
#define ENC_B1 3   // Encoder B for Left Motor
#define M1_IN1 4   // Left Motor PWM pin
#define M1_IN2 5   // Left Motor Direction pin

// Motor 2 Pins (Right Motor)
#define ENC_A2 18  // Encoder A for Right Motor
#define ENC_B2 19  // Encoder B for Right Motor
#define M2_IN1 6   // Right Motor PWM pin
#define M2_IN2 7   // Right Motor Direction pin

// IR Sensor Pins
#define IR_LEFT A0   // Left IR sensor
#define IR_CENTER A1 // Center IR sensor
#define IR_RIGHT A2  // Right IR sensor

// Volatile variables for encoder counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// PID control structure for motors
struct MotorPID {
  long prevTime = 0;
  float ePrev = 0;
  float eInt = 0;
  float kp = 1.0;
  float ki = 0.1;
  float kd = 0.01;
};

// PID structures for each motor
MotorPID leftMotorPID;
MotorPID rightMotorPID;

// Line following PID constants
const float LINE_KP = 2.0;
const float LINE_KI = 0.1;
const float LINE_KD = 1.0;

// Sensor threshold and variables
const int IR_THRESHOLD = 500;
float lastError = 0;
float errorIntegral = 0;

void setup() {
  Serial.begin(9600);
  
  // Motor Encoder Pin Setup
  pinMode(ENC_A1, INPUT);
  pinMode(ENC_B1, INPUT);
  pinMode(ENC_A2, INPUT);
  pinMode(ENC_B2, INPUT);
  
  // IR Sensor Pin Setup
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT, INPUT);
  
  // Encoder Interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A1), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), readRightEncoder, RISING);
  
  Serial.println("Encoder Line Follower Initialized");
}

void loop() {
  // Read IR Sensor Values
  int leftSensor = analogRead(IR_LEFT);
  int centerSensor = analogRead(IR_CENTER);
  int rightSensor = analogRead(IR_RIGHT);
  
  // Calculate line position error
  float error = calculateLineError(leftSensor, centerSensor, rightSensor);
  
  // PID Line Following Calculation
  float deltaTime = 0.01; // Fixed time step (100Hz)
  
  // Integral term
  errorIntegral += error * deltaTime;
  
  // Derivative term
  float errorDerivative = (error - lastError) / deltaTime;
  
  // Compute PID output
  float pidOutput = (LINE_KP * error) + 
                    (LINE_KI * errorIntegral) + 
                    (LINE_KD * errorDerivative);
  
  // Base speed
  int baseSpeed = 100; // Adjust as needed
  
  // Calculate motor speeds
  int leftSpeed = constrain(baseSpeed + pidOutput, 0, 255);
  int rightSpeed = constrain(baseSpeed - pidOutput, 0, 255);
  
  // Set motor directions and speeds
  setMotorSpeed(M1_IN1, M1_IN2, leftSpeed);
  setMotorSpeed(M2_IN1, M2_IN2, rightSpeed);
  
  // Debug output
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);
  
  // Update last error
  lastError = error;
  
  delay(10); // Small delay for stability
}

// Calculate line position error
float calculateLineError(int leftSensor, int centerSensor, int rightSensor) {
  // More advanced error calculation
  if (leftSensor > IR_THRESHOLD && centerSensor > IR_THRESHOLD && rightSensor > IR_THRESHOLD) {
    // All sensors on line - go straight
    return 0;
  }
  
  // Weighted error calculation
  if (leftSensor > IR_THRESHOLD && rightSensor < IR_THRESHOLD) {
    return -1.0; // Turn left
  }
  
  if (rightSensor > IR_THRESHOLD && leftSensor < IR_THRESHOLD) {
    return 1.0; // Turn right
  }
  
  // If center sensor is different, use it for more precise positioning
  if (centerSensor > IR_THRESHOLD) {
    // Slight bias based on left and right sensors
    return (leftSensor - rightSensor) * 0.5;
  }
  
  // If no line detected
  return 0;
}

// Motor encoder reading for Left Motor
void readLeftEncoder() {
  int b = digitalRead(ENC_B1);
  leftEncoderCount += (b > 0) ? 1 : -1;
}

// Motor encoder reading for Right Motor
void readRightEncoder() {
  int b = digitalRead(ENC_B2);
  rightEncoderCount += (b > 0) ? 1 : -1;
}

// Set motor speed with direction
void setMotorSpeed(int pwmPin, int dirPin, int speed) {
  // Determine direction
  digitalWrite(dirPin, (speed >= 0) ? HIGH : LOW);
  
  // Write absolute speed
  analogWrite(pwmPin, abs(speed));
}

// Optional: Emergency stop function
void stopMotors() {
  analogWrite(M1_IN1, 0);
  analogWrite(M2_IN1, 0);
}