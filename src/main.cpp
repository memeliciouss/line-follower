#include <Arduino.h>

// Motor and Sensor Pin defines
#define ENC_A1 2
#define ENC_B1 3
#define M1_IN1 4
#define M1_IN2 5
#define ENC_A2 18
#define ENC_B2 19
#define M2_IN1 6
#define M2_IN2 7
#define IR_LEFT A0
#define IR_CENTER A1
#define IR_RIGHT A2

// Set OCR1A to 125 (2ms*16MHz/256) for interrupts at every 2 mili seconds
int comp_match = 125;  

//to time motors
int motorFlagCount = 0;

// Base speed
int baseSpeed = 100;

// encoder motors
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
float prevT=0;

// Line following variables
volatile float error = 0;
volatile int leftSpeed = 100;
volatile int rightSpeed = 100;

// PID constants
const float LINE_KP = 2.0;
const float LINE_KI = 0.1;
const float LINE_KD = 1.0;

float lastError = 0;
float errorIntegral = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(ENC_A1, INPUT);
  pinMode(ENC_B1, INPUT);
  pinMode(ENC_A2, INPUT);
  pinMode(ENC_B2, INPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT, INPUT);
  
  // Encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A1), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), readRightEncoder, RISING);
  
  TCCR1A = 0;
  TCCR1B = 0;
  
  // Set prescaler to 256
  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);
  
  // Set OCR1A for output compare match
  OCR1A = comp_match;
  
  // Enable Timer1 compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();
}

void loop() {
  
}

// Interrupt Service Routine
ISR(TIMER1_COMPA_vect) {

  // Read IR Sensors
  int leftSensor = analogRead(IR_LEFT);
  int centerSensor = analogRead(IR_CENTER);
  int rightSensor = analogRead(IR_RIGHT);
  
  //call motor setup
  if (motorFlagCount == 4) {
    SetMotor(leftSensor, centerSensor, rightSensor);
	motorFlagCount=0;
} else {
    motorFlagCount++;
}

}

void SetMotor(int leftSensor, int centerSensor, int rightSensor){
	//calculate error
	error = calculateLineError(leftSensor, centerSensor, rightSensor);
  
  // PID
  float deltaTime = 0.01; // 100 Hz
  
  // Integral term
  errorIntegral += error * deltaTime;
  
  // Derivative term
  float errorDerivative = (error - lastError) / deltaTime;
  
  // Calculate PID output
  float pidOutput = (LINE_KP * error) + 
                    (LINE_KI * errorIntegral) + 
                    (LINE_KD * errorDerivative);
  
  // Calculate motor speeds
  leftSpeed = constrain(baseSpeed + pidOutput, 0, 255);
  rightSpeed = constrain(baseSpeed - pidOutput, 0, 255);
  
  setMotorSpeed(M1_IN1, M1_IN2, leftSpeed);
  setMotorSpeed(M2_IN1, M2_IN2, rightSpeed);
  
  // Update last error
  lastError = error;
}

// Encoder reading functions
void readLeftEncoder() {
  int b = digitalRead(ENC_B1);
  leftEncoderCount += (b > 0) ? 1 : -1;
  long currT=micros();
  float deltaT=((float)(currT-prevT))/1.0e6;
  float speed=((b > 0) ? 1 : -1)/deltaT;
  prevT=currT;
}

void readRightEncoder() {
  int b = digitalRead(ENC_B2);
  rightEncoderCount += (b > 0) ? 1 : -1;
}

// Setting motor speed
void setMotorSpeed(int pwmPin, int dirPin, int speed) {
  digitalWrite(dirPin, (speed >= 0) ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}

// Line error calculation function
float calculateLineError(int leftSensor, int centerSensor, int rightSensor) {
  //straight
  if (centerSensor == HIGH && leftSensor == LOW && rightSensor == LOW) {
    return 0; // Go straight
  }
  
  //left side
  if (leftSensor == HIGH && rightSensor == LOW) {
    return -1.0; // Turn left
  }
  
  // right side
  if (rightSensor == HIGH && leftSensor == LOW) {
    return 1.0; // Turn right
  }

  return 1.0; // Default to turn right
}