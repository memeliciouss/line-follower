#include <Arduino.h>

// Motor and Sensor Pin defines

// Left motor
#define ENC_A1 3
#define ENC_B1 12
#define M1_IN1 4
#define M1_IN2 7

// Right motor
#define ENC_A2 2
#define ENC_B2 11
#define M2_IN1 6
#define M2_IN2 5

#define IR_LEFT A2
#define IR_CENTER A0
#define IR_RIGHT A1

// Set threshold value for IR sensors
int threshold = 512;
// Set OCR1A to 125 (2ms*16MHz/256) for interrupts at every 2 mili seconds
int comp_match = 125;

// to time motors
int motorFlagCount = 0;

// Base speed
int baseSpeed = 100;

// encoder motors
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
float prevT = 0;

volatile float left_speed = 0;
volatile float right_speed = 0;
float left_targetSpeed = 0;
float right_targetSpeed = 0;

long left_prevT = 0;
long right_prevT = 0;

// Line following PID variables
volatile float error_line = 0;
float errorIntegral_line = 0;
float prevError_line = 0;

// Speed PID variables
volatile float error_speed = 0;
float errorIntegral_speed = 0;
float prevError_speed = 0;

// PID constants
const float Kp_line = 2.0;
const float Ki_line = 0.1;
const float Kd_line = 1.0;

const float Kp_speed = 0.8;
const float Ki_speed = 0.05;
const float Kd_speed = 0.1;


void setup()
{

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

  // Reset timer control register A
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

  // Enable global interrupts
  sei();
}

void loop()
{
  // empty loop
}

// Interrupt Service Routine
ISR(TIMER1_COMPA_vect)
{

  // Read IR Sensors
  int leftSensor = analogRead(IR_LEFT);
  int centerSensor = analogRead(IR_CENTER);
  int rightSensor = analogRead(IR_RIGHT);

  // call motor setup
  if (motorFlagCount == 4)
  {
    SetMotor(leftSensor, centerSensor, rightSensor);
    motorFlagCount = 0;
  }
  else
  {
    motorFlagCount++;
  }
}

// Set Motors 
void SetMotor(int leftSensor, int centerSensor, int rightSensor)
{

  // calculate error
  error_line = calculateLineError(leftSensor, centerSensor, rightSensor);
  float deltaTime = 0.01; // 100 Hz

  // Integral term
  errorIntegral_line += error_line * deltaTime;

  // Derivative term
  float errorDerivative_line = (error_line - prevError_line) / deltaTime;

  // Calculate PID output
  float pidOutput = (Kp_line * error_line) +
                    (Ki_line * errorIntegral_line) +
                    (Kd_line * errorDerivative_line);

  // Calculate motor speeds
  left_targetSpeed = constrain(baseSpeed + pidOutput, 0, 255);
  right_targetSpeed = constrain(baseSpeed - pidOutput, 0, 255);

  // Compute req speed in terms of PWM
  int left_pwm = computeSpeedPID(left_targetSpeed, left_speed);
  int right_pwm = computeSpeedPID(right_targetSpeed, right_speed);

  // Set Motors
  setMotorSpeed(M1_IN1, M1_IN2, left_pwm);
  setMotorSpeed(M2_IN1, M2_IN2, right_pwm);
  
  // Update last error
  prevError_line = error_line;
}

// Encoder reading functions
void readLeftEncoder()
{
  int b = digitalRead(ENC_B1);
  leftEncoderCount += (b > 0) ? 1 : -1;
  long currT = micros();
  float deltaT = ((float)(currT - left_prevT)) / 1.0e6;
  left_speed = ((b > 0) ? 1 : -1) / deltaT;
  left_prevT = currT;
}

void readRightEncoder()
{
  int b = digitalRead(ENC_B2);
  rightEncoderCount += (b > 0) ? 1 : -1;
  long currT = micros();
  float deltaT = ((float)(currT - right_prevT)) / 1.0e6;
  right_speed = ((b > 0) ? 1 : -1) / deltaT;
  right_prevT = currT;
}

// Setting motor speed
void setMotorSpeed(int pwmPin, int dirPin, int speed)
{
  digitalWrite(dirPin, (speed >= 0) ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}

// Line error calculation
float calculateLineError(int leftSensor, int centerSensor, int rightSensor)
{
  // straight
  if (centerSensor > threshold && leftSensor < threshold && rightSensor <threshold)
  {
    return 0; // Go straight
  }

  // left side
  if (leftSensor > threshold && rightSensor < threshold)
  {
    return -1.0; // Turn left
  }

  // right side
  if (leftSensor < threshold && rightSensor > threshold)
  {
    return 1.0; // Turn right
  }

  return 1.0; // Default to turn right
}

// Compute PWM from req speed using PID
int computeSpeedPID(float targetSpeed, float currSpeed)
{
  // calculate error
  error_speed = targetSpeed - currSpeed;
  float deltaT = 0.01; // 100 Hz

  // Integral term
  errorIntegral_speed += error_speed * deltaT;

  // Derivative term
  float errorDerivative_speed = (error_speed - prevError_speed) / deltaT;

  // PID output
  float output = (Kp_speed * error_speed) +
                 (Ki_speed * errorIntegral_speed) +
                 (Kd_speed * errorDerivative_speed);


  int pwm = constrain((int)output,-255,255);
  return pwm;
}