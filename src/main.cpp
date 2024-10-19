#include <Arduino.h>

#define encA 2 //encoder readings from motor
#define encB 3 //encoder readings from motor
#define motor1 4 //motor pin M1
#define motor2 5 //motor pin M2
#define pwmPin 6 //motor driver PWM input

volatile int count=0;

//for error calculations
long prevTime=0;
float ePrev=0;
float eInt=0;

//PID constants
float kp=1;
float ki=1;
float kd=1;

void readEncB();
void setMotor(int dir, int pwmVal, int pwm, int m1, int m2);

void setup() {

  pinMode(encA,INPUT);
  pinMode(encB,INPUT);
  attachInterrupt(digitalPinToInterrupt(encA),readEncB,RISING);
  }

void loop() {

  //target position
  int target = 1200;

  //deltaTime
  long currTime=micros();
  float deltaT = (currTime - prevTime)/1e6;
  prevTime=currTime;

  //error
  float e = target - count;

  //e integral
  eInt = eInt + (e*deltaT);

  //e derivative
  float de = (e-ePrev)/deltaT;
  ePrev = e;

  //PID control
  float u = (kp*e) + (ki*eInt) + (kd*de);

  //controls the motor
  setMotor((u>=0)? 1:-1, constrain(abs(u),0,255), pwmPin, motor1, motor2)
}


//reading encoderB value to determine motor position, triggered on encoderA interrupt (RISING)
void readEncB(){
  int b = digitalRead(encB);
  if (b>0){
    count++;
  }
  else{
    count--;
  }
}


//sets motor values for (direction, PWM value, PWM pin, motor M1, motor M2)
void setMotor(int dir, int pwmVal, int pwmPin, int m1, int m2){
  analogWrite(pwmPin, pwmVal);
  digitalWrite(m1,(dir==1)*HIGH + (dir==-1)*LOW);
  digitalWrite(m2,(dir==1)*LOW + (dir==-1)*HIGH);
  }