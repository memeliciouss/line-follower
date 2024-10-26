#include <Arduino.h>

#define encA 2 //encoder readings from motor
#define encB 11 //encoder readings from motor
#define m_in1 5 //motor out pwm
#define m_in2 6 //motor out dir


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
  Serial.begin(9600);
  pinMode(encA,INPUT);
  pinMode(encB,INPUT);
  attachInterrupt(digitalPinToInterrupt(encA),readEncB,RISING);
  Serial.print("Setup complete");
  }

void loop() {

  //target position
  int target = 250*sin(prevTime/1e6);

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

  //control the motor
  setMotor((u>=0)? 1:-1, constrain(abs(u),0,255), m_in1, m_in2);
  
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


//sets motor values for (direction, PWM value, PWM pin, direction pin)
void setMotor(int dir, int pwmVal, int pwm_pin, int dir_pin){
  analogWrite(pwm_pin, pwmVal);
  digitalWrite(dir_pin,(dir==1)? HIGH : LOW);
  }