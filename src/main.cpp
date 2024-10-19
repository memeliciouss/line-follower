#include <Arduino.h>

int encA=2; //encoder readings from motor
int encB=3; //encoder readings from motor
volatile int count=0;

void readEncB();

void setup() {
  Serial.begin(9600);

  pinMode(encA,INPUT);
  pinMode(encB,INPUT);
  pinMode(5,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encA),readEncB,RISING);
  }

void loop() {
  int a = digitalRead(encA);
  int b = digitalRead(encB);
  Serial.print(a);
  Serial.print(" ");
  Serial.print(b);
  Serial.print(" ");
  Serial.println(count);
}
void readEncB(){
  int b = digitalRead(encB);
  if (b>0){
    count++;
  }
  else{
    count--;
  }
}