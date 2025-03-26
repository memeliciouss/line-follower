#include <Arduino.h>

const int LED_PIN=13;

void setup() {
  Serial.begin(9600);

  TCCR1A = 0;
  TCCR1B = 0;

  // Set CTC mode
  TCCR1B &= ~(1 << WGM13);  // Clear WGM13
  TCCR1B |= (1 << WGM12);   // Set WGM12 for CTC mode
  
  // Set prescaler to 256
  TCCR1B |= (1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B &= ~(1 << CS10);
  
  // Set OCR1A to 32000 (32000/256) for interrupts at every 2 mili seconds
  OCR1A = 125;
  
  // Enable Timer1 compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();
}

void loop() {
  
}

ISR(TIMER1_COMPA_vect) {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}
