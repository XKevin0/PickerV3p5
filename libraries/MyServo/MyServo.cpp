#include "Arduino.h"
#include "MyServo.h"

MyServo::MyServo(int pin){
  pwmpin = pin;
  
  pinMode(pwmpin, OUTPUT);
}

// set abs pos from 0 to 180 deg
// 500 - 2500us = 2kHz to 0.4kHz
// use a 200Hz PWM signal = 5000us period
// 10% to 50% duty cycle 
// Using timer2 16Mhz / 512 prescaler / 200Hz = 156.25
// set timer2 overflow to 156 for 200.32051Hz
// Fast PWM mode WGM1 = 1, WGM0 = 1
// OCRA = TOP
// prescaler CS22 = 1, CS21 = 1
// compare output mode, fast pwm mode COM2A1 = 1
MyServo::setAbsPos(int16_t cdeg){
  int pulseLength = map(cdeg, 0, 18000, 26, 128);

  
  // set up PWM
  TCCR2A = 0;
  TCCR2B = 0;
  
  TCCR2A |= (1<<WGM21) | (1<<WGM20) | (1<<COM2A1);
  TCCR2B |= (1<<CS22) | (1<<CS21);
  OCR2A = pulseLength;
}

MyServo::shutdown(){

  // reset registers to stop PWM
  TCCR2A = 0;
  TCCR2B = 0;
}
