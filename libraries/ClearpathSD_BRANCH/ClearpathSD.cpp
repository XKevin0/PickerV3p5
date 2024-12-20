#include "Arduino.h"
#include "ClearpathSD.h"

ClearpathSD::ClearpathSD(int HL, int inpA, int inpB, int en, int pro){
  // define pins
  // standard HLFB = A3, inputA = A2, inputB = PD10, enable = A0;
  enable = en;
  inputA = inpA;
  inputB = inpB;
  HLFB = HL;
  prox = pro;
  
  // define pinmodes
  pinMode(en, OUTPUT);
  pinMode(inpA, OUTPUT);
  pinMode(inpB, OUTPUT);
  pinMode(HLFB, INPUT_PULLUP);
  pinMode(prox, INPUT);
	
  ppr = 6400;     // 6400 pulses per revolution
  zrpm = 100;   // default rpm = 100
  
  // initialize z tracking variables
  zpos = 0;  // 122,324 encoder counts per full range
  zdirection = 0; // 1 up -1 down 0 dont move
  ztarget = 0;
  stepsToRamp = 0;
  stepsToStop = 0;
  zdist = 0;
}



// move up until prox sensor
// set pos to 122,324 encoder counts
// ppr = encoder counts per rev = 6400
// HOME PAUSES CODE EXECUTION
int ClearpathSD::home(){
  // TODO wire proximity Sensor
  // set zpos to 0
  zpos = 0;
  return zpos;
  // move up
  // pulseGen(255);
  
  // while(digitalRead(7) == LOW);
  
  // zpos = 120000;
  // return zpos;
}

// absolute position move
// 0 encoder counts = lowest point
// 122,324 encoder counts = highest point 
// direction determined by math
int ClearpathSD::setAbsPos(int pos){
  bool up;
  zdist = 0;
  ztarget = pos;
  
  // calculate direction and distance
  // CW is up = inputA HIGH
  // CCW is down = inputA LOW
  if(pos > zpos){
    up = true; 
    zdist = (pos - zpos) * ppr / 6400;
	zdirection = 1;
  }else if (pos < zpos){
    up = false;
    zdist = (zpos - pos) * ppr / 6400;
	zdirection = -1;
  }else{
    return;
  };
	
	// set direction
	digitalWrite(inputA, up);

	// calculate pulses per second according to speed
	// calculate OCR1A and ICR1
	long int pps = zrpm * ppr / 60;
	int targetICR1 = 2000000.0/pps - 1;
	
	// example ICR1: pps = 100 * 6400 / 60 = 10666.666 = 10666
	// setICR1 = 2000000 / 10666 - 1 = 186.511
	// minimum speed = 7812.5 pps / 6400 ppr =  73.2421875 rpm
	
	// timer2 acceleration
	// 20% accelerating, 60% at speed, 20% decelerating
	// calculate stepsToAccel, stepsToDecel
	// calculate average step frequency
	// use timer2 to ramp ICR1 for ramping and deramping 
	stepsToRamp = zdist / 5;
	stepsToStop = zdist - stepsToStop;
	int averageStepLength = (255 + targetICR1) / 5 + (targetICR1 * 3 / 5);
	int rampRate = stepsToRamp / (255 - targetICR1);
	
	pulseGen(targetICR1, averageStepLength);
}

// set Speed by changing pulse frequency
// max pulse freq = 667kHz
// min pulse period = 1.5us
// MCU clock = 16MHz --> no rpm limit
// motor RPM limit = 4000
ClearpathSD::setSpeed(int rpm){
  zrpm = rpm;
}

// engage motor by setting enable HIGH
ClearpathSD::engage(){
  // ensure motor is disabled
  digitalWrite(enable, LOW);

  // set inputs to low
  digitalWrite(inputA, LOW);
  digitalWrite(inputB, LOW);

  // toggle enable to engage motor
  digitalWrite(enable, HIGH);
}

// disengage motor
ClearpathSD::disengage(){
  stop();
  digitalWrite(enable, LOW);
  digitalWrite(inputA, LOW);
  digitalWrite(inputB, LOW);

}

// Stop motion and disable interrupt
ClearpathSD::stop(){
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 = 0;
}

// Output pulses on digital pin 9 with set frequency
ClearpathSD::pulseGen(int setICR1, int averageStepLength){
  /*// Set up PWM timer2
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 = 0;
  
  timer2Steps = 0;
  
  // WGM2, WGM1, WGM0 = 1 for Fast PWM OCRA Top
  // CS21 = 8 for 8 factor prescaling
  // Enable interrupts when timer2 overflows
  TCCR2A |= (1<<WGM21) | (1<<WGM20);
  TCCR2B |= (1<<WGM22) | (1<<CS21);
  TIMSK2 |= (1<<TOIE2);
  
  OCR2A = averageStepLength;
  */
  if (ICR1 > 0xFFFF){
	ICR1 = 0xFFFF;
  }
  
    // Set up PWM timer1
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  
  TCCR1A |= (1<<WGM11) | (1<<COM1B1);
  // WGM13, WGM12, WGM11 = 1 for Fast PWM ICR1 Top COM1B1 = 1 
  // CS11 = 1 for 8 factor prescaling
  TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11); 
  // Enable interrupts when timer1 overflows
  TIMSK1 |= (1<<TOIE1);
  // max pulse freq = 500khz
  OCR1B = 2;
  ICR1 = setICR1;
  	
}

int ClearpathSD::getZpos(){
	return zpos;
}

int ClearpathSD::setZpos(int pos){
	zpos = pos;
}

int ClearpathSD::getZdirection(){
	return zdirection;
}

int ClearpathSD::getZtarget(){
	return ztarget;
}

int ClearpathSD::getZdist(){
	return zdist;
}

int ClearpathSD::getStepsToRamp(){
	return stepsToRamp;
}

int ClearpathSD::getStepsToStop(){
	return stepsToStop;
}