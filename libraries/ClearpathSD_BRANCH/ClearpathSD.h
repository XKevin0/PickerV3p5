#include "Arduino.h"

class ClearpathSD{
  public:
    ClearpathSD(int HL, int inpA, int inpB, int en, int pro);
    engage();
	disengage();
    setSpeed(int rpm);
	pulseGen(int setICR1, int averageStepLength);
	stop();
	
	int setAbsPos(int pos);
	int home();
	
	// getters and setters
	int getZpos();
	setZpos(int pos);
	int getZdirection();
	int getZtarget();
	int getStepsToRamp();
	int getStepsToStop();
	int getZdist();
	
  private:
	
    int HLFB;
    int inputB;
    int inputA;
    int enable;
	int prox;
	
    int ppr;
    double zrpm;
	
	// zmotor tracking
    int zpos;
	int ztarget;
	
	// zmotor acceleration
	int zdirection;
	int stepsToRamp;
	int stepsToStop;
	int timer2Steps;
	int zdist;
};
