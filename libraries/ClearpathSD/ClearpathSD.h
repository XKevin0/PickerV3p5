#include "Arduino.h"

class ClearpathSD{
  public:
    ClearpathSD(int HL, int inpA, int inpB, int en, int pro);
    engage();
	disengage();
    int setAbsPos(int32_t pos);
    setSpeed(int rpm);
    setAccel(int rpms);
	pulseGen(int setICR1);
	stop();
	long int home();
	setzpos(int32_t zposTemp);
    
  private:
	
    int HLFB;
    int inputB;
    int inputA;
    int enable;
	int prox;
	
    int32_t zpos;
    int32_t ppr;
	int zaccel;
    double zrpm;
};
