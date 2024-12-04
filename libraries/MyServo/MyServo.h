#include "Arduino.h"

class MyServo
{
  public:
    MyServo(int pin);
    setAbsPos(int16_t cdeg);
    shutdown();
  private:
    int pwmpin;
  
};
