#ifndef AS5055_h
#define AS5055_h

#include <Arduino.h>
#include <SPI.h>

#define ss1 5
#define ss2 6 // or 5 for other port



class AS5055
{
public:
  AS5055(int ss); // constructor: set ss
  float getAngle();
  int getRawValue();
  void setOffset(int offset);
private:
  unsigned int data1,data2;
  int rawValue;
  int offsetValue;
  int mySS;
};

#endif

