#ifndef motor_h
#define motor_h

class motor
{
public:
  motor(int,int,int,int); // constructor
  void setPWM(int);
  void setPWMlimit(int,int);
  float getCurrent();
  boolean getError();
private:
    int outputMin,outputMax;
  int myDirPin, myPwmPin, myCurrentPin, myErrorPin; // local pin definitions
  int myValue;
  int rawADCvalue;
};

#endif

