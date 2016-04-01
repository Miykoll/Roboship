#include "Arduino.h"
#include "motor.h"

motor::motor(int dirPin, int pwmPin,int currentPin, int errorPin){
  myDirPin= dirPin;
  myPwmPin= pwmPin;
  myCurrentPin = currentPin;
  myErrorPin = errorPin;
  myValue = 0;
  motor::setPWMlimit(-255,255); // default value, might be changed later.
  pinMode(myDirPin,OUTPUT);
  pinMode(myPwmPin,OUTPUT);
  digitalWrite(myErrorPin,HIGH); // turn on pullup
  TCCR1B = 0x01; // set timer to high frequency (inaudible PWM)
}

float motor::getCurrent(){
  static float returnvalue;
  rawADCvalue = analogRead(myCurrentPin);
  returnvalue += rawADCvalue;
  returnvalue = returnvalue/2;
  return returnvalue;
}
boolean motor::getError(){
 return digitalRead(myErrorPin);
}


void motor::setPWM(int value){
  myValue = constrain(value,outputMin,outputMax);
  digitalWrite(myDirPin,(value<0 ? HIGH : LOW));
  analogWrite(myPwmPin,(value<0 ? abs(value+255) : abs(value)));
}

void motor::setPWMlimit(int minValue, int maxValue){
  outputMin = minValue;
  outputMax = maxValue;
}

