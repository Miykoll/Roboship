#include "Arduino.h"
#include "AS5055.h"

AS5055::AS5055(int ss){
  mySS = ss;
  // Start the SPI library:
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32) ;  //SPI_dev = 16 : 1MHz;   (2,4,8,16,32,64,128) 
  SPI.setBitOrder( MSBFIRST  );
  SPI.setDataMode( SPI_MODE1 );
  // Deselect sensor-slave: 
  pinMode(mySS, OUTPUT);
  digitalWrite(mySS, HIGH); // deselect
}

int AS5055::getRawValue(){
//////////////////////
// Read angular data from the AS5055 (16bits)
// The data is treated as 14 bit of data
// with the actual resolution 12 bit
  static int lastValue,lastDDRawValue;
  digitalWrite(mySS, LOW); // activate slave

  // send a value to read the bytes returned:
  data1 = SPI.transfer(0xff);
  data2 = SPI.transfer(0xff);

  // take the chip select high to de-select:
  digitalWrite(mySS, HIGH);
  data1 &=0b00111111;
  data1 = data1<<8;
  rawValue= (data1 | data2)>>2;
  if(rawValue-offsetValue<0) return (rawValue-offsetValue+4096)%4096;
  else return((rawValue-offsetValue)%4096);
}

void AS5055::setOffset(int offset)
{
  offsetValue = offset;
}

float AS5055::getAngle(){
    return ((((AS5055::getRawValue()))) *360.0) / 4095.0;     // Angle value in degree (0..359.9) 
}


