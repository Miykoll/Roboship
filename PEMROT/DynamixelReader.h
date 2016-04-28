#ifndef __DYNAMIXEL_READER_H__
#define __DYNAMIXEL_READER_H__

#include <Arduino.h>

///#define DYNAMIXEL_BUFFER_SIZE (64) /* Just a lot so that it will not overflow */
///#define DYNAMIXEL_RETURN_SIZE 32 
#define DYNAMIXEL_BUFFER_SIZE (128) /* Just a lot so that it will not overflow */
#define DYNAMIXEL_RETURN_SIZE 128 

#define BOARD_ID_1 0x03
#define BOARD_ID_2 0x04

#define MEM_LENGTH 0x34

#define toggle(pin) digitalWrite(pin, !digitalRead(pin)) 
#define	outb(addr, data)	addr = (data)
#define	inb(addr)		(addr)
#define BV(bit)			(1<<(bit))
#define cbi(reg,bit)	reg &= ~(BV(bit))
#define sbi(reg,bit)	reg |= (BV(bit))

#define byte uint8_t


// Prototype for the function we need to call from outside
void DynamixelPoll(void);
void nudgeTimeOut(void);
int getTimeOut(void);

struct DynamixelRAM {
// note, Arduino compiler stores bytes LSB first (so the lowest byte (LSB) of 'int model' is stored on place 0, the MSB in place 1)
  int model; //0x00
  byte version; //0x02
  byte ID;
  byte baudrate;
  byte returnDelay;
  int angleLimitCW; //0x06
  int angleLimitCCW;
  byte dummy4; //0x0A is empty
  byte tempLimitHigh;//0x0B
  byte voltageLimitLow;
  byte voltageLimitHigh;
  int maxTorque; //0x0E
  byte statusReturn;//0x10
  byte alarmLED;
  byte alarmShutdown; //0x12
////----- end of EEPROM memory -----////
  int springOffset; //0x13
  int angleOffset; //0x15
  byte springConstant; //0x17
////---- gap in memory space for future eeprom//
  byte torqueEnable; //0x18
  byte LED;
  byte complianceMarginCW; // 0x1A
  byte complianceMarginCCW; // 0x1B
  byte complianceSlopeCW; //0x1C
  byte complianceSlopeCCW; //0x1D
  int goalPosition; //0x1E
  int movingSpeed; //0x20
  int torqueSetpoint; //0x22
  int presentPosition;   //0x24 - odometry
  int presentSpeed;      //0x26 - driving speed
  int presentLoad;  //0x28 - drive motor load
  int presentVoltage;      //0x2A - module angle
  int currentAngleLoad;  //0x2C - bend motor load
  int currentTorque;     //0x2E - clamping force
  byte bendControlMode;      //0x30
  byte driveControlMode; //0x31
  byte currentTemperature;
  byte registered;
  byte moving; //0x35
  byte lock;
  int punch;
};

#endif

