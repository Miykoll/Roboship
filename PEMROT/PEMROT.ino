
/************************************************************************************
 Standard roboshipPEMrotMotor
 
 Version 1.3, Michael ten Den, March 31th, 2016
 Firmware is based on Standard pirateMotorDriver, Version 1.0, E13, May 24th, 2014
 
 Notes V1.3
 - Added PID contoller
 - Mechanical stop prevents the base to rotate more than 360 degrees (NOT IMPLEMENTED)
 - Another interrupt is implemented for the wall detection (NOT IMPLEMENTED)
 - The interrupt for the encoder is upgraded with an angle and revolutions counter.
 The ratio of the encoder is unknown but can be easily determined by a test of one 
 full revolution and counting the rising PCINT change (Ratio is 412 counts per 360 deg

---------------------------------------------------------------------------------
/* Pin to interrupt map:
* D8 - D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
*/


boolean DEBUG = false;

//#include <SPI.h>
#include "DynamixelReader.h"
#include "motor.h"
#include "Arduino.h"
#include <PID_v1.h>

#include <avr/wdt.h>
#include <avr/interrupt.h>

// pin defines
#define LEDblue A5
#define LEDred A4
#define RS485sr 2
#define hallOutA 11 // MOSI pin i.e. PCINT3
#define hallOutB 12 // MISO pin i.e. PCINT4
#define SS2 6 // For the wall sensor

// **** Parameters ****
volatile unsigned long lasttime, transitiontime;
volatile int drivingdirection;
unsigned long khztime;
unsigned long hztime;
unsigned long pemHzTime;
volatile long encodercount = 0;
volatile long prevEncoderCount;
volatile long currentROTAngle;

// PID parameters
 double Kp = 2.5;
 double Ki = 10.0;//2.02;
 double Kd = 1.5;
 int PWM_MIN = 255;
 int PWM_MAX = 255;
 double PIDSetpoint, PIDInput, PIDOutput;
 volatile double ErrorPID;

byte dataValid = 0;

int revolutions,revolutionsOffset;
int rotMargin = 2;
const float pi = 3.14;

// Debug Interrupt params 
volatile uint8_t counterPCINT = 0;
volatile uint8_t portbhistory = 0x00;
volatile boolean boolINT = false;
volatile boolean boolRising = false;
volatile boolean boolFalling = false;

// **** Constructors ****
// Constructing Dynamixel
DynamixelRAM DXL_PEM;
char* DXL_PEM_ptr = (char*) &DXL_PEM;
DynamixelRAM DXL_ROT;
char* DXL_ROT_ptr = (char*) &DXL_ROT;


// Constructing motor and PEM
//motor Motor_PEM = motor(8,9,A0,A2);  // PEM
motor Motor_ROT = motor(7,10,A1,A3); // Rotation motor
//motor Motor_ROT = motor(8,9,A0,A2); // DEBUG

// Construct PID Controller
PID PID_ROT(&PIDInput,&PIDOutput,&PIDSetpoint,Kp,Ki,Kd,REVERSE);


// the setup routine runs once when you press reset:
void setup() {       
Serial.begin(250000); // Dynamixel communcition
//Serial.begin(115200); // Serial communication 

  // PEM pins Enabling
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  digitalWrite(3,HIGH);
  digitalWrite(4,HIGH);
  
  // Wall sensor, pull-up sensor
  pinMode(SS2,INPUT); // SS2 -> Wall sensor
  digitalWrite(SS2,HIGH); //pull up

  wdt_enable(WDTO_500MS);
  TCCR1B = TCCR1B & 0b11111000 | 0x01;

  pinMode(LEDblue,OUTPUT);
  pinMode(LEDred,OUTPUT);
  pinMode(RS485sr,OUTPUT);
  
  // Interrupt setup
  pinMode(hallOutA,INPUT);
  pinMode(hallOutB,INPUT);

  pciSetup(hallOutA);
  pciSetup(hallOutB); 
  
 // Motor_PEM.setPWM(0);
  Motor_ROT.setPWM(0);
  
  // Parameters PID controller
  PID_ROT.SetSampleTime(1);
  PID_ROT.SetOutputLimits(-PWM_MIN,PWM_MAX);
  PID_ROT.SetMode(AUTOMATIC);

  /// DXL 1:
     //DXL_PEM.model=0x0140; // MX-106
      DXL_PEM.model=0x0040; //RX-64
      DXL_PEM.version = 0x24; //??
      DXL_PEM.ID = BOARD_ID_1;
      DXL_PEM.baudrate = 34;
      DXL_PEM.returnDelay = 10;  
  
  /// DXL 2: Rotation is always the second ID
      //DXL_ROT.model=0x0140; // MX-106
       DXL_ROT.model=0x0040; //RX-64
       DXL_ROT.version = 0x24; //??
       DXL_ROT.ID = BOARD_ID_2;
       DXL_ROT.baudrate = 34;
       DXL_ROT.returnDelay = 10;
       DXL_ROT.angleLimitCCW = 1023;
       DXL_ROT.angleLimitCW = 0;
       DXL_ROT.complianceMarginCW = 10*Kd +1; // P_D_GAIN max 255
       DXL_ROT.complianceMarginCCW = 10*Ki +1; // P_I_GAIN
       DXL_ROT.complianceSlopeCW = 10*Kp +1;//P_P_GAIN
       DXL_ROT.complianceSlopeCCW = PWM_MIN; // PWM min    
       DXL_ROT.movingSpeed = PWM_MAX; // PWM max
       DXL_ROT.presentLoad = 0; // Current value  
       
}

// the loop routine runs over and over again forever:
void loop() 
{
  uint8_t activePINS = 24; // debug

  wdt_reset();
  DynamixelPoll();
  DXL_ROT.presentPosition = currentROTAngle; // * (1024 / 360);
  DXL_ROT.presentSpeed = encodercount;
  DXL_ROT.presentLoad = int(Motor_ROT.getCurrent()); // get current the motor is using
  
  if(millis()>khztime+1)
  { // 500 Hz
    khztime = millis();
    hztime++;
    if(hztime>49)    ///10Hz
    {
      //digitalWrite(LEDred,DXL_ROT.LED);
      DXL_PEM.currentTorque = digitalRead(SS2); // if LOW then connected to the wall
      
      // Control of the magnet PEM
		switch(DXL_PEM.movingSpeed)
		{											// PEM
		   case 0: // Magnet normal
			{
			  digitalWrite(3,HIGH);
			  digitalWrite(4,HIGH);
			  break;
			}
			case 1: // Magnet ON/OFF
			{
			  digitalWrite(3,HIGH);
			  digitalWrite(4,LOW);
			  DXL_PEM.movingSpeed = 11;
			  break;
			}
			case 2: // Magnet ON/OFF
			{
			  digitalWrite(3,LOW);
			  digitalWrite(4,LOW);
			  DXL_PEM.movingSpeed = 21;
			  break;
			}
			default:
			{
			  break;
			}
		}

      
      
      /* Calibration of the base rotation motor
        *  LED == 1, then calib mode. Calib mode is:
        *  rotate until mechanical block. If against mech block
        *  then make this point 0 deg. No sensor on the block
        *  and current measure needs filtering, therefore if
        *  position not changing for 1/4 sec, then calibration 
        *  finished.
       */
		/*
     if(DXL_ROT.LED) // Calibration mode with mechanical stop
      {
        DXL_ROT.torqueEnable = 0;
        if( (encodercount > (prevEncoderCount - rotMargin)) && (encodercount < (prevEncoderCount + rotMargin)) ) // This means the base is not rotating any more. Margin is for slightly changing encoder pulses
        { 
          pemHzTime++;
          if(pemHzTime == 3) // 300 ms
          {
            DXL_ROT.LED = 0;
            encodercount = 0; // Reset the counter to be 0;
            Motor_ROT.setPWM(0);
          } 
        } 
        else 
        {
          pemHzTime = 0;
          Motor_ROT.setPWM(200);
        }
        
      }
      */
		
		if(DXL_ROT.LED) // Calibration mode without mechnaical stop. User input required
		{
			Motor_ROT.setPWM(0);
			encodercount = 0;
			currentROTAngle = 0;
            DXL_ROT.LED  = 0;
		}
       
         /*
       // Counter interrupt serial print
         digitalWrite(RS485sr, HIGH);
          Serial.print("CountInt: ");
          Serial.print(counterPCINT);
          Serial.print("; EncCount: ");
          Serial.print(encodercount);
          Serial.print("; Direction: ");
          Serial.print(drivingdirection);
          Serial.print("\r\n");
          Serial.flush();
         digitalWrite(RS485sr, LOW);
       */
       

     
       
     // check PID values and update if changed
     if((DXL_ROT.complianceMarginCW-1 != int(10*Kd)) || (DXL_ROT.complianceMarginCCW-1 != int(10*Ki )) || (DXL_ROT.complianceSlopeCW-1 != int(10*Kp )))
     {
       Kd = double(DXL_ROT.complianceMarginCW-1)/10;
       Ki = double(DXL_ROT.complianceMarginCCW-1)/10;
       Kp = double(DXL_ROT.complianceSlopeCW-1)/10;
       PID_ROT.SetTunings(Kp/10,Ki,Kd);
       
       if(DEBUG){
		   digitalWrite(RS485sr, HIGH);
			  Serial.print("PID values updated:");
			  Serial.print("\r\n");
			  Serial.flush();
			 digitalWrite(RS485sr, LOW);   
       }
     }
     // Update PWM limits via complianceSlopeCCW and movingSpeed
     if((DXL_ROT.complianceSlopeCCW != PWM_MIN) || (DXL_ROT.movingSpeed != PWM_MAX) )
     {
       PWM_MIN = DXL_ROT.complianceSlopeCCW;
       PWM_MAX = DXL_ROT.movingSpeed;
       PID_ROT.SetOutputLimits(-PWM_MIN,PWM_MAX);
       DXL_ROT.currentAngleLoad = PWM_MAX;
     }
     
     if(DXL_ROT.torqueEnable){
    	 if(DEBUG){
			digitalWrite(RS485sr, HIGH);
			  Serial.print("PIDInput: ");
			  Serial.print(PIDInput);
			  Serial.print("; PIDSetpoint: ");
			  Serial.print(PIDSetpoint);
			  Serial.print("; PIDOutput: ");
			  Serial.print(PIDOutput);
			  Serial.print("\r\n");
			  Serial.print("Kp: ");
			  Serial.print(Kp);
			  Serial.print("; Ki: ");
			  Serial.print(Ki);
			  Serial.print("; Kd: ");
			  Serial.print(Kd);
			  Serial.print("\r\n");
			  Serial.flush();
			 digitalWrite(RS485sr, LOW);
    	 }
     }
     
       

        hztime=0;
        nudgeTimeOut();
        toggle(LEDblue);
    } //10Hz

   } // 500Hz
   
// Control of the rotation of the base
 if (1 == DXL_ROT.torqueEnable){
   calculatePWMSetpoint(&PIDInput,&PIDSetpoint,DXL_ROT.goalPosition);
   DXL_ROT.presentVoltage = abs(PIDOutput);
   Motor_ROT.setPWM(-PIDOutput); // Negative PWM value to turn right direction
 } else {
   Motor_ROT.setPWM(0);
  }
}


void ProcessDynamixelData(const unsigned char ID, const int dataLength, const unsigned char* const Data){
  byte buffer[DYNAMIXEL_RETURN_SIZE];
 /// toggle(LEDred);
  switch(Data[0]){
  case 0x01: //ping
    ReturnDynamixelData(ID,0,0);
    break;
  case 0x02: //read
   
   if (DXL_ROT.ID == ID)
   {
      for(int n=0; n<Data[2];n++)
      buffer[n] = DXL_ROT_ptr[Data[1]+n];
       dataValid=1;
      delayMicroseconds(DXL_ROT.returnDelay*2);
      ReturnDynamixelData(ID,Data[2],buffer);
      break; 
   }
   if (DXL_PEM.ID == ID)
    {
      for(int n=0; n<Data[2];n++)
        buffer[n] = DXL_PEM_ptr[Data[1]+n];
      //      buffer[n] = memCom[Data[1]+n];
    
      dataValid=1;
      delayMicroseconds(DXL_PEM.returnDelay*2);
      ReturnDynamixelData(ID,Data[2],buffer);
      break; 
    }
     
  case 0x03: // Write command
    if(Data[1]<MEM_LENGTH && (dataLength-3)<(MEM_LENGTH-Data[1]))
    {// safeguard
      if ( DXL_PEM.ID == ID)
      { 
        for (int n=0;n<dataLength-3;n++)
        {
          DXL_PEM_ptr[(byte)Data[1]+n] = (byte)Data[2+n];
          //        memCom[(byte)Data[1]+n] = (byte)Data[2+n];
        }
      }
      if ( DXL_ROT.ID == ID)
      { 
        for (int n=0;n<dataLength-3;n++)
        {
          DXL_ROT_ptr[(byte)Data[1]+n] = (byte)Data[2+n];
          //        memCom[(byte)Data[1]+n] = (byte)Data[2+n];
        }
      }      
      dataValid = 1;
    }
    break;
  case 0x04: //REG WRITE
  case 0x05: //ACTION
  case 0x06: //RESET 
  case 0x83: //SYNC WRITE
  default:
    break;
  }
}

void ReturnDynamixelData(const unsigned char ID, const int dataLength, const unsigned char* const Data){
  unsigned char buffer[DYNAMIXEL_RETURN_SIZE];
///  unsigned int checksum = 0;
  unsigned char checksum = 0;
  unsigned char error = dynamixelError();
  buffer[0] = (0xFF);
  buffer[1] = (0xFF);
  buffer[2] = (ID);
  /////
  buffer[3] = (dataLength+2); // was dataLegth+2
  buffer[4] = (error);
  if(dataLength>0){
    for (int n = 0; n<(dataLength); n++) { // dataLength = N paramaters + 2
      buffer[5+n] = Data[n];
      checksum += Data[n];
    }
    buffer[5+dataLength] = (0xFF & (~(checksum+ID+error+dataLength+2)));
  }
  else{
    buffer[5+dataLength] = (~(ID+error+dataLength+2));
  }
  // OK, do transmission now: //
  digitalWrite(RS485sr,HIGH); // send data to HOST
  Serial.write(buffer,dataLength+6);
  Serial.flush(); // wait for buffer to empty
  digitalWrite(RS485sr,LOW);
}

unsigned char dynamixelError(){
  unsigned char returnValue = 0;
  if(Motor_ROT.getError()) returnValue |= (1<<5); // current overload if .getError() == 0
  return returnValue;
}


// This enables the interrupt in the register of the assigned pin.  
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

// Interrupt routine which is called when interrupt occurs
ISR (PCINT0_vect)
{
	// Parameters define
  boolean boolCCW = false;
  boolean boolCW = false;
  uint8_t changedbits;
  uint8_t activePINS = 24;
 
  // Pattern CCW
  // 0 -> 01000 -> 11000 -> 10000 -> 0
  // 0 -> 8 -> 24 -> 16 -> 0

  // Pattern CW
  // 0 -> 10000 -> 11000 -> 01000 -> 0
  // 0 -> 16 -> 24 -> 8 -> 0
  
  // Check if one of the encoder pins is changed in value.
  // If changed, check the previous value to determine if 
  // the base is rotating CW or CCW.
  switch(PINB & activePINS){
    case 0:
    {
      if(portbhistory == 8){boolCW = true;}
      else if(portbhistory == 16){boolCCW = true;}
      break;
    }
    case 8:
    {
      if(portbhistory == 24){boolCW = true;}
      else if(portbhistory == 0){boolCCW = true;}
      break;
    }
    case 16:
    {
      if(portbhistory == 0){boolCW = true;}
      else if(portbhistory == 24){boolCCW = true;}
      break;
    }
    case 24:
    {
      if(portbhistory == 16){boolCW = true;}
      else if(portbhistory == 8){boolCCW = true;}
      break;
    }
    default:
    {
      break;    
    }
  }
  
  if(boolCCW){
    encodercount++; 
    drivingdirection = 1; // CCW
  }
  if(boolCW){
    encodercount--; 
    drivingdirection = -1; // CW
  }
  
  boolCCW = false;
  boolCW  = false;
  
  encoderToAngle();
      
  changedbits  = PINB ^ portbhistory;
  portbhistory = PINB & activePINS;
  counterPCINT = changedbits; 
}


// Called in interrupt routine
// Encoder count to angle in rad
void encoderToAngle()
{
  currentROTAngle = encodercount * 1024 / (2*51.45*4); // Pololu gear ratio = 51.45, Number of changes per revolution motor shaft = 4, Gear ratio with base = 2;
}


// Calculates the PWM by use of PID
int calculatePWMSetpoint(double* inputPID, double* setpointPID, int goalPos)
{
  // Update the setpoint, input and output value by pointer for the PID object
  // Goal pos ranges from 0 till 1023
    if(goalPos > DXL_ROT.angleLimitCCW){
      goalPos = DXL_ROT.angleLimitCCW;
  } else if(goalPos < DXL_ROT.angleLimitCW){
    goalPos = DXL_ROT.angleLimitCW; 
  }
  
   *setpointPID = double(goalPos);
   *inputPID = currentROTAngle;
   PID_ROT.Compute(); 
}

