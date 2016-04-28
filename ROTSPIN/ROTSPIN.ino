/************************************************************************************
 * roboship Motor 1(Spindle Arm) and 2 (Rotation Arm)
 * 
 * Version 1.3, Michael ten Den, March 29th, 2016
 *
 */

boolean DEBUG = false;

#include <SPI.h>
#include "DynamixelReader.h"
#include "motor.h"
#include "AS5055.h"
#include "PID_v1.h"

#include <avr/wdt.h>

// pin defines
#define LEDblue A5
#define LEDred A4
#define RS485sr 2
#define ss1 5
#define ss2 6

// **** Parameters ****
unsigned long khztime;
unsigned long hztime;
unsigned long hztime2;
byte dataValid = 0;

long PWMvalue1, PWMvalue2;
const float pi = 3.14;

bool calibrationBool1, calibrationBool2;
bool spindleCalibFlag = false;
int spindleFlag   = 0;
bool rotFlag = false;
boolean upFlag;

int load_t = 0;
int maxCur2 = 0;

// PID parameters Rot arm
 double Kp = 5;
 double Ki = 2.0;//2.02;
 double Kd = 0.01;
 int PWM_MIN = 255;
 int PWM_MAX = 255;
 double PIDSetpoint, PIDInput, PIDOutput;
 volatile double ErrorPID;

// Revolutions
long prevPos1, curPos1, prevPrevPos1, revolutions1, diffPos1, offsetPos1, newPos1; // Required to track if the spindle has made a revolution
long prevPos2, curPos2, prevPrevPos2, revolutions2, diffPos2, offsetPos2, newPos2; // Required to track if the Rot has made a revolution

// Counter for calibration and spindle control
int counterLED = 0;
int counterSpindle = 0;
int counterRot = 0;

// **** Constructors ****
// Construct dynamixels
DynamixelRAM DXL_SPIN;
char* DXL_SPIN_ptr = (char*) &DXL_SPIN;
DynamixelRAM DXL_ROT;
char* DXL_ROT_ptr = (char*) &DXL_ROT;

// Construct motors and sensors 
motor Motor_ROT = motor(8,9,A0,A2); // dir on 8, PWM on 9, current on A0, error on A2
motor Motor_SPIN = motor(7,10,A1,A3); // dir on 7, PWM on 10, current on A1, error on A3
AS5055 Sensor1 = AS5055(ss1); // ss on pin 6
AS5055 Sensor2 = AS5055(ss2); // ss on pin 5

// Construct PID controller
PID PID_ROT(&PIDInput,&PIDOutput,&PIDSetpoint,Kp,Ki,Kd,REVERSE);

// the setup routine runs once when you press reset:
void setup() {        
  Serial.begin(250000); // Dynamixel communication
  //Serial.begin(115200); // Serial communication

  pinMode(LEDblue,OUTPUT);
  pinMode(LEDred,OUTPUT);
  pinMode(RS485sr,OUTPUT);

  wdt_enable(WDTO_500MS);

  Motor_SPIN.setPWM(0);
  Motor_ROT.setPWM(0);

  Sensor1.setOffset(0);
  Sensor2.setOffset(0);  
  PWMvalue1 = 0;
  PWMvalue2 = 0;
  
  // Revolutions init
  revolutions1 = 0;
  revolutions2 = 0;
  curPos1 = 0;
  curPos2 = 0;
  prevPos1 = 0;
  prevPos2 = 0;
  prevPrevPos1 = 0;
  prevPrevPos2 = 0;
  offsetPos1 = 0;
  offsetPos2 = 0;
  newPos1 = 0;
  newPos2 = 0;
  
  // Calibration
  calibrationBool1 = false;
  calibrationBool2 = false;
  upFlag = true;

  TCCR1B = TCCR1B & 0b11111000 | 0x01;

  // Parameters PID controller
  PID_ROT.SetSampleTime(1);
  PID_ROT.SetOutputLimits(-PWM_MIN,PWM_MAX);
  PID_ROT.SetMode(AUTOMATIC);

  // DXL configuration
  /// DXL 1:
    //DXL_SPIN.model=0x0140; // MX-106
    DXL_SPIN.model=0x0040; //RX-64
    DXL_SPIN.version = 0x24; //??
    DXL_SPIN.ID = BOARD_ID_1;
    DXL_SPIN.baudrate = 34; // or 34
    DXL_SPIN.returnDelay = 10;
    DXL_SPIN.torqueEnable = 0;    // Torque enable dxl 1
    DXL_SPIN.goalPosition = 1000;//Sensor1.getRawValue();     // goalPosition dxl 1
    DXL_SPIN.movingSpeed  = 0;
    // DXL_SPIN.complianceSlopeCW	= 100;  // Kp1
    //DXL_SPIN.complianceMarginCCW = 50;  // bang banf gain
    //DXL_SPIN.complianceMarginCW = 870 + DXL_SPIN.complianceMarginCCW;

  /// DXL2 - Rotation is always the second ID
    // DXL_ROT.model=0x0140; // MX-106
    DXL_ROT.model=0x0040; //RX-64
    DXL_ROT.version = 0x24; //??
    DXL_ROT.ID = BOARD_ID_2;
    DXL_ROT.baudrate = 34;// or 34
    DXL_ROT.returnDelay = 10;
    DXL_ROT.torqueEnable  = 0;    // Torque enable dxl 1
    DXL_ROT.goalPosition = 600; //Sensor2.getRawValue();// Sensor1.getRawValue();     // goalPosition dxl 1
    DXL_ROT.movingSpeed  = 0;     // goalPosition dxl 2
    DXL_ROT.angleLimitCW = 50; // Min angle possible
    DXL_ROT.angleLimitCCW = 870 + DXL_ROT.angleLimitCW; // Max angle possible
    DXL_ROT.complianceMarginCW = 10*Kd +1; // P_D_GAIN max 255
    DXL_ROT.complianceMarginCCW = 10*Ki +1; // P_I_GAIN
    DXL_ROT.complianceSlopeCW = 10*Kp +1;//P_P_GAIN
    DXL_ROT.complianceSlopeCCW = PWM_MIN; // PWM min    
    DXL_ROT.movingSpeed = PWM_MAX;   
    DXL_ROT.bendControlMode = 9;

}

// the loop routine runs over and over again forever:
void loop() 
{
  wdt_reset();
  DynamixelPoll();
  
  // Insert offset when calibrated
  newPos1 = Sensor1.getRawValue() - offsetPos1;
  if(newPos1<0){curPos1 = (newPos1 + 4096) / 4;}
  else{curPos1 = ( (newPos1)%4096 )/4;}
  
  newPos2 = Sensor2.getRawValue() - offsetPos2;
  if(newPos2<0){DXL_ROT.presentPosition = (newPos2 + 4096) / 4;}
  else{DXL_ROT.presentPosition = ( (newPos2)%4096 )/4;}
  
  // Don't respond on value 0 and 1023, which happen often with bad connection
  //if(DXL_SPIN.presentPosition == 0 || DXL_SPIN.presentPosition == 1023){DXL_SPIN.presentPosition = prevPos1;}
  //if(DXL_ROT.presentPosition == 0 || DXL_ROT.presentPosition == 1023){DXL_ROT.presentPosition = prevPos2;}
  if(~(spindleCalibFlag)){
	  if(curPos1 - prevPos1 > 900){revolutions1--;};
	  if(curPos1 - prevPos1 < -900){revolutions1++;};
	  spindleCalibFlag = false;
  }
  if(DXL_ROT.presentPosition - prevPos2 > 900){revolutions2--;};
  if(DXL_ROT.presentPosition - prevPos2 < -900){revolutions2++;};
  
  DXL_SPIN.presentPosition = curPos1; //+ 1024*revolutions1;
  
  
  if(millis()>khztime+1)
  { // 500 Hz
    khztime = millis();
    hztime++;
    hztime2++;
    if(hztime>49)    ///10Hz
    {
    	counterSpindle++;
    	counterRot++;
    	     if( (spindleFlag == true && counterSpindle > 20) || (counterSpindle > 2 && spindleFlag == true && abs(curPos1 - prevPrevPos1) < 1) )// two seconds
    	     {
    	    	 Motor_SPIN.setPWM(0);
    	    	 spindleFlag = false;
    	    	 
    	    	 if(DXL_SPIN.movingSpeed == 11) // Spindle going up
		{
    	    	  if(DXL_SPIN.LED == 11)
		  {
		    offsetPos1 = Sensor1.getRawValue() - 200; // 200/4 = 50
		    revolutions1 = 0;
		    spindleCalibFlag = true;
		  }
                  delay(100); // wait 100 ms before going down again
      	    	  DXL_SPIN.LED = 2;
		}

                if(DXL_SPIN.movingSpeed == 21)
                {
                  upFlag = false;
                }
    	    	     	    	 
    	    	 // Incidate if spindle is stopped by counter or by no change in position
    	    	 if(counterSpindle > 20)
		{
	    	 DXL_SPIN.movingSpeed = 0; // Counter stop
		} else {
		 DXL_SPIN.movingSpeed = 33; // No change in position stop
		}
    	     }
    	     
    	     
    	     if( (rotFlag == true && counterRot > 20) || (rotFlag == true && counterRot > 5 && abs(DXL_ROT.presentPosition - prevPrevPos2) < 2 ) )
    	     {
                 DXL_ROT.presentSpeed = 2;
                 Motor_ROT.setPWM(0);
                 
    	    	 if(counterRot > 20)
    	    	 {
    	    		DXL_ROT.presentSpeed = 2;
    	    	 } else {
    	    		DXL_ROT.presentSpeed = 3;
    	    	 }
    	    	 DXL_ROT.LED = 0;
		offsetPos2 = Sensor2.getRawValue() - 200; // 200/4 = 50;
		rotFlag = false;
    	     }
    	    	 
    	     
     
    	
     
     switch(DXL_SPIN.movingSpeed)
     {
		 case 0:
		 {
                         Motor_SPIN.setPWM(0);
			 break;
		 }
		 case 1: // Up motion
		 {
			 Motor_SPIN.setPWM(-255); // Upwards
			 
			 DXL_SPIN.movingSpeed = 11;
			 DXL_SPIN.LED = 11;
			 counterSpindle = 0;
			 spindleFlag = true;
			 upFlag = true;
			 break;
		 }
		 case 2: // Down motion
		 {
			 Motor_SPIN.setPWM(255); // Downwards
			 
			 DXL_SPIN.movingSpeed = 21;
			 counterSpindle = 0;
			 spindleFlag = true;
			 break;
		 }
		 default:
		 {
			 break;
		 }
     }
     
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
        
      hztime=0;
      nudgeTimeOut();
      toggle(LEDblue);
    } //10Hz

  } // 500Hz

  prevPrevPos1 = prevPos1;
  prevPrevPos2 = prevPos2;

  prevPos1 = curPos1;
  prevPos2 = DXL_ROT.presentPosition;
  
  // Update DXL values
  DXL_SPIN.torqueSetpoint   = abs(diffPos1);   // current dxl 1
  DXL_ROT.torqueSetpoint    = abs(diffPos2);   // current dxl 2
  
  DXL_SPIN.punch       = revolutions1;
  DXL_ROT.punch        = revolutions2;
  
  DXL_SPIN.presentLoad   = int(Motor_SPIN.getCurrent());   // current dxl 1
  DXL_ROT.presentLoad    = int(Motor_ROT.getCurrent());   // current dxl 2


  // Calibration or normal control mode motor 1 Spindle
  switch (DXL_SPIN.LED) // Calibration mode
  { 
    case 0:
    {
      break;
    }
    case 1:
    {
      DXL_SPIN.movingSpeed = 1;
      DXL_SPIN.LED = 11;
	  
      break;
    }
    case 2:
    {
      if ( abs(DXL_SPIN.presentPosition - DXL_SPIN.goalPosition) < 25 )
	  {
		DXL_SPIN.LED = 0;
		Motor_SPIN.setPWM(0);
	  }
      else if  (DXL_SPIN.presentPosition < DXL_SPIN.goalPosition)
      {    
    	Motor_SPIN.setPWM(200);  
      }
	  else 
	  {    
		Motor_SPIN.setPWM(-200); 
	  } 
      
      break;
    }
    case 11:
    {
      
      break;
    }
    default:
    {
      
      break;  
    }
  }

  ////// Rotation part
  // Calibration or normal control mode motor 2 Rotation
  switch(DXL_ROT.LED) 
  { // Calibration mode
	  case 1:
		  {
			  DXL_ROT.torqueEnable = 0;
			  if (upFlag){
				counterRot = 0;
				rotFlag = true;
				Motor_ROT.setPWM(-255);
				DXL_ROT.LED = 11;
			  } else {
				DXL_ROT.LED = 0;
			  }
			  break;
		  }
	  default:
	  {
		  break;
	  }
	  case 0:
	  {
		// Motor 2 - Rotation Motor
		// The rotation motor has a decreasing encoder value from its 0 radius to 10 cm radius.
		// The motor does not spin a full revolution.
		// If the upFlag is triggerd, it means that a measurement is busy and no rotation may take place.
		if (1 == DXL_ROT.torqueEnable){
		 if (upFlag){
		  DXL_ROT.bendControlMode = 1;
		  if (!(DXL_ROT.presentPosition == 1023 || DXL_ROT.presentPosition ==0)){
			calculatePWMSetpoint(&PIDInput,&PIDSetpoint,DXL_ROT.goalPosition);
			DXL_ROT.presentVoltage = abs(PIDOutput); 
			Motor_ROT.setPWM(-PIDOutput);
		  } 
		  else {
			DXL_ROT.bendControlMode = 2;  
			Motor_ROT.setPWM(0);
		  }
		 }
                 else {
                	 DXL_ROT.bendControlMode = 3; 
                   Motor_ROT.setPWM(0);
                 }
		} 
		else {
			DXL_ROT.bendControlMode = 4; 
		  Motor_ROT.setPWM(0);
		}
	  }
  }
}

// Dynamixel obtain data
void ProcessDynamixelData(const unsigned char ID, const int dataLength, const unsigned char* const Data){
  byte buffer[DYNAMIXEL_RETURN_SIZE];
  /// toggle(LEDred);
  switch(Data[0]){
  case 0x01: //ping
    ReturnDynamixelData(ID,0,0);
    break;
  case 0x02: //read

    if ( DXL_SPIN.ID == ID)
    {
      for(int n=0; n<Data[2];n++)
        buffer[n] = DXL_SPIN_ptr[Data[1]+n];
      //      buffer[n] = memCom[Data[1]+n];

      dataValid=1;
      delayMicroseconds(DXL_SPIN.returnDelay*2);
      ReturnDynamixelData(ID,Data[2],buffer);
      break; 
    }
    if ( DXL_ROT.ID == ID)
    {
      for(int n=0; n<Data[2];n++)
        buffer[n] = DXL_ROT_ptr[Data[1]+n];
      //    buffer[n] = memCom[Data[1]+n];
      dataValid=1;
      delayMicroseconds(DXL_ROT.returnDelay*2);
      ReturnDynamixelData(ID,Data[2],buffer);
      break; 
    }

  case 0x03: // Write command
    if(Data[1]<MEM_LENGTH && (dataLength-3)<(MEM_LENGTH-Data[1]))
    {// safeguard
      ///      for (int n=0;n<dataLength-4;n++){
      if ( DXL_SPIN.ID == ID)
      { 
        for (int n=0;n<dataLength-3;n++)
        {
          DXL_SPIN_ptr[(byte)Data[1]+n] = (byte)Data[2+n];
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


// Dynamixel send data
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

  unsigned char returnvalue = 0;
  // if (!dataValid) returnvalue |= (1<<6); // Instruction Error
  if (Motor_ROT.getError()) returnvalue |= (1<<5); // current overload if .getError() == 0
  if (Motor_SPIN.getError()) returnvalue |= (1<<4); // current overload if .getError() == 0
  if (upFlag) returnvalue |= (1<<0);
  return returnvalue;
}

int calculatePWMSetpoint(double* inputPID, double* setpointPID, int goalPos)
{
  // Update the setpoint, input and output value by pointer for the PID object
  // Goal pos ranges from 0 till 1023
    if(goalPos > DXL_ROT.angleLimitCCW)
    {
      goalPos = DXL_ROT.angleLimitCCW;
    } 
    else if(goalPos < DXL_ROT.angleLimitCW)
    {
    goalPos = DXL_ROT.angleLimitCW; 
    }

   *setpointPID = double(goalPos);
   *inputPID = DXL_ROT.presentPosition;
   PID_ROT.Compute(); 
}




