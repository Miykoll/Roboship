/*****************************************************************************
 * Dynamixel Reader
 * 
 * All kinds of functions and variables having to do with reading Dynamixel
 * Messages.   
 * 
 * Call the function DynamixelPoll() regularly.
 * This will call 
 *   void ProcessDynamixelMessage(int id, unsigned char length, unsigned char *Data)
 *   which you will have to provide yourself, whenever a dynamixel message was
 *   received.   
 *   
 * As a user, you are responsible for initializing the Serial port (Serial.begin())    
 **********************************************************************/
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include "DynamixelReader.h"


//#define DEBUG (1) /* Do undef for no debugging*/

// Prototype for a function that the user will provide somewhere.
void ProcessDynamixelData(const unsigned char ID, const int dataLength, const unsigned char* const Data);
volatile int timeOutValue;
void fl();

typedef enum
{
  WaitingForFirstHeaderByte,
  WaitingForSecondHeaderByte,
  WaitingForIDByte,
  WaitingForDataLengthByte,
  WaitingForRestOfMessage
} 
e_receive_state;


void DynamixelPoll()  
{ 
  static e_receive_state ReceiveState;
  static unsigned char c;
  static char NumberOfDataBytesReceived = 0;
  static unsigned char addressBuffer = 0;
  static unsigned char Data[DYNAMIXEL_BUFFER_SIZE];
  static unsigned char DataLengthBuffer = 0;
  static unsigned int checksumBuffer = 0;

  if (Serial.available()>0)
  {
    c = Serial.read();   

    
//      digitalWrite(2, HIGH); 
//         Serial.write(c);
//         Serial.write(".");
//          Serial.flush();
//          digitalWrite(2, LOW);

    switch (ReceiveState)
    {
    case WaitingForFirstHeaderByte:
     if (c == 0xFF) // first byte of header  == (c == 255)
     {
          ReceiveState = WaitingForSecondHeaderByte;
//          digitalWrite(2, HIGH); 
//          Serial.print("WaitingForSecondHeaderByte\r\n");
//          Serial.flush();
//          digitalWrite(2, LOW);
      }

//    fl();
      break;
      
    case WaitingForSecondHeaderByte:
      if (c == 0xFF) 
      {
        //          toggle(LED_YELLOW);
        ReceiveState = WaitingForIDByte;
//         digitalWrite(2, HIGH); 
//          Serial.print("WaitingForIDByte\r\n");
//          Serial.flush();
//          digitalWrite(2, LOW);
      } 
      else 
      {
        ReceiveState = WaitingForFirstHeaderByte;
//          digitalWrite(2, HIGH); 
//          Serial.print("ELSE\r\n");
//          Serial.flush();
//          digitalWrite(2, LOW);
      }
//          fl();
      break;
      
    case WaitingForIDByte:
      // store byte

      addressBuffer = c;
//     if(addressBuffer != BOARD_ID)
     if((addressBuffer != BOARD_ID_1) && (addressBuffer != BOARD_ID_2))
      {
        ReceiveState=WaitingForFirstHeaderByte; 
//         
//          digitalWrite(2, HIGH); 
//         Serial.print("wrong ID(=1) !!\r\n");
//         Serial.print(c,HEX);
//         Serial.print("\tShould be:\t");
//         Serial.print(BOARD_ID,HEX);
//          Serial.flush();
//          digitalWrite(2, LOW);
              break;
      }
    
          
      
      checksumBuffer = c;
      ReceiveState = WaitingForDataLengthByte;
//       digitalWrite(2, HIGH); 
//          Serial.print("WaitingForDataLengthByte\r\n");
//          Serial.flush();
//          digitalWrite(2, LOW);
//      fl();
      break;
    case WaitingForDataLengthByte:      
                
      DataLengthBuffer = c;
      NumberOfDataBytesReceived = 0;
      checksumBuffer += c;
      ReceiveState = WaitingForRestOfMessage;
      
//      digitalWrite(2, HIGH); 
//          Serial.print("DataLength Buffer:\t");
//          Serial.print(c,DEC);
//            Serial.print("\r\nWaitingForRestOfMessage\r\n");
//          Serial.flush();
//          digitalWrite(2, LOW);
//      fl();
      break;
    case WaitingForRestOfMessage:
      
      if (NumberOfDataBytesReceived < DYNAMIXEL_BUFFER_SIZE)
      {
        Data[NumberOfDataBytesReceived] = c;
      } // // otherwise, we would have buffer overflow

      checksumBuffer += c;
      NumberOfDataBytesReceived++;
     
      if (NumberOfDataBytesReceived == (DataLengthBuffer-0)) // actually, larger can never happen, but just in case...
      {
        // Check if the data is correct by means of checksum.
        // Also, if we had a buffer overflow, we know for sure that the
        // data is invalid (because we only stored part of it, so in that case we
        // should not process it either. It would have been a faulty command anyway).
        if(((checksumBuffer & 0xFF) == 0xFF) && NumberOfDataBytesReceived<DYNAMIXEL_BUFFER_SIZE)
        { 
//           digitalWrite(2, HIGH); 
//          Serial.print("HOERA!\r\n->ProcessDXLData...\r\n");
//          Serial.print(checksumBuffer,DEC);
//          Serial.flush();
//          digitalWrite(2, LOW);
          
          timeOutValue=0;
          ProcessDynamixelData(addressBuffer, DataLengthBuffer, Data); 
        } 
        else
        {
//          digitalWrite(2, HIGH); 
//          Serial.print("CheckSum error\r\nCheck sum = ");
//          Serial.print(checksumBuffer,DEC);
//          Serial.flush();
//          digitalWrite(2, LOW);
        }
        ReceiveState = WaitingForFirstHeaderByte;
//          digitalWrite(2, HIGH); 
//          Serial.print("WaitingForFirstHeaderByte\r\n");
//          Serial.flush();
//          digitalWrite(2, LOW);
//      fl();
      break;
      }
//      fl();
      break;
    } // of switch

  }
  
}

int getTimeOut(void){
  return(timeOutValue);
}
void nudgeTimeOut(void){
 /// timeOutValue++;
}

void fl()
{
//   while (Serial.available()>0)
//      {
//        Serial.read();
//      }
}
