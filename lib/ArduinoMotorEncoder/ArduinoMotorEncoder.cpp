#include "ArduinoMotorEncoder.h"
#include <SPI.h>

//*****************************************************

const int encoder::chipSelectPin1 = 10;
const int encoder::chipSelectPin2 = 9;
const int encoder::chipSelectPin3 = 8;

long encoder::getEncoderValue(int encoder){
    unsigned int count1Value, count2Value, count3Value, count4Value;
    long result;
    
    encoder::selectEncoder(encoder);
    
    SPI.transfer(0x60); // Request count
    count1Value = SPI.transfer(0x00); // Read highest order byte
    count2Value = SPI.transfer(0x00);
    count3Value = SPI.transfer(0x00);
    count4Value = SPI.transfer(0x00); // Read lowest order byte
    
    encoder::deselectEncoder(encoder);
   
    result= ((long)count1Value<<24) + ((long)count2Value<<16) + ((long)count3Value<<8) + (long)count4Value;
    
    return result;
}//end func

void encoder::selectEncoder(int encoder){
  switch(encoder)
  {
     case 1:
        digitalWrite(encoder::chipSelectPin1,LOW);
        break;
     case 2:
       digitalWrite(encoder::chipSelectPin2,LOW);
       break;
     case 3:
       digitalWrite(encoder::chipSelectPin3,LOW);
       break;    
  }//end switch
  
}//end func

void encoder::deselectEncoder(int encoder){
  switch(encoder)
  {
     case 1:
        digitalWrite(encoder::chipSelectPin1,HIGH);
        break;
     case 2:
       digitalWrite(encoder::chipSelectPin2,HIGH);
       break;
     case 3:
       digitalWrite(encoder::chipSelectPin3,HIGH);
       break;    
  }//end switch
  
}//end func

// LS7366 Initialization and configuration
void encoder::LS7366_Init(void){
   
    // SPI initialization
    SPI.begin();
    //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
    delay(10);
   
   digitalWrite(encoder::chipSelectPin1,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(encoder::chipSelectPin1,HIGH); 
   
   
   digitalWrite(encoder::chipSelectPin2,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(encoder::chipSelectPin2,HIGH); 
   
   
   digitalWrite(encoder::chipSelectPin3,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(encoder::chipSelectPin3,HIGH); 
   
}//end func