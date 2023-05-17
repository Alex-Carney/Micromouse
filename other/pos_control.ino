#include "NAxisMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include <SPI.h>
#include "ArduinoMotorShieldR3.h"
​
NAxisMotion mySensor;                 //Object that for the sensor
ArduinoMotorShieldR3 md;              //Object for the motor
long lastStreamTime = 0;              //To store the last streamed time stamp
const int streamPeriod = 40000;       //To stream at 25Hz without using additional timers (time period(us) =1000000/frequency(Hz))
bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream
const int enc_m1 = 1;
const int enc_m2 = 2;
const double r = 2*M_PI;                  //reference RPM -->rad/s
bool stream = false;                  //did I stream in the last loop? start false
const double K = 1.3;                 //proportional gain
bool END = false;
double send_m1 = 0;                       //what am I sending to PWM command
double send_m2 = 0;
long newTime = 0;
long oldTime = 0;
const int size = 600;
long nums_m1[size];
double nums2_m1[size][3];
long nums_m2[size];
double nums2_m2[size][3];
long pos_m1;
long start;
long encoder3Val_start_m1;
long encoder3Val_start_m2;
long pos_m2;
long enc_start_m1;
long enc_start_m2;
float  r_gain1 = 0.8;
float  r_gain2 = 0.7;
​
//long start_m2;
int i = 1;
//const int speeds[20] = {25, 50, 75, 100, 150, 200, 250, 300, 350, 400, -25, -50, -75, -100, -150, -200, -250, -300, -350, -400};
int s = 0;
​
​
const double a = 1.429;//1.8184; //coefficient for u[k-1]
const double b = .6202;//.2613; //coefficient for e[k]
const double c = 0.03096;//0.004857; //coefficient for e[k-1]
const double d = -.5892;//-.2564; //coefficient for e[k-2]
const double e = -.4286;//-0.8182; //coefficient for u[k-2]
​
​
const int chipSelectPin1=10;                //Int for x encoder
const int chipSelectPin2=9;                 //Int for y encoder
const int chipSelectPin3=8;           //Int for encoder channel 3
​
void setup() {
  //Peripheral Initialization
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor. 
  //Motor Init
  md.init();
  //Sensor Initialization
  mySensor.initSensor(0x28);          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  
  //Encoder init
  pinMode(chipSelectPin1, OUTPUT);
  pinMode(chipSelectPin2, OUTPUT);
  pinMode(chipSelectPin3, OUTPUT);
  
  digitalWrite(chipSelectPin1, HIGH);
  digitalWrite(chipSelectPin2, HIGH);
  digitalWrite(chipSelectPin3, HIGH);
  LS7366_Init();
  
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();
  updateSensorData = true;
  Serial.println();
  Serial.println("Default accelerometer configuration settings...");
  Serial.print("Range: ");
  Serial.println(mySensor.readAccelRange());
  Serial.print("Bandwidth: ");
  Serial.println(mySensor.readAccelBandwidth());
  Serial.print("Power Mode: ");
  Serial.println(mySensor.readAccelPowerMode());
  Serial.println("Streaming in ...");	//Countdown
  Serial.print("3...");
  delay(1000);	//Wait for a second
  Serial.print("2...");
  delay(1000);	//Wait for a second
  Serial.println("1...");
  delay(1000);	//Wait for a second
​
 md.setM1Speed(200);
 md.setM2Speed(-200);
 delay(1500);
    
  //md.setM1Speed(-400); //Start Motor channel 1 at 50% speed forward
  //delay(1000); //wait for motor to get upto speed
  oldTime = micros();
  start = oldTime;
  lastStreamTime = oldTime;
  encoder3Val_start_m1 = getEncoderValue(enc_m1); //Starting value for the encoder
  encoder3Val_start_m2 = getEncoderValue(enc_m2);
  enc_start_m1 = encoder3Val_start_m1;
  enc_start_m2 = encoder3Val_start_m2;
  encoder3Val_start_m1 = 0;
  encoder3Val_start_m2 = 0;
​
​
}
​
​
void loop() {
  // put your main code here, to run repeatedly:
​
  md.setM1Speed((int) send_m1);
  md.setM2Speed((int) send_m2);
​
​
  long newTime = micros();
  pos_m1 = getEncoderValue(enc_m1) - enc_start_m1;
  pos_m2 = getEncoderValue(enc_m2) - enc_start_m2;
​
​
  if(((newTime - lastStreamTime)) >= streamPeriod)
  {
    
    //nums[i] = { newTime, pos, ((encoder3Val_start - pos)/(newTime-oldTime)*60000) };
    
    lastStreamTime = newTime;
​
    nums_m1[i] = newTime;
    nums2_m1[i][0] = ((((double)encoder3Val_start_m1-(double)pos_m1)/1260.0)*360)*M_PI/180; //position in rad/s
    nums2_m1[i][1] = (r - nums2_m1[i][0]); //error signal in rad
    nums2_m1[i][2]  = (a*nums2_m1[i-1][2] + b*nums2_m1[i][1] + c*nums2_m1[i-1][1] + d*nums2_m1[i-2][1] + e*nums2_m1[i-1][2]) ; //control effort
    
​
    nums_m2[i] = newTime;
    nums2_m2[i][0] = ((((double)encoder3Val_start_m2 - (double)pos_m2)/1260.0)*360)*M_PI/180; //position in rad/s
    nums2_m2[i][1] = ((-r)-nums2_m2[i][0]); //error signal in rad/s
    nums2_m2[i][2]  = (a*nums2_m2[i-1][2] + b*nums2_m2[i][1] + c*nums2_m2[i-1][1] + d*nums2_m2[i-2][1] + e*nums2_m2[i-1][2]) ; //control effort
​
​
    //delay(1);
    oldTime = newTime;
    //encoder3Val_start_m1 = pos_m1;
    //encoder3Val_start_m2 = pos_m2;
​
    send_m1 = getU( nums2_m1[i][2] );
    send_m2 = getU( nums2_m2[i][2] );
​
    // Serial.print("velocity: ");
    // Serial.print(nums2_m1[i][0]);
    // Serial.println();
​
    // Serial.print("error: ");
    // Serial.print(nums2_m1[i][1]);
    // Serial.println();
​
    // Serial.print("U: ");
    // Serial.print(nums2_m1[i][2]);
    // Serial.println();
​
    // Serial.print("GetU: ");
    // Serial.print(send_m1);
    // Serial.println();
​
​
​
​
    
​
    i++;
​
    
    
    
    if (i >= size)
    {
      Serial.print("ABORT, REACHED THE END OF THE ARRAY");
      md.setM1Speed(0);
      md.setM2Speed(0);
      END = true;
      exit(0);
    }
​
    
  }   
​
  else if((newTime - start >= 9000000) || END)
  {
    
    //else{
    md.setM1Speed(0);
    md.setM2Speed(0);
    Serial.print("Max Values Recorded: ");
    Serial.print( i );
    Serial.println();
    
    /*for (int k = 0; k < i; k++)
    {
      Serial.print( nums_m1[k] );
      Serial.print(", ");
      for(int j = 0; j < 3; j++)
      {
        Serial.print(nums2_m1[k][j]);
        if(j<2)
          Serial.print(", ");
      }
      Serial.println();
    }  */ 
​
    
​
    Serial.print("\r\n");
    Serial.print("Error:");
  for (int i = 0; i < size; i++){
    Serial.print(nums2_m1[i][1]);
    Serial.print(" ");
    }
​
​
    Serial.print("\r\n");
    Serial.print("Position:");
  for (int i = 0; i < size; i++){
     Serial.print(nums2_m1[i][0]);
     Serial.print(" ");
  }
​
​
    Serial.print("\r\n");
    Serial.print("Controll Effort:");
​
     for (int i = 0; i < size; i++){
     Serial.print(nums2_m1[i][2]);
     Serial.print(" ");
  }
​
    Serial.print("\r\n");
    Serial.print("Time:");
for (int i = 0; i < size; i++){
     Serial.print(nums_m1[i]);
     Serial.print(" ");
  }
​
​
    exit(0);
    //}
    
  }
​
  
  
  
​
}
​
//*****************************************************  
double getU(double V)
//*****************************************************
{
  int result;
  //double V = scaled;
​
  if(V < -8.18 ){
    result = (67.869*V)+249.736;
    
    if(result <= -400){
      result = -400;
    }
​
  }
  else if((V >= -8.18) && (V<-5.75)){
    result = (42.7*V)+72.06;    
​
  }
  else if((V>=-5.75) && (V<-2.5)){
    
    result  =  (-0.301*V*V) + 16.898*V - 60.4455;  
  }
​
   else if((V>=-2.5) && (V<-0)){
    
    result  =  -150;
  }
​
  else if((V>=0.0) && (V<4)){
    
    result  = 150;
  }
​
   else if((V>=4) && (V<5.5)){
    
    result  = (15.1688*V) + 65.818;
  }
​
  else if((V>=5.5) && (V < 7.75)){
    
    result = 37.1683*V - 40.9424;
​
  }
​
​
  else if((V>=7.75)){
​
    result = 61.0135*V - 206.4433;
​
    if(result >= 400){
      result = 400;
    }
  }
​
​
  // int result;
  // //double V = scaled;
​
  // if(V <= 0 ){
  //   result = -(3.261*100000000)+ (3.261*100000000)*cos(V*0.0001591) - (4.495*10000)*sin(V*0.0001591);
    
  //   if(result <= -400){
  //     result = -400;
  //   }
​
  // }
  // else if(V > 0){
  //   result = (9.193*10000000) - (9.193*10000000)*cos(V*-0.0002888) + (2.121*10000)*sin(V*0.0002888); 
​
  //   if(result >= 400){
  //     result = 400;
  //   }
  // }
​
  // //Serial.print("PWM : ")
  //Serial.println(result);
  
    
  
  return result; 
​
​
}
​
​
//*****************************************************  
long getEncoderValue(int encoder)
//*****************************************************
{
    unsigned int count1Value, count2Value, count3Value, count4Value;
    long result;
    
    selectEncoder(encoder);
    
     SPI.transfer(0x60); // Request count
    count1Value = SPI.transfer(0x00); // Read highest order byte
    count2Value = SPI.transfer(0x00);
    count3Value = SPI.transfer(0x00);
    count4Value = SPI.transfer(0x00); // Read lowest order byte
    
    deselectEncoder(encoder);
   
    result= ((long)count1Value<<24) + ((long)count2Value<<16) + ((long)count3Value<<8) + (long)count4Value;
    
    return result;
}//end func
​
​
​
//*************************************************
void selectEncoder(int encoder)
//*************************************************
{
  switch(encoder)
  {
     case 1:
       digitalWrite(chipSelectPin1,LOW);
       break;
     case 2:
       digitalWrite(chipSelectPin2,LOW);
       break;
     case 3:
       digitalWrite(chipSelectPin3,LOW);
       break;     
  }//end switch
  
}//end func
​
​
​
//*************************************************
void deselectEncoder(int encoder)
//*************************************************
{
  switch(encoder)
  {
     case 1:
       digitalWrite(chipSelectPin1,HIGH);
       break;
     case 2:
       digitalWrite(chipSelectPin2,HIGH);
       break;
     case 3:
       digitalWrite(chipSelectPin3,HIGH);
       break;     
  }//end switch
  
}//end func
​
// LS7366 Initialization and configuration
//*************************************************
void LS7366_Init(void)
//*************************************************
{
   
    
    // SPI initialization
    SPI.begin();
    //SPI.setClockDivider(SPI_CLOCK_DIV16);      // SPI at 1Mhz (on 16Mhz clock)
    delay(10);
   
   
   digitalWrite(chipSelectPin1,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin1,HIGH); 
   
   
   digitalWrite(chipSelectPin2,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin2,HIGH);
   
   digitalWrite(chipSelectPin3,LOW);
   SPI.transfer(0x88); 
   SPI.transfer(0x03);
   digitalWrite(chipSelectPin3,HIGH);
   
}//end func