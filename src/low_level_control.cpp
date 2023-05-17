#include "low_level_control.h"
#include "ArduinoMotorShieldR3.h"
#include "CircularBuffer.h"
#include "curve_fitting.h"
#include "MotorCommand.h"
#include <Arduino.h>

// Set DEBUG_MODE to 1 to enable debugging
#define DEBUG_MODE 0

// Define a macro for PI
#define M_PI 3.14159265358979323846

// internal function prototypes
double getU(double V);
double getUForPos(double V);

namespace ll_control
{
    // DEBUGGING
    const int size = 20;
    int i = 2;
    double nums2_m2[size][3] = {0};

    // Compensator Constants for Driving
    namespace speed_compensation {
      const double a = 2;      // coefficient for u[k-1]
      const double b = -1;     // coefficient for u[k-2]
      const double c = 2.131;  // coefficient for e[k]
      const double d = -2.908; // coefficient for e[k-1]
      const double e = 0.9612; // coefficient for e[k-2]
    }

    // Compensator Constants for Turning

    // Compensator Constants Position Control
    namespace position_compensation {
      const double a = 1.429;  // coefficient for u[k-1]
      const double b = .6202; // coefficient for e[k]
      const double c = .03096; // coefficient for e[k-1]
      const double d = -.5892; // coefficient for e[k-2]
      const double e = -.4286; // coefficient for u[k-2]
    }

    MotorCommand compensateMousePosition(
      double pos_ref,
      long motor1_pos,
      long motor2_pos,
      long newTime,
      long oldTime,
      long encoder3Val_start_m1,
      long encoder3Val_start_m2,
      CircularBuffer<double>& control_buffer_m1,
      CircularBuffer<double>& control_buffer_m2)
    {

    double motor1_values[3] = {
      ((((double)encoder3Val_start_m1-(double)motor1_pos)/1260.0)*360)*M_PI/180, //position in rad/s
      (r - motor1_values[0]), //error signal in rad
      (a*control_buffer_m1.getValue(1, 2) + b*motor1_values[1] + c*control_buffer_m1.getValue(1,1) + d*control_buffer_m1.getValue(2, 1) + e*control_buffer_m1.getValue(1, 2)) //control effort
    };

    double motor2_values[3] = {
      ((((double)encoder3Val_start_m2 - (double)motor2_pos)/1260.0)*360)*M_PI/180, //position in rad/s
      ((-r)-motor2_values[0]), //error signal in rad/s
      (a*control_buffer_m2.getValue(1, 2) + b*motor2_values[1] + c*control_buffer_m2.getValue(1, 1) + d*control_buffer_m2.getValue(2, 1) + e*control_buffer_m2.getValue(1, 2)) //control effort
    };

    MotorCommand command;
    command.motor1_pwm = getUForPos(motor1_values[2]);
    command.motor2_pwm = getUForPos(motor2_values[2]);

    return command;


    }
    




    MotorCommand compensateDriveSpeed(
        double ref_right,
        double ref_left,
        long motor1_pos,
        long motor2_pos,
        long newTime,
        long oldTime,
        long encoder3Val_start_m1,
        long encoder3Val_start_m2,
        CircularBuffer<double>& control_buffer_m1,
        CircularBuffer<double>& control_buffer_m2)
    {
        // nums_m1[i] = newTime;
        // VELOCITY // ERROR // CONTROL EFFORT
        double motor1_values[3] = {
            (60000000.0 * ((encoder3Val_start_m1 - motor1_pos)) / ((double)(newTime - oldTime) * 1260.0)) * M_PI / 30,
            (ref_left - motor1_values[0]),
            (speed_compensation::a * control_buffer_m1.getValue(1, 2) + speed_compensation::b * control_buffer_m1.getValue(2, 2) + speed_compensation::c * motor1_values[1] + speed_compensation::d * control_buffer_m1.getValue(1, 1) + speed_compensation::e * control_buffer_m1.getValue(2, 1))};

        double motor2_values[3] = {
            (60000000.0 * ((encoder3Val_start_m2 - motor2_pos)) / ((double)(newTime - oldTime) * 1260.0)) * M_PI / 30,
            ((-ref_right) - motor2_values[0]),
            (speed_compensation::a * control_buffer_m2.getValue(1, 2) + speed_compensation::b * control_buffer_m2.getValue(2, 2) + speed_compensation::c * motor2_values[1] + speed_compensation::d * control_buffer_m2.getValue(1, 1) + speed_compensation::e * control_buffer_m2.getValue(2, 1))};

        control_buffer_m1.addValues(motor1_values);
        control_buffer_m2.addValues(motor2_values);



        // Serial.println("Printing second circular buffer");
        // control_buffer_m2.print();
        // Serial.println("Done Printing circular buffers");
        // only print if debug

        #ifdef DEBUG_MODE
        #if DEBUG_MODE == 1
        Serial.println("Printing circular buffers");
        control_buffer_m1.print();
        Serial.println("Values from Buffer");
        Serial.print(motor1_values[0]);
        Serial.print(" ");
        Serial.print(motor1_values[1]);
        Serial.print(" ");
        Serial.print(motor1_values[2]);
        Serial.println();
        #endif
        #endif
        // double send_m1 = speedPwmCommandFromVoltage(nums2_m1[i][2]);
        //TODO: Fix this later 
        // double send_m1 = speedPwmCommandFromVoltage(motor1_values[2]);
        // double send_m2 = speedPwmCommandFromVoltage(motor2_values[2]);

        MotorCommand command;
        command.motor1_pwm = getU(motor1_values[2]);
        command.motor2_pwm = getU(motor2_values[2]);

        return command;
    }
}

//*****************************************************
double getU(double V)
//*****************************************************
{
  int result;
  // double V = scaled;

  if (V < -8.18)
  {
    result = (67.869 * V) + 249.736;

    if (result <= -400)
    {
      result = -400;
    }
  }
  else if ((V >= -8.18) && (V < -5.75))
  {
    result = (42.7 * V) + 72.06;
  }
  else if ((V >= -5.75) && (V < -0))
  {

    result = (-0.301 * V * V) + 16.898 * V - 60.4455;
  }

  else if ((V >= 0.0) && (V < 5.5))
  {

    result = (15.1688 * V) + 65.818;
  }

  else if ((V >= 5.5) && (V < 7.75))
  {

    result = 37.1683 * V - 40.9424;
  }

  else if ((V >= 7.75))
  {

    result = 61.0135 * V - 206.4433;

    if (result >= 400)
    {
      result = 400;
    }
  }

  return result;
}

//*****************************************************  
double getUForPos(double V)
//*****************************************************
{
  int result;
  //double V = scaled;
  if(V < -8.18 ){
    result = (67.869*V)+249.736;
    
    if(result <= -400){
      result = -400;
    }
  }
  else if((V >= -8.18) && (V<-5.75)){
    result = (42.7*V)+72.06;    
  }
  else if((V>=-5.75) && (V<-2.5)){
    
    result  =  (-0.301*V*V) + 16.898*V - 60.4455;  
  }
   else if((V>=-2.5) && (V<-0)){
    
    result  =  -150;
  }
  else if((V>=0.0) && (V<4)){
    
    result  = 150;
  }
   else if((V>=4) && (V<5.5)){
    
    result  = (15.1688*V) + 65.818;
  }
  else if((V>=5.5) && (V < 7.75)){
    
    result = 37.1683*V - 40.9424;
  }
  else if((V>=7.75)){
    result = 61.0135*V - 206.4433;
    if(result >= 400){
      result = 400;
    }
  }
  return result; 
}