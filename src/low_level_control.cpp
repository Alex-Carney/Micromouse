#include "low_level_control.h"
#include "ArduinoMotorShieldR3.h"
#include "CircularBuffer.h"
#include "curve_fitting.h"
#include <Arduino.h>

// Set DEBUG_MODE to 1 to enable debugging
#define DEBUG_MODE 0

// Define a macro for PI
#define M_PI 3.14159265358979323846

// do function prototype ffor get U
double getU(double V);
namespace ll_control
{
    // DEBUGGING
    const int size = 20;
    int i = 2;
    double nums2_m2[size][3] = {0};

    // Compensator Constants
    const double a = 2;      // coefficient for u[k-1]
    const double b = -1;     // coefficient for u[k-2]
    const double c = 2.131;  // coefficient for e[k]
    const double d = -2.908; // coefficient for e[k-1]
    const double e = 0.9612; // coefficient for e[k-2]



    void updateWheelSpeed(
        double ref_right,
        double ref_left,
        long motor1_pos,
        long motor2_pos,
        long newTime,
        long oldTime,
        long encoder3Val_start_m1,
        long encoder3Val_start_m2,
        CircularBuffer<double>& control_buffer_m1,
        CircularBuffer<double>& control_buffer_m2,
        ArduinoMotorShieldR3& md)
    {


        // nums_m1[i] = newTime;
        // VELOCITY // ERROR // CONTROL EFFORT
        double motor1_values[3] = {
            (60000000.0 * ((encoder3Val_start_m1 - motor1_pos)) / ((double)(newTime - oldTime) * 1260.0)) * M_PI / 30,
            (ref_left - motor1_values[0]),
            (a * control_buffer_m1.getValue(1, 2) + b * control_buffer_m1.getValue(2, 2) + c * motor1_values[1] + d * control_buffer_m1.getValue(1, 1) + e * control_buffer_m1.getValue(2, 1))};

        double motor2_values[3] = {
            (60000000.0 * ((encoder3Val_start_m2 - motor2_pos)) / ((double)(newTime - oldTime) * 1260.0)) * M_PI / 30,
            ((-ref_right) - motor2_values[0]),
            (a * control_buffer_m2.getValue(1, 2) + b * control_buffer_m2.getValue(2, 2) + c * motor2_values[1] + d * control_buffer_m2.getValue(1, 1) + e * control_buffer_m2.getValue(2, 1))};

        control_buffer_m1.addValues(motor1_values);
        control_buffer_m2.addValues(motor2_values);



        // Serial.println("Printing second circular buffer");
        // control_buffer_m2.print();
        // Serial.println("Done Printing circular buffers");
        // only print if debug

        #ifdef DEBUG_MODE
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
        // double send_m1 = speedPwmCommandFromVoltage(nums2_m1[i][2]);
        //TODO: Fix this later 
        // double send_m1 = speedPwmCommandFromVoltage(motor1_values[2]);
        // double send_m2 = speedPwmCommandFromVoltage(motor2_values[2]);

        double toTest = getU(motor1_values[2]);
        double toTest2 = getU(motor2_values[2]);

        md.setM1Speed((int)toTest);
        md.setM2Speed((int)toTest2);

        i++;

        return;
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
