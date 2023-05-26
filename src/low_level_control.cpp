#include "low_level_control.h"
#include "ArduinoMotorShieldR3.h"
#include "BasicLinearAlgebra.h"
#include "CircularBuffer.h"
#include "curve_fitting.h"
#include "MotorCommand.h"
#include <Arduino.h>
#include <Wire.h>
#include "macros.h"



// Set DEBUG_MODE to 1 to enable debugging
#define DEBUG_MODE 0


// Define a macro for PI
#define M_PI 3.14159265358979323846

// internal function prototypes
double getU(double V);
double getUForPos(double V);
double getU_R(double R);
float unwrap_Heading(float previous_angle, float new_angle);

namespace ll_control
{
  // DEBUGGING
  const int size = 20;
  int i = 2;
  double nums2_m2[size][3] = {0};

  // Compensator Constants for Driving
  namespace speed_compensation
  {



    //CONSTANSTS FOR FAST SPEED
    #if SPEED_MODE == 1
      const double a = 2; //coefficient for u[k-1]
      const double b = -1;  //coefficient for u[k-2]
      const double c = 1.473; //coefficient for e[k]
      const double d = -2.592; //coefficient for e[k-1]
      const double e = 1.135; //coefficient for e[k-2]
    #else
      const double a = 2;      // coefficient for u[k-1]
      const double b = -1;     // coefficient for u[k-2]
      const double c = 2.131;  // coefficient for e[k]
      const double d = -2.908; // coefficient for e[k-1]
      const double e = 0.9612; // coefficient for e[k-2]
    #endif
  }

  // Compensator Constants for Turning

  // Compensator Constants Position Control
  namespace position_compensation
  {
    const double a = 1.81;          // coefficient for u[k-1]
    const double b = 1.18 * 0.2516; //.1936; // coefficient for e[k]
    const double c = 1.18 * 0.0032; //.002463; // coefficient for e[k-1]
    const double d = 1.18 * -.2484; //-.1912; // coefficient for e[k-2]
    const double e = -.8182;        // coefficient for u[k-2]
  }

  MotorCommand compensateMousePosition(
      double pos_left,
      double pos_right,
      long motor1_pos,
      long motor2_pos,
      long newTime,
      long oldTime,
      long encoder3Val_start_m1,
      long encoder3Val_start_m2,
      CircularBuffer<double> &control_buffer_m1,
      CircularBuffer<double> &control_buffer_m2)
  {

    double motor1_values[3] = {
        ((((double)encoder3Val_start_m1 - (double)motor1_pos) / 1260.0) * 360) * M_PI / 180,                                                                                                                                                                                                                  // position in rad/s
        (pos_left - motor1_values[0]),                                                                                                                                                                                                                                                                        // error signal in rad
        (position_compensation::a * control_buffer_m1.getValue(1, 2) + position_compensation::b * motor1_values[1] + position_compensation::c * control_buffer_m1.getValue(1, 1) + position_compensation::d * control_buffer_m1.getValue(2, 1) + position_compensation::e * control_buffer_m1.getValue(1, 2)) // control effort
    };

    double motor2_values[3] = {
        ((((double)encoder3Val_start_m2 - (double)motor2_pos) / 1260.0) * 360) * M_PI / 180,                                                                                                                                                                                                                  // position in rad/s
        ((-pos_right) - motor2_values[0]),                                                                                                                                                                                                                                                                    // error signal in rad/s
        (position_compensation::a * control_buffer_m2.getValue(1, 2) + position_compensation::b * motor2_values[1] + position_compensation::c * control_buffer_m2.getValue(1, 1) + position_compensation::d * control_buffer_m2.getValue(2, 1) + position_compensation::e * control_buffer_m2.getValue(1, 2)) // control effort
    };

    MotorCommand command;
    command.motor1_pwm = getUForPos(motor1_values[2]);
    command.motor2_pwm = getUForPos(motor2_values[2]);

    // print out the error values, motor1_values[1] and motor2_values[1]

    if (abs(motor1_values[1]) < .1 && abs(motor2_values[1]) < .1)
    {
      command.next_state = true;
    }
    else
    {
      command.next_state = false;
    }

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
      CircularBuffer<double> &control_buffer_m1,
      CircularBuffer<double> &control_buffer_m2)
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
    // TODO: Fix this later
    // double send_m1 = speedPwmCommandFromVoltage(motor1_values[2]);
    // double send_m2 = speedPwmCommandFromVoltage(motor2_values[2]);

    MotorCommand command;
    command.motor1_pwm = getU(motor1_values[2]);
    command.motor2_pwm = getU(motor2_values[2]);
    command.next_state = false;

    return command;
  }

  MotorCommand compensateTurning(
      long motor1_pos,
      long motor2_pos,
      long newTime,
      long oldTime,
      long encoder3Val_start_m1,
      long encoder3Val_start_m2,
      double turn_start,
      double heading_meas,
      BLA::Matrix<2, 2> A,
      BLA::Matrix<2, 2> B,
      BLA::Matrix<2, 2> C,
      BLA::Matrix<2, 4> F_bar,
      BLA::Matrix<2, 1> y_star,
      BLA::Matrix<2, 1>& yk,
      BLA::Matrix<2, 1>& xk,
      BLA::Matrix<2, 1>& xkp,
      BLA::Matrix<2, 1>& zk,
      BLA::Matrix<2, 1>& zkp,
      BLA::Matrix<4, 1>& wk,
      BLA::Matrix<2, 1>& u)
  {
    // Measure velocity
    float vel_r = (60000000.0 * ((encoder3Val_start_m1 - motor1_pos)) / ((double)(newTime - oldTime) * 1260.0)) * M_PI / 30;
    float vel_l = (60000000.0 * ((encoder3Val_start_m2 - motor2_pos)) / ((double)(newTime - oldTime) * 1260.0)) * M_PI / 30;

    // Update measurement vector with linear velocity and angular velocity
    // TODO: Look at warning here
    yk = {-(0.0175 * vel_r) + (0.0175 * vel_l), unwrap_Heading(yk(0,1), heading_meas)};

    // Predict next time step
    xkp = A * xk + (B * u);
    zkp = zk + y_star - yk;

    // set to old time step
    xk = xkp;
    zk = zkp;

    wk = xk && zk;

    // calculate control effort
    u = (F_bar * wk);


    MotorCommand command;
    command.motor1_pwm = getU_R(u(0, 0));
    command.motor2_pwm = getU_R(-u(0, 1));
    command.next_state = false;

    return command;
  }

  float unwrap_Heading(float previous_angle, float new_angle)
  {
    float d = new_angle - previous_angle;
    d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
    return previous_angle + d;

  }

}

// Get PWM from Voltage control effort (DRIVING CONTROL)
//*****************************************************
double getU(double V)
//*****************************************************
{
  double result = 0;
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

// Get PWM output for Voltage control effort (WITH DITHERING)
//*****************************************************
double getUForPos(double V)
//*****************************************************
{
  double result = 0;

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
  else if ((V >= -5.75) && (V < -2.5))
  {

    result = (-0.301 * V * V) + 16.898 * V - 60.4455;
  }
  else if ((V >= -2.5) && (V < -0))
  {

    result = -150;
  }
  else if ((V >= 0.0) && (V < 4))
  {

    result = 150;
  }
  else if ((V >= 4) && (V < 5.5))
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

// Get PWM for RADIAN/s control effort
//*****************************************************
double getU_R(double R)
//*****************************************************
{
  double res = 0;
  // double V = scaled;

  if (R <= -4.5)
  {
    res = (R * R * R * R * 0.0086) + (R * R * R * 0.3524) + (R * R * 3.4233) + (R * 18.421) - (62.1883);

    if (res <= -400)
    {
      res = -400;
    }

    if ((res < -80) && (res > -130))
    {
      res = -132;
    }
  }

  else
  {
    res = -(R * R * R * R * 0.01) + (R * R * R * 0.4019) - (R * R * 4.0489) + (R * 21.1079) + (62.0307);

    if (res >= 400)
    {
      res = 400;
    }

    if ((res > 80) && (res < 130))
    {
      res = 132;
    }
  }

  return res;
}



  



