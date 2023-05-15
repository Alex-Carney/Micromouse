#include "low_level_control.h"
#include "ArduinoMotorShieldR3.h"
#include "CircularBuffer.h"
#include "curve_fitting.h"
#include <Arduino.h>

// Define a macro for PI
#define M_PI 3.14159265358979323846

namespace ll_control
{
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
        CircularBuffer<double> control_buffer_m1,
        CircularBuffer<double> control_buffer_m2,
        ArduinoMotorShieldR3 md)
    {

        // nums_m1[i] = newTime;
        // VELOCITY // ERROR // CONTROL EFFORT
        double motor1_values[3] = {
            (60000000.0 * ((encoder3Val_start_m1 - motor1_pos)) / ((double)(newTime - oldTime) * 1260.0)) * M_PI / 30,
            (ref_left - motor1_values[0]),
            (a * control_buffer_m1.getValue(1, 2) + b * control_buffer_m1.getValue(2, 2) + c * motor1_values[1] + d * control_buffer_m1.getValue(1, 1) + e * control_buffer_m1.getValue(2, 1))};

        // nums2_m1[i][0] = (60000000.0 * ((encoder3Val_start_m1 - motor1_pos)) / ((double)(newTime - oldTime) * 1260.0)) * M_PI / 30; // velocity in rad/s
        // nums2_m1[i][1] = (ref_left - nums2_m1[i][0]);                                                                               // error signal in rad/s

        // nums2_m1[i][2] = (a * nums2_m1[i - 1][2] + b * nums2_m1[i - 2][2] + c * nums2_m1[i][1] + d * nums2_m1[i - 1][1] + e * nums2_m1[i - 2][1]); // control effort

        // nums_m2[i] = newTime;

        double motor2_values[3] = {
            (60000000.0 * ((encoder3Val_start_m2 - motor2_pos)) / ((double)(newTime - oldTime) * 1260.0)) * M_PI / 30,
            ((-ref_right) - motor2_values[0]),
            (a * control_buffer_m2.getValue(1, 2) + b * control_buffer_m2.getValue(2, 2) + c * motor2_values[1] + d * control_buffer_m2.getValue(1, 1) + e * control_buffer_m2.getValue(2, 1))};

        // nums2_m2[i][0] = (60000000.0 * ((encoder3Val_start_m2 - motor2_pos)) / ((double)(newTime - oldTime) * 1260.0)) * M_PI / 30; // velocity in rad/s
        // nums2_m2[i][1] = ((-ref_right) - nums2_m2[i][0]);                                                                           // error signal in rad/s

        // nums2_m2[i][2] = (a * nums2_m2[i - 1][2] + b * nums2_m2[i - 2][2] + c * nums2_m2[i][1] + d * nums2_m2[i - 1][1] + e * nums2_m2[i - 2][1]); // control effort

        // delay(1);
        // encoder3Val_start_m1 = pos_m1;
        // encoder3Val_start_m2 = pos_m2;

        control_buffer_m1.addValues(motor1_values);
        control_buffer_m2.addValues(motor2_values);

        // double send_m1 = speedPwmCommandFromVoltage(nums2_m1[i][2]);
        double send_m1 = speedPwmCommandFromVoltage(motor2_values[1]);
        double send_m2 = speedPwmCommandFromVoltage(motor2_values[2]);

        md.setM1Speed((int)send_m1);
        md.setM2Speed((int)send_m2);

        return;
    }
}
