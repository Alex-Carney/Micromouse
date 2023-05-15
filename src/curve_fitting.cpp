#include "curve_fitting.h"
#include <Arduino.h>

double speedCoefficients[6][2] = {{67.869, 249.736},
                                  {42.7, 72.06},
                                  {-0.301, 16.898},
                                  {15.1688, 65.818},
                                  {37.1683, -40.9424},
                                  {61.0135, -206.4433}};

double speedVoltageIntervals[7] = {-9.7, -8.18, -5.75, 0, 5.5, 7.75, 9.7};

double speedPwmCommandFromVoltage(double V)
{
    // Check if voltage is within the given interval
    if (V < speedVoltageIntervals[0])
    {
        Serial.println("Error: voltage is outside the valid interval.");
        return -400.0;
    }
    if (V > speedVoltageIntervals[6])
    {
        Serial.println("Error: voltage is outside the valid interval.");
        return 400.0;
    }
    // Find the interval that contains the voltage
    int interval = 0;
    while (V > speedVoltageIntervals[interval + 1])
    {
        interval++;
    }
    // Evaluate the corresponding linear fit at the given voltage
    double slope = speedCoefficients[interval][0];
    double intercept = speedCoefficients[interval][1];
    double result = slope * V + intercept;
    return result;
}
