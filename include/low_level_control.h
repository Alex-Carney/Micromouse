/**
 * @file low_level_control.h
 * @brief Low level control functions for the micromouse project
 * @authors Alex Carney, Youssef Marzouk, Nick Hagler
 */

#ifndef LOW_LEVEL_CONTROL_H
#define LOW_LEVEL_CONTROL_H

#include <Arduino.h>
#include "ArduinoMotorShieldR3.h"
#include "CircularBuffer.h"
#include "MotorCommand.h"

namespace ll_control
{
    /**
     * @brief Sets the motor speeds to the given value
     *
     * @author Youssef Marzouk
     */
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
        CircularBuffer<double>& control_buffer_m2);
    /**
     * @brief Sets the car's position to the given value
     * 
     * @author Nick Hagler
    */
    MotorCommand compensateMousePosition(
      double pos_left,
      double pos_right,
      long motor1_pos,
      long motor2_pos,
      long newTime,
      long oldTime,
      long encoder3Val_start_m1,
      long encoder3Val_start_m2,
      CircularBuffer<double>& control_buffer_m1,
      CircularBuffer<double>& control_buffer_m2);
}

#endif // LOW_LEVEL_CONTROL_H