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
        CircularBuffer<double> &control_buffer_m1,
        CircularBuffer<double> &control_buffer_m2);
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
        CircularBuffer<double> &control_buffer_m1,
        CircularBuffer<double> &control_buffer_m2);

    /**
     * @brief Sets the car's rotational position to the given value
     *
     * @author Youssef Marzouk
     */
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
        BLA::Matrix<2, 1> yk,
        BLA::Matrix<2, 1> xk,
        BLA::Matrix<2, 1> xkp,
        BLA::Matrix<2, 1> zk,
        BLA::Matrix<2, 1> zkp,
        BLA::Matrix<4, 1> wk,
        BLA::Matrix<2, 1> u);
}

#endif // LOW_LEVEL_CONTROL_H