/**
 * @file MotorCommand.h
 * @brief Nothing but a simple struct for storing motor wheel speed commands
*/

#ifndef MOTORCOMMAND_H  // Include guards to prevent multiple inclusion
#define MOTORCOMMAND_H

struct MotorCommand {
    double motor1_pwm;
    double motor2_pwm;
    bool next_state;
};

#endif  // MOTORCOMMAND_H
