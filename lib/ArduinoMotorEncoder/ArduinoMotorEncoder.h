/**
 * @file ArduinoMotorEncoder.h
 * @brief Library for the Arduino Motor Shield R3 and the LS7366R Quadrature Encoder
 * @author  ENGS 147 - Refactored by Alex Carney  
*/

#ifndef ArduinoMotorEncoder_h
#define ArduinoMotorEncoder_h

#include <Arduino.h>


namespace encoder {
    // Public Functions
    extern void LS7366_Init(void);
    extern void selectEncoder(int encoder);
    extern void deselectEncoder(int encoder);
    extern long getEncoderValue(int encoder);

    // Public Constants
    extern const int chipSelectPin1;
    extern const int chipSelectPin2;
    extern const int chipSelectPin3;
}



#endif //ArduinoMotorEncoder_h
