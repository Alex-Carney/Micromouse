#ifndef PositionSensor_h
#define PositionSensor_h

#include <Arduino.h>

class PositionSensor
{
public:
    PositionSensor(int pin, float *coefficients);
    ~PositionSensor(); 
    float readDistanceCM();
    int readDistanceRaw();

private:
    int _pin;
    float *_coefficients;
};

#endif