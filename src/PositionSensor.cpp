#include "Arduino.h"
// Stupid arduino glitch https://stackoverflow.com/questions/41093090/esp8266-error-macro-min-passed-3-arguments-but-takes-just-2
#undef max
#undef min
#include "PositionSensor.h"
#include <memory>



PositionSensor::PositionSensor(int pin, float *coefficients)
{
    const int N_COEFFICIENTS = 2;
    _pin = pin;
    _coefficients = new float[N_COEFFICIENTS];
    for (int i = 0; i < N_COEFFICIENTS; i++)
    {
        _coefficients[i] = coefficients[i];
    }
}

PositionSensor::~PositionSensor()
{
    delete[] _coefficients;
}

float PositionSensor::readDistanceCM()
{
    int analogValue = analogRead(_pin);
    float distance = _coefficients[0] * pow(analogValue, _coefficients[1]);
    return distance;
}

int PositionSensor::readDistanceRaw()
{
    return analogRead(_pin);
}
