#include "Arduino.h"
#include "PositionSensor.h"

const int N_COEFFICIENTS = 2;

PositionSensor::PositionSensor(int pin, float *coefficients)
{
    _pin = pin;
    _coefficients = new float[N_COEFFICIENTS];
    for (int i = 0; i < N_COEFFICIENTS; i++)
    {
        _coefficients[i] = coefficients[i];
    }
}

float PositionSensor::readDistanceCM()
{
    int analogValue = analogRead(_pin);

    // TODO: Implement the conversion from analog value to distance in cm
    float distance = _coefficients[0] * pow(analogValue, _coefficients[1]);

    return distance;
}

int PositionSensor::readDistanceRaw()
{
    return analogRead(_pin);
}
