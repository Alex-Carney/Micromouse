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

float PositionSensor::readDistanceCM(int num_averages)
{
    float distance = 0;
    for (int i = 0; i < num_averages; i++)
    {
        distance += readDistanceRaw();
    }
    distance /= num_averages;
    distance = _coefficients[0] * pow(distance, _coefficients[1]);
    return distance;
}

float PositionSensor::readDistanceCMMedian(int num_medians)
{
    float distances[num_medians];
    for (int i = 0; i < num_medians; i++)
    {
        distances[i] = readDistanceRaw();
    }

    // Sort the distances
    for (int i = 0; i < num_medians; i++)
    {
        for (int j = i + 1; j < num_medians; j++)
        {
            if (distances[i] > distances[j])
            {
                float temp = distances[i];
                distances[i] = distances[j];
                distances[j] = temp;
            }
        }
    }

    // Calculate the median
    float median;
    if (num_medians % 2 == 0)
    {
        median = (distances[num_medians/2 - 1] + distances[num_medians/2]) / 2.0;
    }
    else
    {
        median = distances[num_medians/2];
    }

    // Convert to distance
    median = _coefficients[0] * pow(median, _coefficients[1]);
    return median;

}

int PositionSensor::readDistanceRaw()
{
    return analogRead(_pin);
}
