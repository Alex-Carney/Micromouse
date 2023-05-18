#include "PositionSensor.h"
#include <Arduino.h>

const int sensor1Pin = A11;
const int sensor2Pin = A10;
const int sensor3Pin = A9;

// Characterized in MATLAB
// SENSOR 1 is the sensor with with a single tick on the right
float sensor1_coefficients[] = {2.2e4, -1.227};
// SENSOR 2 is the sensor with two tick marks on it (long)
float sensor2_coefficients[] = {1.18e4, -1.138};
// SENSOR 3 is the sensor with the big circle on it
float sensor3_coefficients[] = {1.3e4, -1.149};


// Instantiate position sensors
PositionSensor sensor_mini_right(sensor1Pin, sensor1_coefficients);
PositionSensor sensor_marks(sensor2Pin, sensor2_coefficients);
PositionSensor sensor_big_circle(sensor3Pin, sensor3_coefficients);

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    float distanceRAW = sensor_big_circle.readDistanceRaw();
    float distanceCM = sensor_big_circle.readDistanceCM();

    Serial.print("Distance in cm: ");
    Serial.print(distanceCM);
    Serial.println();

    Serial.print("Distance Raw: ");
    Serial.print(distanceRAW);
    Serial.println();

    delay(1000);
}
