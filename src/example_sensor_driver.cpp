#include "PositionSensor.h"
#include <Arduino.h>

const int sensor1Pin = A11;
const int sensor2Pin = A10;

// Characterized in MATLAB
// SENSOR 1 is the sensor with serial number 2003
float sensor1_coefficients[] = {2.2e4, -1.227};
// SENSOR 2 is the sensor with serial number 3006
float sensor2_coefficients[] = {1.18e4, -1.138};

// Instantiate position sensors
PositionSensor sensor1(sensor1Pin, sensor1_coefficients);
PositionSensor sensor2(sensor2Pin, sensor2_coefficients);

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    float distanceRAW = sensor2.readDistanceRaw();
    float distanceCM = sensor2.readDistanceCM();

    Serial.print("Distance in cm: ");
    Serial.print(distanceCM);
    Serial.println();

    Serial.print("Distance Raw: ");
    Serial.print(distanceRAW);
    Serial.println();

    delay(1000);
}
