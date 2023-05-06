#include <Arduino.h>

const int IR_SENSOR_PIN = A0;
bool IS_RUNNING = true;

const int STREAM_PERIOD = 100000;     // 100 ms
const long SAMPLE_DURATION = 1000000; // 1 s
const int BUFFER_SIZE = (SAMPLE_DURATION / STREAM_PERIOD);

// Namespace Variables - Mutable
long currentTimeMicros = 0;
long lastStreamTime = 0;

// Define prototype for each function
void print_ir_characterization();
template <typename T>
void __printArray(T arr[]);

void setup()
{
    // Peripheral initialization
    Serial.begin(9600);
}

void loop()
{
    if (IS_RUNNING == true)
    {
        print_ir_characterization();
    }

    IS_RUNNING = false;
}

void print_ir_characterization()
{

    long timeStartMicros = micros();
    long *sensor_readings = new long[BUFFER_SIZE];
    int currentArrayIndex = 0;

    do
    {
        currentTimeMicros = micros() - timeStartMicros;
        if (currentTimeMicros - lastStreamTime >= STREAM_PERIOD)
        {
            lastStreamTime = currentTimeMicros;
            sensor_readings[currentArrayIndex] = analogRead(IR_SENSOR_PIN);
            currentArrayIndex++;
        }

    } while (currentTimeMicros < SAMPLE_DURATION);

    __printArray(sensor_readings);
}

template <typename T>
void __printArray(T arr[])
{
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        Serial.print(arr[i]);
        if (i < BUFFER_SIZE - 1)
        {
            Serial.print(", ");
        }
    }
    Serial.println();
}