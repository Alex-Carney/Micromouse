#include <SPI.h>
#include "ArduinoMotorShieldR3.h"
#include "ArduinoMotorEncoder.h"
#include "CircularBuffer.h"
#include "low_level_control.h"
#include <Arduino.h>

/****************************************************/
/*
Main file for the micromouse project

Authors: Alex Carney, Majd Hamdan, Youssef Marzouk, Nick Hagler
*/
/****************************************************/

// Macros
// Set DEBUG_MODE to 1 to enable debugging
#define DEBUG_MODE 1

// Object definition prototypes
ArduinoMotorShieldR3 md;

// Constant definitions
const int ENCODER_M1 = 1;
const int ENCODER_M2 = 2;

// ------- Global variable definitions ----------- //
// Timing
long newTime = 0;
long oldTime = 0;
long lastSampleTime = 0;
// Global variables for DRIVING
namespace drive_control
{
    // Constants
    const int SAMPLING_PERIOD = 40000; // Sampling period in microseconds
    const double MAX_SPEED_RADS = 3;   // Maximum speed in radians per second

    // Globals
    long motor1_pos;
    long motor2_pos;
    long start_time;
    long encoder3Val_start_m2;
    long encoder3Val_start_m1;
    long time_buffer_m1[100];
    // 0: velocity, 1: error, 2: control effort
    CircularBuffer<double> control_buffer_m1(2, 3); // 2 steps back. 3 values per step
    CircularBuffer<double> control_buffer_m2(2, 3); // 2 steps back. 3 values per step
}
// State machine definitions
enum class MachineState
{
    INITIALIZING,
    DRIVING,
    DECELERATING,
    LOGIC,
    TURNING,
    PROCESS_STOPPED
};
MachineState MACHINE_STATE = MachineState::INITIALIZING;
bool isFirstStateIteration = true;

void setup()
{

    // Axis Encoder Init
    pinMode(encoder::chipSelectPin1, OUTPUT);
    pinMode(encoder::chipSelectPin2, OUTPUT);
    pinMode(encoder::chipSelectPin3, OUTPUT);

    digitalWrite(encoder::chipSelectPin1, HIGH);
    digitalWrite(encoder::chipSelectPin2, HIGH);
    digitalWrite(encoder::chipSelectPin3, HIGH);
    encoder::LS7366_Init();

    // Motor Initialization
    md.init();

// Peripheral Initialization
#ifdef DEBUG_MODE
    Serial.begin(115200);               // Initialize the Serial Port to view information on the Serial Monitor
    Serial.println("%Streaming in..."); // Countdown
    Serial.print("%3...");
    delay(1000); // Wait for a second
    Serial.print("%2...");
    delay(1000); // Wait for a second
    Serial.println("%1...");
    delay(1000); // Wait for a second
#endif
}

void loop()
{
    // Main loop
    switch (MACHINE_STATE)
    {
    case MachineState::INITIALIZING:
        // Run through initializing steps, then change to driving
        // Initialization
        MACHINE_STATE = MachineState::DRIVING;
        isFirstStateIteration = true;
        break;
    case MachineState::DRIVING:
        if (isFirstStateIteration)
        {
            // Initialize T and X
            oldTime = micros();
            drive_control::start_time = oldTime;
            lastSampleTime = oldTime;
            drive_control::encoder3Val_start_m1 = encoder::getEncoderValue(ENCODER_M1); // Starting value for the encoder
            drive_control::encoder3Val_start_m2 = encoder::getEncoderValue(ENCODER_M2);

            // Stop wheels just in case
            md.setM1Speed(0);
            md.setM2Speed(0);

            isFirstStateIteration = false;
        }

        newTime = micros();

        if (((newTime - lastSampleTime)) >= drive_control::SAMPLING_PERIOD)
        {
            // Collect data
            drive_control::motor1_pos = encoder::getEncoderValue(ENCODER_M1);
            drive_control::motor2_pos = encoder::getEncoderValue(ENCODER_M2);
            lastSampleTime = newTime;

            // Compensate wheel speed
            ll_control::updateWheelSpeed(
                drive_control::MAX_SPEED_RADS,
                drive_control::MAX_SPEED_RADS,
                drive_control::motor1_pos,
                drive_control::motor2_pos,
                newTime,
                oldTime,
                drive_control::encoder3Val_start_m1,
                drive_control::encoder3Val_start_m2,
                drive_control::control_buffer_m1,
                drive_control::control_buffer_m2,
                md);

            // Update data
            oldTime = newTime;
            drive_control::encoder3Val_start_m1 = drive_control::motor1_pos;
            drive_control::encoder3Val_start_m2 = drive_control::motor2_pos;
        }

        break;
    case MachineState::DECELERATING:
        // Initialization
        break;
    case MachineState::LOGIC:
        // Initialization
        break;
    case MachineState::TURNING:
        // Initialization
        break;
    case MachineState::PROCESS_STOPPED:
        // Initialization
        break;
    default:
        break;
    }
}