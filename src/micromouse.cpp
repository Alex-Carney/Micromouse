#include <SPI.h>
#include "ArduinoMotorShieldR3.h"
#include "ArduinoMotorEncoder.h"
#include "CircularBuffer.h"
#include "low_level_control.h"
#include "PositionSensor.h"
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
#define WALL_FOLLOW 1

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

namespace traversal
{
    // Constants
    const int PATH_DISTANCE_THRESHOLD = 10; // (cm) Distance threshold for path detection
    const int INTERSECTION_WALK_DISTANCE = 10; // (cm) Distance to walk once intersection is found

    // Globals
    // For Path managing and maze traversal
    int path_right = -1;
    int path_left = -1;
    int path_forward = -1;
    float pre_distance_left = 0;
    float pre_distance_right = 0;

}

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
    double motor1_reference_speed;
    double motor2_reference_speed;
    long encoder3Val_start_m2;
    long encoder3Val_start_m1;
    long time_buffer_m1[100];
    // 0: velocity, 1: error, 2: control effort
    CircularBuffer<double> control_buffer_m1(2, 3); // 2 steps back. 3 values per step
    CircularBuffer<double> control_buffer_m2(2, 3); // 2 steps back. 3 values per step
    float sensor1_coefficients[] = {2.2e4, -1.227};
    PositionSensor sensor_mini_right((int)A10, sensor1_coefficients);
    float sensor2_coefficients[] = {1.18e4, -1.138};
    PositionSensor sensor_marks((int)A11, sensor2_coefficients);
    const double DEL_X_GAIN = 0.1;
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
#if DEBUG_MODE == 1
    Serial.begin(115200);               // Initialize the Serial Port to view information on the Serial Monitor
    Serial.println("%Streaming in..."); // Countdown
    Serial.print("%3...");
    delay(1000); // Wait for a second
    Serial.print("%2...");
    delay(1000); // Wait for a second
    Serial.println("%1...");
    delay(1000); // Wait for a second
#endif
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
            //Serial.println("Got into driving state");
            // Initialize T and X
            oldTime = micros();
            drive_control::start_time = oldTime;
            lastSampleTime = oldTime;
            drive_control::encoder3Val_start_m1 = encoder::getEncoderValue(ENCODER_M1); // Starting value for the encoder
            drive_control::encoder3Val_start_m2 = encoder::getEncoderValue(ENCODER_M2);

            // Stop wheels just in case
            md.setM1Speed(0);
            md.setM2Speed(0);

            // State transition logic: Record distances right, left forward.
            traversal::pre_distance_right = drive_control::sensor_mini_right.readDistanceCM();
            traversal::pre_distance_left = drive_control::sensor_marks.readDistanceCM();

            isFirstStateIteration = false;
        }

        newTime = micros();

        if (((newTime - lastSampleTime)) >= drive_control::SAMPLING_PERIOD)
        {
            drive_control::motor1_pos = encoder::getEncoderValue(ENCODER_M1);
            drive_control::motor2_pos = encoder::getEncoderValue(ENCODER_M2);
            //Serial.println("Time to sample");
            // Collect data

            lastSampleTime = newTime;

            // Measure distances to both walls, in CM 
            float dist_left_cm = drive_control::sensor_mini_right.readDistanceCM();
            float dist_right_cm = drive_control::sensor_marks.readDistanceCM();
            float dist_diff = dist_left_cm - dist_right_cm;

            // Calculate Del W_ref from Del X
            double delWRef = drive_control::DEL_X_GAIN * dist_diff;

            #ifdef DEBUG_MODE
            #if DEBUG_MODE == 1
            Serial.print("DISTANCE RIGHT: ");
            Serial.println(dist_right_cm);
            Serial.print("DISTANCE LEFT: ");
            Serial.println(dist_left_cm);
            Serial.print("DISTANCE DIFFERENTIAL: ");
            Serial.println(dist_diff);
            Serial.print("DEL W REF: ");
            Serial.println(delWRef);
            #endif
            #endif

            // Calculate new reference speeds - only if wall following on 
            #ifdef WALL_FOLLOW
            #if WALL_FOLLOW == 1
            drive_control::motor1_reference_speed = min(drive_control::MAX_SPEED_RADS + delWRef, 4.0);
            drive_control::motor2_reference_speed = min(drive_control::MAX_SPEED_RADS - delWRef, 4.0);
            #else
            drive_control::motor1_reference_speed = drive_control::MAX_SPEED_RADS;
            drive_control::motor2_reference_speed = drive_control::MAX_SPEED_RADS;
            #endif
            #endif







            // Compensate wheel speed
            ll_control::updateWheelSpeed(
                drive_control::motor1_reference_speed,
                drive_control::motor2_reference_speed,
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


            // State transition logic: Record distances right, left forward.
            int intersection_found = 0;
            if (dist_left_cm - traversal::pre_distance_left >  traversal::PATH_DISTANCE_THRESHOLD)
            {
                // double check its not a glitch reading
                if (drive_control::sensor_marks.readDistanceCM() - traversal::pre_distance_left >  traversal::PATH_DISTANCE_THRESHOLD){
                    intersection_found = 1;
                    traversal::path_left = 1;
                    traversal::pre_distance_left = dist_left_cm;
                }  
                drive_control::sensor_mini_right.readDistanceCM();
            }
            if (dist_right_cm - traversal::pre_distance_right > traversal::PATH_DISTANCE_THRESHOLD){
                // double check its not a glitch reading
                if ( drive_control::sensor_mini_right.readDistanceCM() - traversal::pre_distance_right > traversal::PATH_DISTANCE_THRESHOLD){
                    intersection_found = 1;
                    traversal::path_right = 1;
                    traversal::pre_distance_right = dist_right_cm;
                }
            }

            if(intersection_found){
                // Stop motors
                md.setM1Speed(0);
                md.setM2Speed(0);
                // Update state
                MACHINE_STATE = MachineState::PROCESS_STOPPED;
            }

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

        // TODO: Talk to maze traversal and reset path variables

        // Stop motors
        md.setM1Speed(0);
        md.setM2Speed(0);

        // walk forward traversal::INTERSECTION_WALK_DISTANCE
        // TODO: Use Nick Code here


        // Update state
        MACHINE_STATE = MachineState::TURNING;

        break;
    default:
        break;
    }
}