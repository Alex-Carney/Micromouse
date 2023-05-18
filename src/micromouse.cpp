#include <SPI.h>
#include "ArduinoMotorShieldR3.h"
#include "ArduinoMotorEncoder.h"
#include "CircularBuffer.h"
#include "low_level_control.h"
#include "PositionSensor.h"
#include "MotorCommand.h"
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

// Function prototypes
bool isIntersection(float dist_left_cm, float dist_right_cm);
bool isWall(float dist_front_cm);

// ------- Global variable definitions ----------- //
// Timing
long newTime = 0;
long oldTime = 0;
long lastSampleTime = 0;

// Position Sensors
float sensor1_coefficients[] = {2.2e4, -1.227};
PositionSensor sensor_mini_right((int)A10, sensor1_coefficients);
float sensor2_coefficients[] = {1.18e4, -1.138};
PositionSensor sensor_marks((int)A11, sensor2_coefficients);
float sensor3_coefficients[] = {1.3e4, -1.149};
PositionSensor sensor_big_circle((int)A9, sensor3_coefficients);

namespace traversal
{
    // Constants
    const int PATH_DISTANCE_THRESHOLD = 20; // (cm) Distance threshold for path detection
    const int STOP_DISTANCE_THRESHOLD = 20; // (cm) Distance threshold for stopping at a wall (front only)

    const int INTERSECTION_WALK_DISTANCE = 20; // (cm) Distance to walk once intersection is found
    const int DESIRED_WALL_DIST = 15; // (cm) Distance to keep from wall, when a front wall is present
 

    // Globals
    // For Path managing and maze traversal
    int path_right = -1;
    int path_left = -1;
    int path_forward = -1;
    float pre_distance_left = 0;
    float pre_distance_right = 0;

}

namespace pos_control
{
    // Constants
    const int SAMPLING_PERIOD = 40000; // Sampling period in microseconds

    // Globals
    long encoder3Val_start_m2;
    long encoder3Val_start_m1;
    long enc_start_m1;
    long enc_start_m2;
    long motor1_pos;
    long motor2_pos;
    CircularBuffer<double> control_buffer_m1(2, 3); // 2 steps back. 3 values per step
    CircularBuffer<double> control_buffer_m2(2, 3); // 2 steps back. 3 values per step
    double position_reference;

    int front_wall_mode = 0; // 0: no wall, 1: wall
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
            traversal::pre_distance_right = sensor_mini_right.readDistanceCM();
            traversal::pre_distance_left = sensor_marks.readDistanceCM();

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
            float dist_left_cm = sensor_mini_right.readDistanceCM();
            float dist_right_cm = sensor_marks.readDistanceCM();
            float dist_diff = dist_left_cm - dist_right_cm;

            // Measure front distance for later
            float dist_front_cm = sensor_big_circle.readDistanceCM();
            Serial.println(dist_front_cm);

            bool wall = isWall(dist_front_cm);
            bool intersection = isIntersection(dist_left_cm, dist_right_cm);
            if(wall || intersection) {
                Serial.println("IM IN HERE!!!");
                Serial.println("I AM HERE BECAUSE OF A ");
                Serial.println(wall ? "WALL" : "INTERSECTION");
                md.setM1Speed(0);
                md.setM2Speed(0);
                pos_control::front_wall_mode = (int)wall;
                MACHINE_STATE = MachineState::DECELERATING;
                isFirstStateIteration = true;
                break; // Cancel this state early! Don't compensate 
            }

            // Calculate Del W_ref from Del X
            double delWRef = drive_control::DEL_X_GAIN * dist_diff;

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
            MotorCommand command = ll_control::compensateDriveSpeed(
                drive_control::motor1_reference_speed,
                drive_control::motor2_reference_speed,
                drive_control::motor1_pos,
                drive_control::motor2_pos,
                newTime,
                oldTime,
                drive_control::encoder3Val_start_m1,
                drive_control::encoder3Val_start_m2,
                drive_control::control_buffer_m1,
                drive_control::control_buffer_m2);

            // Apply control to motors
            md.setM1Speed((int)command.motor1_pwm);
            md.setM2Speed((int)command.motor2_pwm);

            // Update data
            oldTime = newTime;
            drive_control::encoder3Val_start_m1 = drive_control::motor1_pos;
            drive_control::encoder3Val_start_m2 = drive_control::motor2_pos;
        }

        break;
    case MachineState::DECELERATING:
        // Initialization
        if (isFirstStateIteration)
        {
            oldTime = micros();
            lastSampleTime = oldTime;
            // This seems pointless but it's not mine to change
            pos_control::encoder3Val_start_m1 = encoder::getEncoderValue(ENCODER_M1); // Starting value for the encoder
            pos_control::encoder3Val_start_m2 = encoder::getEncoderValue(ENCODER_M2);
            pos_control::enc_start_m1 = pos_control::encoder3Val_start_m1;
            pos_control::enc_start_m2 = pos_control::encoder3Val_start_m2;
            pos_control::encoder3Val_start_m1 = 0;
            pos_control::encoder3Val_start_m2 = 0;

            isFirstStateIteration = false;

        }
        if (((newTime - lastSampleTime)) >= drive_control::SAMPLING_PERIOD)
        {
            // Collect data
            pos_control::motor1_pos = encoder::getEncoderValue(ENCODER_M1) - pos_control::enc_start_m1;
            pos_control::motor2_pos = encoder::getEncoderValue(ENCODER_M2) - pos_control::enc_start_m2;
            lastSampleTime = newTime;
            float curr_pos_from_wall = sensor_big_circle.readDistanceCM();

            if(pos_control::front_wall_mode == 1) {
                // Compensate distance based on wall
                pos_control::position_reference = curr_pos_from_wall - traversal::DESIRED_WALL_DIST;
            } else {
                // No wall to compensate for
                pos_control::position_reference = traversal::INTERSECTION_WALK_DISTANCE;
            }

            MotorCommand command = ll_control::compensateMousePosition(
                pos_control::position_reference,
                pos_control::motor1_pos,
                pos_control::motor2_pos,
                newTime,
                oldTime,
                pos_control::encoder3Val_start_m1,
                pos_control::encoder3Val_start_m2,
                pos_control::control_buffer_m1,
                pos_control::control_buffer_m2);

            // Apply control to motors
            md.setM1Speed((int)command.motor1_pwm);
            md.setM2Speed((int)command.motor2_pwm);

            oldTime = newTime;
        }
        break;
    case MachineState::LOGIC:
        // Initialization
        break;
    case MachineState::TURNING:
        // Initialization
        break;
    case MachineState::PROCESS_STOPPED:
        // Stop motors
        md.setM1Speed(0);
        md.setM2Speed(0);
        break;
    default:
        break;
    }
}

bool isIntersection(float dist_left_cm, float dist_right_cm)
{
    // Check if intersection
    if (dist_left_cm - traversal::pre_distance_left > traversal::PATH_DISTANCE_THRESHOLD)
    {
        // double check its not a glitch reading
        if (sensor_marks.readDistanceCM() - traversal::pre_distance_left > traversal::PATH_DISTANCE_THRESHOLD)
        {
            traversal::pre_distance_left = dist_left_cm;
            return true;
        }
    }
    if (dist_right_cm - traversal::pre_distance_right > traversal::PATH_DISTANCE_THRESHOLD)
    {
        // double check its not a glitch reading
        if (sensor_mini_right.readDistanceCM() - traversal::pre_distance_right > traversal::PATH_DISTANCE_THRESHOLD)
        {
            traversal::pre_distance_right = dist_right_cm;
            return true; 
        }
    }
    return false;
}

bool isWall(float dist_front_cm)
{
    // Check if wall
    if (dist_front_cm < traversal::STOP_DISTANCE_THRESHOLD)
    {
        return true;
    }
    return false;
}