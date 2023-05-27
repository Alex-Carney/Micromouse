#include <SPI.h>
#include "ArduinoMotorShieldR3.h"
#include "ArduinoMotorEncoder.h"
#include "CircularBuffer.h"
#include "low_level_control.h"
#include "MotorCommand.h"
#include "NAxisMotion.h"
#include "PositionSensor.h"
#include <Arduino.h>
#include <Wire.h>
#include "macros.h"

/****************************************************/
/*
Main file for the micromouse project

Authors: Alex Carney, Majd Hamdan, Youssef Marzouk, Nick Hagler
*/
/****************************************************/

// Macros
#define WALL_FOLLOW 1

// Set Turning Control Gains
#define FAST_TURN 0

// Set Speed Mode
#define ZOOM 1

// Define a macro for Pi
#define M_PI 3.14159265358979323846

// Object definition prototypes
ArduinoMotorShieldR3 md;
NAxisMotion mySensor;
// BasicLinearAlgebra BLA;

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
double WHEEL_CIRCUMFERENCE = 22;               // cm
double NORTH, SOUTH, EAST, WEST, curr_Heading; // Degrees for the 4 directions of the Maze

// Position Sensors
float sensor1_coefficients[] = {2.2e4, -1.227};
PositionSensor sensor_mini_right((int)A10, sensor1_coefficients);
float sensor2_coefficients[] = {1.18e4, -1.138};
PositionSensor sensor_marks((int)A11, sensor2_coefficients);
float sensor3_coefficients[] = {1.3e4, -1.149};
PositionSensor sensor_big_circle((int)A9, sensor3_coefficients);

namespace traversal
{
    // Constants for stopping
    const int PATH_DISTANCE_DIFFERENTIAL_THRESHOLD = 10; // (cm) If the difference in distance is greater than this, it's an intersection
    const int STOP_DISTANCE_THRESHOLD = 24;              // (cm) Distance threshold for stopping at a wall (front only)
    const int INTERSECTION_WALK_DISTANCE = 15;           // (cm) Distance to walk once intersection is found
    const int DESIRED_WALL_DIST = 11;                    // (cm) Distance to keep from wall, when a front wall is present

    // Constants for turning
    const int VALID_PATH_THRESHOLD = 20; // (cm) If the distance to the left,right,forward wall is less than this, it's a valid path

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
    const int SAMPLING_PERIOD = 10000; // Sampling period in microseconds

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

    // int front_wall_mode = 0; // 0: no wall, 1: wall
    enum class StopMode
    {
        WALL_PRESENT,
        NO_WALL
    };
    StopMode stop_mode = StopMode::NO_WALL;
}

// Global variables for DRIVING
namespace drive_control
{
// Constants
#if SPEED_MODE == 1
    const int SAMPLING_PERIOD = 20000; // Sampling period in microseconds
    const double MAX_SPEED_RADS = 7;   // Maximum speed in radians per second
#else
    const int SAMPLING_PERIOD = 40000; // Sampling period in microseconds
    const double MAX_SPEED_RADS = 3;   // Maximum speed in radians per second
#endif
    const int MAX_SPEED_AFTER_WALL_COMPENSATION = MAX_SPEED_RADS + 1;
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
    const double DEL_X_GAIN = 0.05;

    // when to turn on wall following
    const long WALL_FOLLOW_TIME = 1500000; // 1.5 seconds
    long time_started_driving = 0;
    bool control_based_on_wall = false;
}

// Global variables for TURNING
namespace turning
{
    // Constants
    const int SAMPLING_PERIOD = 10000; // Sampling period in microseconds

    // Globals
    long encoder3Val_start_m2;
    long encoder3Val_start_m1;
    long enc_start_m1;
    long enc_start_m2;
    long motor1_pos;
    long motor2_pos;
    CircularBuffer<double> control_buffer_m1(2, 3); // 2 steps back. 3 values per step
    CircularBuffer<double> control_buffer_m2(2, 3); // 2 steps back. 3 values per step
    double SINGLE_WHEEL_POS_REFERENCE = 3.55;       // (rad) How much to turn one wheel for all turns
    enum class TurnDirection
    {
        LEFT,
        RIGHT,
        BACKWARD,
        FORWARD
    };
    TurnDirection turn_direction = TurnDirection::FORWARD;
    double position_reference_left;
    double position_reference_right;
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

    // Sensor Initialization
    I2C.begin();
    mySensor.initSensor(0x28);                      // The I2C Address can be changed here inside this function in the library
    mySensor.setOperationMode(OPERATION_MODE_NDOF); // Can be configured to other operation modes as desired
    mySensor.setUpdateMode(MANUAL);                 // The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions

// Peripheral Initialization
#ifdef DEBUG_MODE
#if DEBUG_MODE == 1
    Serial.begin(115200); // Initialize the Serial Port to view information on the Serial Monitor

    mySensor.updateAccelConfig();
    mySensor.updateCalibStatus();
    Serial.println(mySensor.readSystemCalibStatus());

    Serial.print("Range: ");
    Serial.println(mySensor.readAccelRange());
    Serial.print("Bandwidth: ");
    Serial.println(mySensor.readAccelBandwidth());
    Serial.print("Power Mode: ");
    Serial.println(mySensor.readAccelPowerMode());

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

        // Initialize Set Directions
        mySensor.updateEuler();
        NORTH = mySensor.readEulerHeading();
        SOUTH = NORTH + 180;
        EAST = NORTH + 90;
        WEST = NORTH + 270;
        curr_Heading = NORTH * M_PI / 180;
        break;

    case MachineState::DRIVING:
        if (isFirstStateIteration)
        {
            // Serial.println("Got into driving state");
            //  Initialize T and X
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
            drive_control::time_started_driving = micros();
            drive_control::control_based_on_wall = false;
        }

        newTime = micros();

        if (((newTime - lastSampleTime)) >= drive_control::SAMPLING_PERIOD)
        {
            drive_control::motor1_pos = encoder::getEncoderValue(ENCODER_M1);
            drive_control::motor2_pos = encoder::getEncoderValue(ENCODER_M2);
            // Serial.println("Time to sample");
            //  Collect data

            lastSampleTime = newTime;

            // Check if we should turn on wall following
            if (drive_control::control_based_on_wall == false)
            {
                if ((newTime - drive_control::time_started_driving) > drive_control::WALL_FOLLOW_TIME)
                {
                    drive_control::control_based_on_wall = true;
                }
            }

            // Measure distances to both walls, in CM
            float dist_left_cm = sensor_mini_right.readDistanceCM();
            float dist_right_cm = sensor_marks.readDistanceCM();
            float dist_diff = dist_left_cm - dist_right_cm;

            // Measure front distance for later
            float dist_front_cm = sensor_big_circle.readDistanceCM();
            bool wall = isWall(dist_front_cm);
            bool intersection = isIntersection(dist_left_cm, dist_right_cm);
            // Update pre distances for next iteration
            traversal::pre_distance_left = dist_left_cm;
            traversal::pre_distance_right = dist_right_cm;
            if ((wall || intersection) && drive_control::control_based_on_wall == true)
            {
                Serial.println("IM IN HERE!!!");
                Serial.println("I AM HERE BECAUSE OF A ");
                Serial.println(wall ? "WALL" : "INTERSECTION");
                md.setM1Speed(0);
                md.setM2Speed(0);
                // pos_control::front_wall_mode = (int)wall;
                pos_control::stop_mode = wall ? pos_control::StopMode::WALL_PRESENT : pos_control::StopMode::NO_WALL;
                MACHINE_STATE = MachineState::DECELERATING;
                isFirstStateIteration = true;
                break; // Cancel this state early! Don't compensate
            }

            // Calculate Del W_ref from Del X
            double delWRef = drive_control::DEL_X_GAIN * dist_diff;

// Calculate new reference speeds - only if wall following on
#ifdef WALL_FOLLOW
#if WALL_FOLLOW == 1
            if (drive_control::control_based_on_wall == true)
            {
                drive_control::motor1_reference_speed = min(drive_control::MAX_SPEED_RADS + delWRef, drive_control::MAX_SPEED_AFTER_WALL_COMPENSATION);
                drive_control::motor2_reference_speed = min(drive_control::MAX_SPEED_RADS - delWRef, drive_control::MAX_SPEED_AFTER_WALL_COMPENSATION);
            }
            else
            {
                drive_control::motor1_reference_speed = drive_control::MAX_SPEED_RADS;
                drive_control::motor2_reference_speed = drive_control::MAX_SPEED_RADS;
            }

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

// if state delay
#if STATE_DELAY == 1
            md.setM1Speed(0);
            md.setM2Speed(0);
            delay(200);
#endif
            double curr_pos_from_wall = sensor_big_circle.readDistanceCM();
            double pos_to_move_rad = ((curr_pos_from_wall - traversal::DESIRED_WALL_DIST) * 2 * M_PI) / WHEEL_CIRCUMFERENCE;

            if (pos_control::stop_mode == pos_control::StopMode::WALL_PRESENT)
            {
                // Compensate distance based on wall
                pos_control::position_reference = pos_to_move_rad;
            }
            else
            {
                // No wall to compensate for
                pos_control::position_reference = (traversal::INTERSECTION_WALK_DISTANCE * 2 * M_PI) / WHEEL_CIRCUMFERENCE;
            }
        }
        // not first run
        newTime = micros();
        if (((newTime - lastSampleTime)) >= pos_control::SAMPLING_PERIOD)
        {
            // Collect data
            pos_control::motor1_pos = encoder::getEncoderValue(ENCODER_M1) - pos_control::enc_start_m1;
            pos_control::motor2_pos = encoder::getEncoderValue(ENCODER_M2) - pos_control::enc_start_m2;
            lastSampleTime = newTime;

            MotorCommand command = ll_control::compensateMousePosition(
                pos_control::position_reference,
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

            if (command.next_state == true)
            {
                Serial.print("Zero ESS");

                MACHINE_STATE = MachineState::LOGIC;
                isFirstStateIteration = true;
            }
        }
        break;
    case MachineState::LOGIC:
// Initialization
#if STATE_DELAY == 1
        md.setM1Speed(0);
        md.setM2Speed(0);
        delay(200);
#endif
        // Check what directions are valid
        traversal::path_left = sensor_mini_right.readDistanceCM() > traversal::VALID_PATH_THRESHOLD ? 1 : 0;
        traversal::path_right = sensor_marks.readDistanceCM() > traversal::VALID_PATH_THRESHOLD ? 1 : 0;
        traversal::path_forward = sensor_big_circle.readDistanceCM() > traversal::VALID_PATH_THRESHOLD ? 1 : 0;

        // make a decision about which way to turn - MAJD
        if (traversal::path_left == 1)
        {
            turning::turn_direction = turning::TurnDirection::LEFT;
        }
        else if (traversal::path_right == 1)
        {
            turning::turn_direction = turning::TurnDirection::RIGHT;
        }
        else if (traversal::path_forward == 1)
        {
            turning::turn_direction = turning::TurnDirection::FORWARD;
        }
        else
        {
            turning::turn_direction = turning::TurnDirection::BACKWARD;
        }
        // set state to turning
        MACHINE_STATE = MachineState::TURNING;
        isFirstStateIteration = true;

        break;
    case MachineState::TURNING:
        // Initialization
        if (isFirstStateIteration)
        {
            oldTime = micros();
            lastSampleTime = oldTime;
            // This seems pointless but it's not mine to change
            turning::encoder3Val_start_m1 = encoder::getEncoderValue(ENCODER_M1); // Starting value for the encoder
            turning::encoder3Val_start_m2 = encoder::getEncoderValue(ENCODER_M2);
            turning::enc_start_m1 = turning::encoder3Val_start_m1;
            turning::enc_start_m2 = turning::encoder3Val_start_m2;
            turning::encoder3Val_start_m1 = 0;
            turning::encoder3Val_start_m2 = 0;

            isFirstStateIteration = false;

// TODO: Should we remove this?
#if STATE_DELAY == 1
            md.setM1Speed(0);
            md.setM2Speed(0);
            delay(200);
#endif
        }

        newTime = micros();
        if (((newTime - lastSampleTime)) >= turning::SAMPLING_PERIOD)
        {
            // Collect data
            turning::motor1_pos = encoder::getEncoderValue(ENCODER_M1) - turning::enc_start_m1;
            turning::motor2_pos = encoder::getEncoderValue(ENCODER_M2) - turning::enc_start_m2;
            lastSampleTime = newTime;

            // Calculate reference position
            switch (turning::turn_direction)
            {
            case turning::TurnDirection::LEFT:
                turning::position_reference_left = -turning::SINGLE_WHEEL_POS_REFERENCE;
                turning::position_reference_right = turning::SINGLE_WHEEL_POS_REFERENCE;
                break;
            case turning::TurnDirection::RIGHT:
                turning::position_reference_left = turning::SINGLE_WHEEL_POS_REFERENCE;
                turning::position_reference_right = -turning::SINGLE_WHEEL_POS_REFERENCE;
                break;
            case turning::TurnDirection::FORWARD:
                turning::position_reference_left = 0;
                turning::position_reference_right = 0;
                break;
            case turning::TurnDirection::BACKWARD:
                // TODO: HOW DO WE DO A 180?
                turning::position_reference_left = -turning::SINGLE_WHEEL_POS_REFERENCE;
                turning::position_reference_right = -turning::SINGLE_WHEEL_POS_REFERENCE;
                break;
            }

            MotorCommand command = ll_control::compensateMousePosition(
                turning::position_reference_left,
                turning::position_reference_right,
                turning::motor1_pos,
                turning::motor2_pos,
                newTime,
                oldTime,
                turning::encoder3Val_start_m1,
                turning::encoder3Val_start_m2,
                turning::control_buffer_m1,
                turning::control_buffer_m2);

            // Apply control to motors
            md.setM1Speed((int)command.motor1_pwm);
            md.setM2Speed((int)command.motor2_pwm);
            oldTime = newTime;

            if (command.next_state == true)
            {
                Serial.print("Zero ESS");
                md.setM1Speed(0);
                md.setM2Speed(0);
                MACHINE_STATE = MachineState::DRIVING;
                isFirstStateIteration = true;
            }
        }
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
    if (abs(dist_left_cm - traversal::pre_distance_left) > traversal::PATH_DISTANCE_DIFFERENTIAL_THRESHOLD)
    {
        // double check its not a glitch reading
        if (abs(sensor_marks.readDistanceCM() - traversal::pre_distance_left) > traversal::PATH_DISTANCE_DIFFERENTIAL_THRESHOLD)
        {
            traversal::pre_distance_left = dist_left_cm;
            return true;
        }
    }
    if (abs(dist_right_cm - traversal::pre_distance_right) > traversal::PATH_DISTANCE_DIFFERENTIAL_THRESHOLD)
    {
        // double check its not a glitch reading
        if (abs(sensor_mini_right.readDistanceCM() - traversal::pre_distance_right) > traversal::PATH_DISTANCE_DIFFERENTIAL_THRESHOLD)
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