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
#include "MazeTraversal.h"

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
bool isIntersectionRaw(float dist_left_cm, float dist_right_cm);
bool isWall(float dist_front_cm, bool is_wall_following_on);
double calculate_delWref(double dist_diff);

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
int NUM_SAMPLES = 10;

// TODO: Majd
int maze[8][10] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
    {1, 0, 0, 0, 0, 0, 1, 0, 0, 1},
    {1, 0, 1, 1, 1, 0, 1, 1, 0, 1},
    {1, 0, 1, 0, 0, 0, 0, 1, 0, 1},
    {1, 0, 1, 0, 1, 1, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
};
MazeTraversal mazeTraversal(8, 10, &maze[0][0]);
namespace traversal
{
    // Constants for stopping
    const int PATH_DISTANCE_DIFFERENTIAL_THRESHOLD = 10; // (cm) If the difference in distance is greater than this, it's an intersection
    const int STOP_DISTANCE_THRESHOLD_WALL_FOLLOW_OFF = 25; // (cm) Distance threshold for stopping at a wall (front only)
    const int STOP_DISTANCE_THRESHOLD_WALL_FOLLOW_ON = 22;  // (cm) Distance threshold for stopping at a wall (front only)
    const int INTERSECTION_WALK_DISTANCE = 12;           // [Default = 15] (cm) Distance to walk once intersection is found
    const int DESIRED_WALL_DIST = 5;                    // (cm) Distance to keep from wall, when a front wall is present

    // Constants for turning
    const int VALID_PATH_THRESHOLD = 25; // (cm) If the distance to the left,right,forward wall is greater than this, it's a valid path

    // Globals
    // For Path managing and maze traversal
    int path_right = -1;
    int path_left = -1;
    int path_forward = -1;
    float pre_distance_left = 0;
    float pre_distance_right = 0;
    int direction = 0; // which direction to turn. 1 = forward, 2 = right, 3 = left, 4 = back

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
    const double MAX_SPEED_AFTER_WALL_COMPENSATION = MAX_SPEED_RADS + 1;
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
    // Ramp input instead of step input
    const double MAX_RAMP_IDX = 40;
    double ramp_idx = MAX_RAMP_IDX;
}

namespace wall_control 
{
    long time_started_driving = 0;
    bool control_based_on_wall = false;
    // when to turn on wall following
    #if SPEED_MODE == 1
        const long WALL_FOLLOW_TIME = 1650000; // 1.0 seconds
    #else
        const long WALL_FOLLOW_TIME = 2200000; // 2.5 seconds
    #endif 

    double prev_error = 0;
    double integral = 0;

    // Gains:
    const double Kp = 0.05;
    const double Ki = 0.0;
    const double Kd = 0.0;
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
    double SINGLE_WHEEL_POS_REFERENCE = 3.5;       // (rad) How much to turn one wheel for all turns
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

    // Traversal
    mazeTraversal.initilizeTraversal();

// Peripheral Initialization
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
            // md.setM1Speed(0);
            // md.setM2Speed(0);

            // State transition logic: Record distances right, left forward.
            traversal::pre_distance_right = sensor_mini_right.readDistanceCM();
            traversal::pre_distance_left = sensor_marks.readDistanceCM();

            isFirstStateIteration = false;
            wall_control::time_started_driving = micros();
            wall_control::control_based_on_wall = false;

            drive_control::ramp_idx = drive_control::MAX_RAMP_IDX;
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
            if (wall_control::control_based_on_wall == false)
            {
                if ((newTime - wall_control::time_started_driving) > wall_control::WALL_FOLLOW_TIME)
                {
                    wall_control::control_based_on_wall = true;
                }
            }

            // Measure distances to both walls, in CM
            float dist_left_cm = sensor_mini_right.readDistanceCM(NUM_SAMPLES);
            float dist_right_cm = sensor_marks.readDistanceCM(NUM_SAMPLES);
            float dist_diff = dist_left_cm - dist_right_cm;

            // Measure front distance for later
            float dist_front_cm = sensor_big_circle.readDistanceCM(NUM_SAMPLES);
            bool wall = isWall(dist_front_cm, wall_control::control_based_on_wall);
            // Majd version
            //bool intersection = isIntersection(dist_left_cm, dist_right_cm);
            bool intersection = isIntersectionRaw(dist_left_cm, dist_right_cm);
            // Update pre distances for next iteration
            traversal::pre_distance_left = dist_left_cm;
            traversal::pre_distance_right = dist_right_cm; 
            if ( (wall || (intersection && wall_control::control_based_on_wall == true) ) )
            {
                Serial.print("I think there is a wall: ");
                Serial.println(dist_front_cm);
                #if DEBUG_MODE == 1
                Serial.println("IM IN HERE!!!");
                Serial.println("I AM HERE BECAUSE OF A ");
                Serial.println(wall ? "WALL" : "INTERSECTION");
                #endif
                md.setM1Speed(0);
                md.setM2Speed(0);
                // pos_control::front_wall_mode = (int)wall;
                pos_control::stop_mode = wall ? pos_control::StopMode::WALL_PRESENT : pos_control::StopMode::NO_WALL;
                MACHINE_STATE = MachineState::DECELERATING;
                isFirstStateIteration = true;
                break; // Cancel this state early! Don't compensate
            }

            // Calculate Del W_ref from Del X
            double delWRef = wall_control::Kp * dist_diff;
            //double delWRef = calculate_delWref(dist_diff);

            if(drive_control::ramp_idx > 1.0) {
                drive_control::ramp_idx = drive_control::ramp_idx - 1.0;
            }

            #if DEBUG_MODE == 1
            Serial.print("delwRef: ");
            Serial.println(delWRef);
            #endif


// Calculate new reference speeds - only if wall following on
#ifdef WALL_FOLLOW
#if WALL_FOLLOW == 1
            if (wall_control::control_based_on_wall == true)
            {
                drive_control::motor1_reference_speed = min(drive_control::MAX_SPEED_RADS + (delWRef/2), drive_control::MAX_SPEED_AFTER_WALL_COMPENSATION);
                drive_control::motor2_reference_speed = min(drive_control::MAX_SPEED_RADS - (delWRef/2), drive_control::MAX_SPEED_AFTER_WALL_COMPENSATION);
            }
            else
            {
                drive_control::motor1_reference_speed = (drive_control::MAX_SPEED_RADS / drive_control::ramp_idx);
                drive_control::motor2_reference_speed = (drive_control::MAX_SPEED_RADS / drive_control::ramp_idx);
            }

#else
            drive_control::motor1_reference_speed = (drive_control::MAX_SPEED_RADS / drive_control::ramp_idx);
            drive_control::motor2_reference_speed = (drive_control::MAX_SPEED_RADS / drive_control::ramp_idx);
#endif
#endif

            #if DEBUG_MODE == 1
            Serial.print("Motor 1 reference speed: ");
            Serial.println(drive_control::motor1_reference_speed);
            Serial.print("Motor 2 reference speed: ");
            Serial.println(drive_control::motor2_reference_speed);
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
            delay(STATE_DELAY_VALUE);
#endif
            double curr_pos_from_wall = sensor_big_circle.readDistanceCM(2 * NUM_SAMPLES);
            double pos_to_move_rad = ((curr_pos_from_wall - traversal::DESIRED_WALL_DIST) * 2 * M_PI) / WHEEL_CIRCUMFERENCE;
            if (pos_control::stop_mode == pos_control::StopMode::WALL_PRESENT)
            {
                // Compensate distance based on wall
                pos_control::position_reference = pos_to_move_rad;
                Serial.print("There is a wall this far away: ");
                Serial.println(curr_pos_from_wall);
                Serial.print("Because there is a wall, I am moving: ");
                Serial.println(pos_to_move_rad);
            }
            else
            {
                // No wall to compensate for
                pos_control::position_reference = (traversal::INTERSECTION_WALK_DISTANCE * 2 * M_PI) / WHEEL_CIRCUMFERENCE;
                Serial.print("Since there is no wall, I am moving: ");
                Serial.println(pos_control::position_reference);
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
                #if DEBUG_MODE == 1
                Serial.print("Zero ESS");
                #endif

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
        delay(STATE_DELAY_VALUE);
#endif
        // Check what directions are valid
        traversal::path_left = sensor_mini_right.readDistanceCM(NUM_SAMPLES) > traversal::VALID_PATH_THRESHOLD ? 1 : 0;
        traversal::path_right = sensor_marks.readDistanceCM(NUM_SAMPLES) > traversal::VALID_PATH_THRESHOLD ? 1 : 0;
        traversal::path_forward = sensor_big_circle.readDistanceCM(NUM_SAMPLES) > traversal::VALID_PATH_THRESHOLD ? 1 : 0;

        #if DEBUG_MODE == 1
        Serial.print("Forward");
        Serial.println(traversal::path_forward);
        Serial.print("left");
        Serial.println(traversal::path_left);
        Serial.print("right");
        Serial.println(traversal::path_right);
        #endif

        // traversal::direction = mazeTraversal.traverse(traversal::path_right, traversal::path_left, traversal::path_forward, 0); 
        // make a decision about which way to turn - MAJD
        // TODO: Youssef come here
        // if (traversal::direction == 3)
        // {
        //     turning::turn_direction = turning::TurnDirection::LEFT;
        // }
        // else if (traversal::direction == 2)
        // {
        //     turning::turn_direction = turning::TurnDirection::RIGHT;
        // }
        // else if (traversal::direction == 1)
        // {
        //     turning::turn_direction = turning::TurnDirection::FORWARD;
        // }
        // else
        // {
        //     turning::turn_direction = turning::TurnDirection::BACKWARD;
        // }
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
            delay(STATE_DELAY_VALUE);
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
                turning::position_reference_left = .9 * turning::SINGLE_WHEEL_POS_REFERENCE;
                turning::position_reference_right = -.9 * turning::SINGLE_WHEEL_POS_REFERENCE;
                break;
            case turning::TurnDirection::FORWARD:
                turning::position_reference_left = 0;
                turning::position_reference_right = 0;
                break;
            case turning::TurnDirection::BACKWARD:
                // TODO: HOW DO WE DO A 180?
                turning::position_reference_left = 2*turning::SINGLE_WHEEL_POS_REFERENCE;
                turning::position_reference_right = -2*turning::SINGLE_WHEEL_POS_REFERENCE;
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
                #if DEBUG_MODE == 1
                Serial.print("Zero ESS");
                #endif
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

bool isIntersectionRaw(float dist_left_cm, float dist_right_cm) 
{
    if ( (abs(dist_left_cm) > traversal::VALID_PATH_THRESHOLD) || (abs(dist_right_cm) > traversal::VALID_PATH_THRESHOLD) )
    {
        return true;
    }
    else
    {
        return false;
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

bool isWall(float dist_front_cm, bool is_wall_following_on)
{
    // Check if wall
    if (is_wall_following_on)
    {
        if (dist_front_cm < traversal::STOP_DISTANCE_THRESHOLD_WALL_FOLLOW_ON)
        {
            return true;
        }
    }
    else
    {
        if (dist_front_cm < traversal::STOP_DISTANCE_THRESHOLD_WALL_FOLLOW_OFF)
        {
            return true;
        }
    }
    return false;
}

double calculate_delWref(double dist_diff) 
{
    double dt = drive_control::SAMPLING_PERIOD / 1000000.0;
    wall_control::integral = wall_control::integral + dist_diff * dt;
    double derivative = (dist_diff - wall_control::prev_error) / dt;
    double delWref = wall_control::Kp * dist_diff + wall_control::Ki * wall_control::integral + wall_control::Kd * derivative;
    wall_control::prev_error = delWref;
    return delWref;
}