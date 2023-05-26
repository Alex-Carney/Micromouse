#include <SPI.h>
#include "ArduinoMotorShieldR3.h"
#include "ArduinoMotorEncoder.h"
#include "BasicLinearAlgebra.h"
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
double WHEEL_CIRCUMFERENCE = 22; // cm
double NORTH, SOUTH, EAST, WEST, curr_Heading; //Degrees for the 4 directions of the Maze

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
namespace turn_control
{
    // Constants
    const int SAMPLING_PERIOD = 20000;                                                         // Sampling Period in microseconds
    const BLA::Matrix<2, 2> A = {1, 0, 0, 1};                                                  // A_bar Matrix for discrete State-Space System (Identity)
    const BLA::Matrix<2, 2> B = {0.00035, 0.00035, 0.00423985463355542, -0.00423985463355542}; // B_bar Matrix for discrete State-Space System
    const BLA::Matrix<2, 2> C = {1, 0, 0, 1};                                                  // C Matrix (Identity) measuring all states

// choose which F matrix to use for turning.
//        -57.3453085546904, -15.2453582250787, 1.12355118451498, 0.9721973845281, -57.5637281185996, 17.8912645110515, 1.1418253266365, -1.19356797352185 

#if FAST_TURN == 1
#define F_BAR                                                                                                                                              \
    {                                                                                                                                                      \
        -64.7412773115279, -17.5858202697336, 1.42970596701333, 1.26112940780672, -64.9208296716241, 19.7608942866482, 1.44647260526361, -1.46423829030544 \
    }
#else

#define F_BAR                                                                                                                                            \
    {                                                                                                                                                    \ 
        -50.3454789302405,-13.9488442765285,0.869813457437172,0.796116413363104,-50.4359171889747,15.044401620436,0.876502603298697,-0.877147859293039                                                                                                                          \
    }
#endif

    const BLA::Matrix<2, 4> F_bar = F_BAR; // assign the controller gain matrix F

    // Globals
    long motor1_pos;
    long motor2_pos;
    long start_time;
    double motor1_reference_speed;
    double motor2_reference_speed;
    long encoder3Val_start_m2;
    long encoder3Val_start_m1;
    double turn_start;
    bool KILL_TURN = false;
    

    //Target Vector
    BLA::Matrix<2,1> y_star;

    // state vectors
    BLA::Matrix<2, 1> xk = {0, 0};  // x(k) vector
    BLA::Matrix<2, 1> xkp = {0, 0}; // x(k+1) vector
    
    // controller vectors
    BLA::Matrix<2, 1> zk = {0, 0};  // z(k) vector
    BLA::Matrix<2, 1> zkp = {0, 0}; // z(k+1) vector

    // Measurement vector
    BLA::Matrix<2, 1> yk = {0, 0};

    // controller
    BLA::Matrix<2, 1> u = {0, 0};

    // Full System Vector
    BLA::Matrix<4, 1> wk = xk && zk; // Vertically concatenate xk and zk [xk;zk]

    enum class TurnCommand
    {
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD
    };
    TurnCommand turn_direction = TurnCommand::FORWARD;

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

    //Sensor Initialization
    I2C.begin();
    mySensor.initSensor(0x28);          //The I2C Address can be changed here inside this function in the library
    mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
    mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  
    
// Peripheral Initialization
#ifdef DEBUG_MODE
#if DEBUG_MODE == 1
    Serial.begin(115200);               // Initialize the Serial Port to view information on the Serial Monitor
    

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

        //Initialize Set Directions
        mySensor.updateEuler();
        NORTH = mySensor.readEulerHeading();
        SOUTH = NORTH + 180;
        EAST = NORTH + 90;
        WEST = NORTH + 270;
        curr_Heading = NORTH*M_PI/180;
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


            //DEBUGGING TURNING WITH HEADING AND DRIVING WITH HEADING.
            // float old_val = 0;
            // float new_val =0;
            // bool BREACH = 0;
            // while(true){
            //     mySensor.updateEuler();
                
            //     new_val = mySensor.readEulerHeading()*M_PI/180;
            //     curr_Heading = ll_control::unwrap_Heading(old_val , new_val); //0-360
                
            //     BREACH = (curr_Heading + (M_PI/2) > 2*M_PI) ||(curr_Heading - (M_PI/2) < 0);

            //     // if(BREACH){

            //     // }else{

            //     // }
            //     // Serial.println(ll_control::unwrap_Heading_Turn(curr_Heading,curr_Heading-(M_PI/2))); //ex: want 300+90 = 30;
            //     Serial.print("curr_Heading: ");
            //     Serial.println(curr_Heading); 
            //     Serial.print("Euler Heading: ");
            //     Serial.println(mySensor.readEulerHeading()*M_PI/180);
            //     delay(400);
            //     old_val = curr_Heading;
                

            // }

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
            // Serial.println(dist_front_cm);
            //  // Print distances to left and right wall minus threshold
            //  Serial.print("Left: ");
            //  Serial.println(dist_left_cm - traversal::pre_distance_left);
            //  Serial.print("Right: ");
            //  Serial.println(dist_right_cm - traversal::pre_distance_right);

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

            // md.setM1Speed(0);
            // md.setM2Speed(0);
            // delay(200);

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
                md.setM1Speed(0);
                md.setM2Speed(0);
                MACHINE_STATE = MachineState::LOGIC;
                isFirstStateIteration = true;
            }
        }
        break;
    case MachineState::LOGIC:
        // Initialization
        delay(100);
        // Check what directions are valid
        traversal::path_left = sensor_mini_right.readDistanceCM() > traversal::VALID_PATH_THRESHOLD ? 1 : 0;
        traversal::path_right = sensor_marks.readDistanceCM() > traversal::VALID_PATH_THRESHOLD ? 1 : 0;
        traversal::path_forward = sensor_big_circle.readDistanceCM() > traversal::VALID_PATH_THRESHOLD ? 1 : 0;

        Serial.print("Forward");
        Serial.println(traversal::path_forward);
        Serial.print("left");
        Serial.println(traversal::path_left);
        Serial.print("right");
        Serial.println(traversal::path_right);

        // make a decision about which way to turn - MAJD
        // TODO: Youssef come here
        if (traversal::path_left == 1)
        {
            turn_control::turn_direction = turn_control::TurnCommand::LEFT;
        }
        else if (traversal::path_right == 1)
        {
            turn_control::turn_direction = turn_control::TurnCommand::RIGHT;
        }
        else if (traversal::path_forward == 1)
        {
            turn_control::turn_direction = turn_control::TurnCommand::FORWARD;
        }
        else
        {
            turn_control::turn_direction = turn_control::TurnCommand::BACKWARD;
        }
        // set state to turning
        MACHINE_STATE = MachineState::TURNING;
        isFirstStateIteration = true;

        break;
    case MachineState::TURNING:
        // Initialization
        if (isFirstStateIteration)
        {
            //Sensor Initialization
            // I2C.begin();
            // mySensor.resetSensor(0x06);
            // mySensor.initSensor(0x28);          //The I2C Address can be changed here inside this function in the library
            // mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
            // mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions

            //set kill order to false
            turn_control::KILL_TURN = false;
            

            oldTime = micros();
            turn_control::start_time = oldTime;
            lastSampleTime = oldTime;
            turn_control::start_time = oldTime;
            turn_control::encoder3Val_start_m1 = encoder::getEncoderValue(ENCODER_M1); // Starting value for the encoder
            turn_control::encoder3Val_start_m2 = encoder::getEncoderValue(ENCODER_M2);

            //Find Current Heading and set to start
            turn_control::turn_start = curr_Heading;

            switch(turn_control::turn_direction) {
                case turn_control::TurnCommand::FORWARD:
                    turn_control::y_star = {0,0};
                    //break state_machine;
                    Serial.println("FORWARD");
                    break;
                case turn_control::TurnCommand::RIGHT:
                    turn_control::y_star = {0,curr_Heading + M_PI/2};
                    Serial.println("RIGHT");
                    break;
                case turn_control::TurnCommand::LEFT:
                    turn_control::y_star = {0,curr_Heading-M_PI/2};
                    Serial.println("LEFT");
                    break;
                case turn_control::TurnCommand::BACKWARD:
                    //NEED TO DO NESTED LEFT TURNS!!!!
                    //ALEX-GO HERE
                    turn_control::y_star = {0,curr_Heading+M_PI};
                    Serial.println("BAKWARD");
                    break;

                

            }

            if(turn_control::turn_direction == turn_control::TurnCommand::LEFT) {
            }
                if(turn_control::turn_direction == turn_control::TurnCommand::RIGHT) {
            }
                if(turn_control::turn_direction == turn_control::TurnCommand::FORWARD) {
            }
                if(turn_control::turn_direction == turn_control::TurnCommand::BACKWARD) {
            }

            // Stop wheels just in case
            md.setM1Speed(0);
            md.setM2Speed(0);

            isFirstStateIteration = false;

            Serial.print("y_star: ");
            Serial.println(turn_control::y_star(0,1));

            Serial.print("Heading: ");
            Serial.println(curr_Heading);

            Serial.print("turn_start: ");
            Serial.println(turn_control::turn_start);
        }

        if((newTime - turn_control::start_time >= 3000000) || (turn_control::KILL_TURN)){
            isFirstStateIteration = true;
            bool double_turn = false;

            //Reset state-space vectors
            turn_control::xk = {0,curr_Heading};
            turn_control::yk = turn_control::xk;
            turn_control::zk = {0,0};

            //RESET DIRECTIONS
            NORTH = ll_control::unwrap_Heading(NORTH*M_PI/180, mySensor.readEulerHeading()*M_PI/180)*180/M_PI;
            SOUTH = NORTH + 180;
            EAST = NORTH + 90;
            WEST = NORTH + 270;
            curr_Heading = NORTH*M_PI/180;

            //ALEX TAKE A LOOK AT THIS (TRYING TO SAY)
            //If turn command was backward, set machine state back to turning and set turn command to left
            // switch(turn_control::turn_direction) {
            //     case turn_control::TurnCommand::BACKWARD:
            //         MACHINE_STATE = MachineState::TURNING;
            //         turn_control::turn_direction = turn_control::TurnCommand::LEFT;
            //         double_turn = true;
            //         break;
            // }
            // if(!double_turn){
                MACHINE_STATE = MachineState::DRIVING;
                break;
            //}
        }

        newTime = micros();
        if (((newTime - lastSampleTime)) >= turn_control::SAMPLING_PERIOD)
        {

            turn_control::motor1_pos = encoder::getEncoderValue(ENCODER_M1);
            turn_control::motor2_pos = encoder::getEncoderValue(ENCODER_M2);
            
            
            mySensor.updateEuler();
            mySensor.updateGyro();

            lastSampleTime = newTime;

            double heading_meas = (mySensor.readEulerHeading() * M_PI / 180);

            
            MotorCommand command = ll_control::compensateTurning(
                turn_control::motor1_pos,
                turn_control::motor2_pos,
                newTime,
                oldTime,
                turn_control::encoder3Val_start_m1,
                turn_control::encoder3Val_start_m2,
                turn_control::turn_start,
                heading_meas,
                turn_control::A,
                turn_control::B,
                turn_control::C,
                turn_control::F_bar,
                turn_control::y_star,
                turn_control::yk,
                turn_control::xk,
                turn_control::xkp,
                turn_control::zk,
                turn_control::zkp,
                turn_control::wk,
                turn_control::u
            );

            //Check if we reached the right orientation then abort
            if((turn_control::yk(0,1) >= turn_control::y_star(0,1) - 0.04) && (turn_control::yk(0,1) <= turn_control::y_star(0,1) + 0.04)){
                turn_control::KILL_TURN = true;
                md.setM1Speed(0);
                md.setM2Speed(0);

            }else{//Else apply normal control effort
                // Apply control to motors
                md.setM1Speed((int)command.motor1_pwm);
                md.setM2Speed((int)command.motor2_pwm);
            }
            
            // Serial.print("yk: {");
            // Serial.print(turn_control::yk(0,0));
            // Serial.print(" , ");
            // Serial.print(turn_control::yk(0,1));
            // Serial.println();

            

            //Update Data
            oldTime = newTime;
            turn_control::encoder3Val_start_m1 = turn_control::motor1_pos;
            turn_control::encoder3Val_start_m2 = turn_control::motor2_pos;

            
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