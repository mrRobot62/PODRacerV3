#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#include <Preferences.h> 

#define pragma once

#define TEST_NO_TRANSMITTER_USED            // TEST - ATTENTION use this only to work withou transmiter



#define FW_VERSION_PATTERN "%02d.%02d.%02d"
#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 2
#define FW_VERSION_PATCH 0

#define BYTE_TO_BINARY_PATTERN  "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY_PATTERN2 "%c%c%c%c %c%c%c%c"
#define BYTE_TO_BINARY(d)  \
  ((d) & 0x80 ? '1' : '0'), \
  ((d) & 0x40 ? '1' : '0'), \
  ((d) & 0x20 ? '1' : '0'), \
  ((d) & 0x10 ? '1' : '0'), \
  ((d) & 0x08 ? '1' : '0'), \
  ((d) & 0x04 ? '1' : '0'), \
  ((d) & 0x02 ? '1' : '0'), \
  ((d) & 0x01 ? '1' : '0') 


#define LOGLEVEL 3  // 3=Info, 4=Debug

//------------------------------------------------------------------------------------------------------
// output data in visualizer-mode or human-readable-mode
#define LOG_OUTPUT_VISUALIZER_MODE 0     // 0 or 1, 1(default) => visualizer-mode, 0=human-readable-mode
//------------------------------------------------------------------------------------------------------



// if you use the PODRacer Visualizer / Analyzer - please use this define and comment ALL LOG_xxxxx output
//#define LOG_VISUALIZER

// WakeUp-Flag for tasks, is set to 0, this task will not run
#define TASK_WAKEUP_BLINK 1
#define TASK_WAKEUP_HOVER 1
#define TASK_WAKEUP_OFLOW 1
#define TASK_WAKEUP_SDIST 0
#define TASK_WAKEUP_STEER 1


//#define LOG_ALL
//#define LOG_BLINK
// this ALLOW_LOGGING directives can suspend additional logoutput as human readable output 
// if set to 0 - suspend logging (prod-environment), 1=Test/development //
#define ALLOW_LOGGING_RECEIVER 0
#define ALLOW_LOGGING_MIXER 1
#define ALLOW_LOGGING_BLINK 0
#define ALLOW_LOGGING_HOVER 1
#define ALLOW_LOGGING_OFLOW 0
#define ALLOW_LOGGING_SDIST 1
#define ALLOW_LOGGING_STEER 0

#define LOG_ONCE_IDX 10


//#define USE_SDIST_VL53L0      // only VL53L0 OR VL53L1 - not both
#define USE_SDIST_VL53L1      // only VL53L1 OR VL53L0 - not both

// to use log-once mechanism in classes below standard defines are valid for all classes
// please note the first 8Bits (0-7) are usable for all classes
// bit 8-15 are individual for classes and are define in the class header file 
#define LOG_ONCE_ERROR0_BIT 0     // for printing an special error condition
#define LOG_ONCE_WARN0_BIT 1      // normally used inside read()
#define LOG_ONCE_DATA1_BIT 2      // same as before
#define LOG_ONCE_DATA2_BIT 3       // normally used inside update()
#define LOG_ONCE_DATA3_BIT 4       // same as before
#define LOG_ONCE_MOCK1_BIT 5      // identify mock data 1
#define LOG_ONCE_MOCK2_BIT 6      // identify mock data 2
#define LOG_ONCE_MOCK3_BIT 7      // identify mock data 3

#define LOG_ONCE_MASK_INIT 0x00   // reset everything to 0


//-------------------------------------------------------------

//-------------------------------------------------------------
// Global variables
//-------------------------------------------------------------
//char buffer[300];               // buffer for log output-messages



//#define TEST_OPTICAL_FLOW


#define LED1 15
#define LED_BUILTIN 2
#define LOOP_TIME 10
#define PIN_PMW3901 5

//------------------------------------------------------------------------------------------------------------
// TASK_IDs are used to indicate a blink pattern
//------------------------------------------------------------------------------------------------------------
// Important note:
// the order number is how mixper.cpp will read output data from the 
// task, means: higher task (priority), will override (or adapt) less task data
// side note: if an error occured, task ID is the msb byte of the error word !
//              example: 0b00000000 00000000
//                         |      | |______|  => 255 possible error codes
//                         |      |
//                         |______|           => which task produce an error?
#define TASK_HOVER 1              // Hover will have the lowest prio
#define TASK_STEERING 2
#define TASK_OPTICALFLOW 3
#define TASK_SURFACEDISTANCE 4     
#define TASK_EMERGENCY 5           // Emergency will have the highes prio
#define TASK_BLINK -1              // is a task but do not have any prio
#define TASK_RECEIVER -1           // remember, receiver is not a real cooptask
#define TASK_MIXER -1              // remember, mixer is not a real cooptask

// this are blink-patterns
#define PATTERN_PREVENTARMING 9
#define PATTERN_DISARMED 8
#define PATTERN_EMERGENCY 7
#define PATTERN_ARMED 6
#define PATTERN_IDLE 0

// note : mocked values must start >= 1, because default value (no mocking) is 0 ans real SBUS-Data received
#define MOCK_RECEIVER_READ 1     // id for createing mock data - see receiver.cpp getMockedData(mode)  

//------------------------------------------------------------------------------------------------------------
// receivers toogle values around 1-5 +/- if nothing done by user activity
// this avoid false readings inside Receiver-class. As bigger the value is, as less sensitive your PODracer is around center gimbal position
//------------------------------------------------------------------------------------------------------------
#define RECEIVER_NOISE 5       // if normaly center pos should be 1500 (calibrated transmitter), mostley this value slip +/- 5 aournd this centerposition
                                // is used inside receiver to remove this slip

#define GIMBAL_CENTER_POSITION (int16_t)1500
#define GIMBAL_MIN (int16_t)1000
#define GIMBAL_MAX (int16_t)2000

#define CC_SBUS_MIN 0       // index for channel_calibration value
#define CC_SBUS_CPO 1       // SBUS CenterPosition
#define CC_SBUS_MAX 2       // SBUS MAX
#define CC_GIMB_MIN 3       // Gimbal MIN
#define CC_GIMB_CPO 4       // Gimbal CP
#define CC_GIMB_MAX 5       // Gimbal MAX

//#define ARMING_VALUE 1600     // everything above this value is armed

#define ARMING_ON true
#define ARMING_OFF false

#define ROLL  0     // internal channel mapping set A(Roll) on position 0
#define PITCH 1     // dito for E(Pitch)
#define THRUST 5    // dito for T(Throttle) -> forward THRUST ESC -> we used the throttle gimbal for flying forward
#define YAW 3       // dito for R(Yaw)
#define ARMING 4    // ch5 for arming/disarming
//#define AUX1      // no AUX1
#define AUX2 6      // AUX2
#define AUX3 7      // AUX3
#define HOVERING 2     // hovering -> all 4 motors Throttle - flight controller manage this

// range interval around centerpositoin for A/E/Y
// this is used by sensors like OpticalFlow. All Values between value-CENTER_RANGE and value+CENTER_RANGE are used as "CENTER"
// this avoid very sensible gimbal movements. For a PODRacer this is ok, for real quad copters a totally NO_GO ;-)
// value is used for isGimbalMin/Max
//#define CENTER_RANGE 50 

// normally used for struct data (see globalvars.h)
#define CH_R 0
#define CH_P 1
#define CH_Y 2


//------------------------------------------------------------------------------------------------------------
// Hovering
//------------------------------------------------------------------------------------------------------------
#define HOVER_ROLL GIMBAL_CENTER_POSITION   // default ch1
#define HOVER_PITCH GIMBAL_CENTER_POSITION  // default ch2
#define HOVER_YAW GIMBAL_CENTER_POSITION    // default ch4
#define HOVER_THRUST GIMBAL_MIN             // ch8 forward mapped used for ESC for EDF thrust nozzle

//------------------------------------------------------------------------------------------------------------
//
// OpticalFlow
//------------------------------------------------------------------------------------------------------------
//#define OFLOW_COUNTER_MAX 100         // currently not used :-/
#define OFLOW_SLIP_RANGE 500          // during measurement we add the result from sensor, if sum is larger than range, we assume PODRacer slips
#define OFLOW_PMW3901_ZERO 1          // due to sensor fluctuation a little bit, this value is used as "minus/plus range" to avoid jitter
#define OFLOW_PID_OUTPUT_LIMIT 250    // to avoid to big output from PIDController, we limit the output value

//------------------------------------------------------------------------------------------------------------
// SurfaceDistance
//------------------------------------------------------------------------------------------------------------

// use below two flags only , if you do not use one of them. 
// Mostly used during developing without a physical sensor available
#define SDIST_IGNORE_TOF_SENSOR     // 
#define SDIST_IGNORE_LIDAR_SENSOR   //
// ----------------------------------------------------------

#define SDIST_MIN_DISTANCE 400         // mm minimum height for hovering
#define SDIST_MAX_DISTANCE 500         // mm maximum height for hovering
#define SDIST_THRESHOLD_RANGE_MM 50    // mm if current range is in a time range SDIST_THRESHOLD_RANGE_MS more than the last value - USE it
#define SDIST_THRESHOLD_RANGE_MS 50    // ms observe values in this timerange
#define SDIST_CONT_SCANS_MS 50         // ms for continous scans
#define SDIST_MINIMAL_HEIGHT 100       // mm up from this height the SDIST-Task will check target height, this is a security topic
#define SDIST_COND_MIN_VALUE 0         // mm define min valid conditions for height. If environment condition is bad, sensor can deliver invalid values
#define SDIST_COND_MAX_VALUE 1000      // mm max valid condition value for height
#define SDIST_PID_OUTPUT_LIMIT 100     // to avoid to big output from PIDController, we limit the output value. Adjust this value if to fast/slow in steps of +/- 10 
#define SDIST_BIAS 0.75                // please adjust this value -> higher more throttle outcome, lower -> less throttle outcome (Multiplicator for PIDAdjustesValue. This value is the new "HOVERING-Value")

#define SDIST_LDATA_TOF_RAW 0              // index in ldata array
#define SDIST_LDATA_TOF_HOVER 1

#define SDIST_FDATA_TOF_PID 0
#define SDIST_FDATA_TOF_HOVER 1
#define SDIST_FDATA_TOF_SETPOINT 2
#define SDIST_FDATA_TOF_SKFVALUE 3

// TFMini Serial 1
#define SDIST_LDATA_LIDAR_RAW 4              // index in ldata array
#define SDIST_LDATA_LIDAR_HOVER 5

#define SDIST_FDATA_LIDAR_PID 4
#define SDIST_FDATA_LIDAR_HOVER 5
#define SDIST_FDATA_LIDAR_SETPOINT 6
#define SDIST_FDATA_LIDAR_SKFVALUE 7

#define RX1_PIN 0                       // GPIO 0 to TX on Lidar
#define TX1_PIN 2                       // GPIO 2 to RX on Lidar



//------------------------------------------------------------------------------------------------------------
// Steering
//------------------------------------------------------------------------------------------------------------
#define STEERING_ROLL_BIAS 0.1      // adjust this if during steering, this bias is not high enough (or is to high) - roll & pitch must not be the same
#define STEERING_PITCH_BIAS 0.1     // adjust this if during steering, this bias is not high enough (or is to high) - roll & pitch must not be the same
#define STEEIRNG_MAX_RP 150         // adjust this, if the maximum compensation is not optimal, as higher as more pitch/roll follow up the yaw steering

//------------------------------------------------------------------------------------------------------------
// Blackbox
//------------------------------------------------------------------------------------------------------------
#define BLACKBOX_CS_PIN 32




#endif