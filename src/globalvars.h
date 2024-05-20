#ifndef _GLOBAL_VARS_H_
#define _GLOBAL_VARS_H_

#include <CoopTaskBase.h>
#include <CoopTask.h>

#define TASK_LIST_SIZE 10


#define SIZE_BYTE_LIST 8
#define SIZE_LONG_LIST 8
#define SIZE_FLOAT_LIST 8
#define NUMBER_OF_CHANNELS 16


typedef struct {
  CoopTask<void> *list[TASK_LIST_SIZE];
} TaskList;
extern TaskList taskList;

struct dataRC {
  uint16_t channels[NUMBER_OF_CHANNELS];
  uint16_t gimbal_min;
  uint16_t gimbal_max;
  uint16_t gimbal_center;
} __attribute__ ((packed));

struct dataSurfaceTask {
  uint8_t pidGainTOF[3];      // represent a floating number multiplied by 100 for all TOF-PID-Controller [0]=PGain, [1]=IGain, [2]=DGain
  uint8_t pidGainLIDAR[3];      // represent a floating number multiplied by 100 for all LIDAR-PID-Controller [0]=PGain, [1]=IGain, [2]=DGain
  uint8_t pidTOF[3];       // represent a floating number multiplied by 100 for ToF-Sensor P/I/D
  uint8_t pidLIDAR[3];     // represent a floating number multiplied by 100 for LIDAR P/I/D
  uint8_t skfE;            // represent a floating number multiplied by 100 for SimpleKalmanFilter (the E-Value)
  uint16_t minHeight;       // minimal height over ground in mm
  uint16_t maxHeight;       // maximal height over ground in mm
  uint8_t thresholdRangeMM; // Threshold for height in mm (0-100)
  uint8_t continousScanRate; // Threshold for scans 50ms
  uint16_t sensorConditionMin;  //
  uint16_t sensorConditionMax;  //
  uint8_t tofErrorLimit;        // 100
  uint8_t lidarErrorLimit;      // 100
  uint8_t tofAdjustment;       // represent floating number multiplied by 100 (=> 0.75 = 75)
  uint8_t lidarAdjustment;     // represent floating number multiplied by 100 (=> 0.75 = 75)

} __attribute__ ((packed));

struct dataOpticalFlow {
  uint8_t  pidRGain[3];         // kp, ki, kd for PID-Controller (multiplyed by 100) (0-255 / 100 =  255=2.55, 1=0.01)
  uint8_t  pidPGain[3];         // kp, ki, kd for PID-Controller (multiplyed by 100)
  uint8_t  pidYGain[3];         // kp, ki, kd for PID-Controller (multiplyed by 100)
  uint8_t  biasRPY[3];           // bias for R/P/Y  (multiplyed by 100)
  double   setPointSlipRP[2];    // set the set point for slipping roll/pitch. normally should be 0.0
} __attribute__ ((packed));

struct dataGlobal {
  uint8_t header;
  long start_millis;
  long end_millis;
  char fwversion[10];
  uint8_t globalByteList[SIZE_BYTE_LIST];
  uint8_t globalLongList[SIZE_LONG_LIST];
  uint8_t globalFloatList[SIZE_FLOAT_LIST];
}__attribute__ ((packed));

struct dataGroup {
  char domain[5];
  char subdomain[5];
}__attribute__ ((packed));

struct dataAll {
  struct dataGlobal global;
  struct dataGroup group;
  struct dataRC rc;
  struct dataSurfaceTask sdist;
  struct dataOpticalFlow oflow;
  uint16_t crc;
}__attribute__ ((packed));


typedef dataAll TDataAll;
typedef dataGlobal TDataGlobal;
typedef dataRC TDataRC;
typedef dataSurfaceTask TDataSDist;
typedef dataOpticalFlow TDataOFlow;


// global variables used tot set via CLI
extern uint16_t ARMING_VALUE;
extern uint8_t CENTER_RANGE;
extern TDataAll mspDataAll;
extern TDataOFlow mspDataOFlow;

#endif