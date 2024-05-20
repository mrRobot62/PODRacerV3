/**
    Use a PMW3901 optical flow sensor

    This sensor is used to check if PODRacer is on a movement over ground in +/-X and +/-Y direction

            -Y
            |
      +X-Y  |  -X-Y
    +X -----|-------
      +X+Y  |  -X+Y 
            |
            +Y
        <SOUTH>
    wiring-connector

    It's normal that the PODRace slip forward/backward/sideways(left/right) due on weather conditions (wind), bad calibrated hardware, flight controler setup bad
    a.s.o

    During hovering, the PODRacer should not slip in any direction. With the PMW3901 we recognise any directions and calculate new receiver channel commands.

    Example:
      Forward-Slip:
      the PODRace slip during hovering forward. That means rear Motors are faster than front motors (or Thrust Prop is to fast).
      To to into back direction the FlightControler should increase spead of front motors this will be done if pitch command is set to backward (GIMBAL move down)

      Backward-Slip:
      pitch command is set to forward (gimbal move up)

      Left Slip:
      correct with roll - ROLL Gimbal move right

      Reight Slip      
      correct with roll - ROLL Gimbal move left

      To avoid an overshoting a PID-Controller is used for adjust slip values

      This adjusted slip value (for X/Y) is a used to calculate new channel values (Values between 1000-2000).
      No - slippering - channel value for roll, pitch, yaw should be 1500



  **/
#ifndef _TOFLOW_H_
#define _TOFLOW_H_
#include "Task.h"
#include <Bitcraze_PMW3901.h>
#include <PID_v1.h>

// error codes
#define ERROR_TASK_PID 1
#define ERROR_TASK_SENSOR_INIT 10
#define ERROR_TASK_SENSOR_BEGIN 11


class TaskOpticalFlow : public Task {
  public:
    TaskOpticalFlow(SLog *log, const char*name, uint8_t taskID, uint8_t csPin, CoopSemaphore *taskSema);

    void init(void) {;};      // implementation form abstract class
    bool begin(bool allowLog = 1) ;      
    void update(bool allowLog = 1) {;};
    void update(bool armed, bool allowLog = 0){;};
    void update(TaskData *data, bool allowLog = 1);

  protected:
    TaskData *getMockedData(TaskData *td, uint8_t mode) {
      return td;
    }

  private:
    // return default data for OpticalFlow
    // in a later versioin, this values are stored inside esp32 file system anc
    // can be manipulated via CLI
    void loadDefaultDataOFlow(TDataOFlow *data) {
      data->biasRPY[0] = 100;   // note: real value diveded by 100
      data->biasRPY[1] = 100;   // note: real value diveded by 100
      data->biasRPY[2] = 100;   // note: real value diveded by 100

      data->pidRGain[0] = 50;    // note: real value diveded by 100
      data->pidRGain[1] = 5;     // note: real value diveded by 100
      data->pidRGain[2] = 0;     // note: real value diveded by 100

      data->pidPGain[0] = 50;    // note: real value diveded by 100
      data->pidPGain[1] = 5;     // note: real value diveded by 100
      data->pidPGain[2] = 0;     // note: real value diveded by 100

      data->pidYGain[0] = 50;    // note: real value diveded by 100
      data->pidYGain[1] = 5;     // note: real value diveded by 100
      data->pidYGain[2] = 0;     // note: real value diveded by 100

      data->biasRPY[0] = 100;   // note: real value diveded by 100
      data->biasRPY[1] = 100;   // note: real value diveded by 100
      data->biasRPY[1] = 100;   // note: real value diveded by 100

      data->setPointSlipRP[0] = 0;  // roll note: real value diveded by 100
      data->setPointSlipRP[1] = 0;  // pitch note: real value diveded by 100

    }

  private:
    Bitcraze_PMW3901 *flow;
    uint8_t csPin;
    TDataOFlow mspDataOFlow;

    // rawX/Y store the raw sensor values for X/Y
    // we add this values to the slipping X/Y values
    int16_t rawX, rawY;
    int16_t rawAdjX, rawAdjY;
    long rawXnormalized, rawYnormalized;

    // the slipping value for X and Y direction
    // roll axis - slip to left/right
    // pitch axis - slip to forward/backward
    double slip2RollAxis, slip2PitchAxis;

    // the adjusted slipping value for X/Y
    double slipAdjX, slipAdjY;

    // the PID controllyer should try to get this setpoint
    //double setPointSlipX, setPointSlipY;
    //double biasRoll = 1.0;
    //double biasPitch = 1.0;    

    //----- PID Controller for OpticalFlow sensor
    //double kpOpticalFlow = 0.5;
    //double kiOpticalFlow = 0.05;  //0.9
    //double kdOpticalFlow = 0;     //3.6

    PID *pidX, *pidY;
    uint8_t flowCounter;

    // north = wiring pins are south, north=flight direction
    //
    // [0]    direction                 : 0=North, 1=WEST, 2=EAST, 3=SOUTH
    // [1..4] rawXY multiplier +1/-1    : [1] Multiplier for X [2] Multiplier for Y
    // Example : NORTH direction        : [0][+1][-1]
    // Example : WEST direction         : [1][-1][+1]
    // Example : EAST direction         : [2][+1][-1]
    // Example : SOUTH direction        : [3][-1][+1]

    int8_t direction[3] = { 0, +1, -1 };
    uint8_t rpy[3];
    uint8_t cnt;    
};




#endif