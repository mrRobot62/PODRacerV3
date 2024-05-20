#include "TaskOpticalFlow.h"

TaskOpticalFlow::TaskOpticalFlow(SLog *log, const char*name, uint8_t taskID, uint8_t csPin, CoopSemaphore *taskSema) 
  : Task(log, name, taskID, taskSema) {
  this->csPin = csPin;
}

//-----------------------------------------------------------------------------------------------
// begin()
// called from main ony one time during initialization
//-----------------------------------------------------------------------------------------------
bool TaskOpticalFlow::begin(bool allowLog) {
  // we assume, we do not have errors ;-)
  this->setInternalError(this->_id, 0);

  sprintf(buffer, "begin() - task ready - allowLogging: %d", allowLog);
  log->info(buffer, true, name);
  flow = new Bitcraze_PMW3901(csPin);
  loadDefaultDataOFlow(&mspDataOFlow);

  if (flow == nullptr) {
    log->error("PMW3901 not availabe", name);
    setInternalError(_id, ERROR_TASK_SENSOR_INIT);
    return false;
  }
  slip2RollAxis = 0.0;
  slip2RollAxis = 0.0;

  rawX = 0;
  rawY = 0;
  //setPointSlipX = 0.0;
  //setPointSlipY = 0.0;

  flowCounter = 0;
  //
  // PID behaviour
  // rawInX represent the input value which got by sensor
  // The PID controller try to adjust to zero for X/Y (0 means no slip/movements)
  // depends who the sensor is assembled

  // if direction==NORTH or direction==EAST, than sensor is assemble in NORTH-SOUTH direction
  if ((direction[0] == 0) || (direction[0] == 2)) {
    pidX = new PID(
            &slipAdjX,                                        // store the adjusted error for X
            &slip2RollAxis,                                   // 
            &mspDataOFlow.setPointSlipRP[CH_R],               // set point which we try to reach
            float(mspDataOFlow.pidRGain[0]/100.0),           // kP value
            float(mspDataOFlow.pidRGain[1]/100.0),           // Ki value
            float(mspDataOFlow.pidRGain[2]/100.0),           // kD value
            DIRECT);
    pidY = new PID(
            &slipAdjY,                                        // store the adjusted error for Y
            &slip2PitchAxis,                                   // 
            &mspDataOFlow.setPointSlipRP[CH_P],               // set point which we try to reach
            float(mspDataOFlow.pidPGain[0]/100.0),           // kP value
            float(mspDataOFlow.pidPGain[1]/100.0),           // Ki value
            float(mspDataOFlow.pidPGain[2]/100.0),           // kD value
            DIRECT);

  }
  else if ((direction[0] == 1) || (direction[0] == 3)) {
    // direction==WEST or direction==SOUTH
    pidX = new PID(
            &slipAdjX,                                        // store the adjusted error for X
            &slip2PitchAxis,                                   // 
            &mspDataOFlow.setPointSlipRP[CH_R],               // set point which we try to reach
            float(mspDataOFlow.pidRGain[0]/100.0),           // kP value
            float(mspDataOFlow.pidRGain[1]/100.0),           // Ki value
            float(mspDataOFlow.pidRGain[2]/100.0),           // kD value
            DIRECT);
    pidY = new PID(
            &slipAdjY,                                        // store the adjusted error for Y
            &slip2RollAxis,                                   // 
            &mspDataOFlow.setPointSlipRP[CH_P],               // set point which we try to reach
            float(mspDataOFlow.pidPGain[0]/100.0),           // kP value
            float(mspDataOFlow.pidPGain[1]/100.0),           // Ki value
            float(mspDataOFlow.pidPGain[2]/100.0),           // kD value
            DIRECT);
  }

  pidX->SetMode(AUTOMATIC);
  pidY->SetMode(AUTOMATIC);
  pidX->SetOutputLimits(-OFLOW_PID_OUTPUT_LIMIT, OFLOW_PID_OUTPUT_LIMIT);
  pidY->SetOutputLimits(-OFLOW_PID_OUTPUT_LIMIT, OFLOW_PID_OUTPUT_LIMIT);
  pidX->SetSampleTime(LOOP_TIME);
  pidY->SetSampleTime(LOOP_TIME);

  if (flow->begin() == false) {
    log->error("oflow->begin() failed", _tname);
    setInternalError(_id, ERROR_TASK_SENSOR_BEGIN);
    return false;    
  }

  sprintf(buffer, "OFlow sensor ready");
  log->info(buffer, true, name);
  cnt=0;
  resetInternalError(this->_id);
  return true;
}

void TaskOpticalFlow::update(TaskData *data, bool allowLog) {

  /** if one gimbal is not centered, no optical flow measurement is needed **/
  /** Assumption: if a gimbal is not centered, pilot interact (like steering) and optical flow measurement should be ignored **/
  //rpy[0] = _recv->isGimbalCentered(ROLL, true);
  //rpy[1] = _recv->isGimbalCentered(PITCH, true);
  //rpy[2] = _recv->isGimbalCentered(YAW, true);

  // only roll/pitch can remove drifting (not yaw!)
  //if ((rpy[0] == false) || (rpy[1] == false)) {

}