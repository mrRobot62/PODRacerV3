#include "TaskSteering.h"

TaskSteering::TaskSteering(SLog *log, const char*name, uint8_t taskID, CoopSemaphore *taskSema) 
  : Task(log, name, taskID, taskSema) {

}

bool TaskSteering::begin(bool allowLog) {
  // we assume, we do not have errors ;-)
  this->setInternalError(this->_id, 0);


  resetInternalError(this->_id);
  return true;
}

void TaskSteering::update(TaskData *data, uint8_t preventLogging) {

}