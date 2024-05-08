#include "Mixer.h"

// Mixer::Mixer(SLog *log, const char*name, CoopSemaphore *taskSema, uint8_t mock) 
//   : PODRacer(log, name, taskSema, mock) {
    
// }

void Mixer::begin(bool allowLog) {
  tdw = new TaskData();
  log->info("begin() - task ready", true, name);
}

void Mixer::update(TaskData *data, bool allowLog) {
  data->data.start_millis = millis();
  this->tdr = data;
  // make a copy of input data. Later on, we do some updates for tdw based on task data
  memcpy(tdw, tdr, sizeof(tdr));

  for (uint8_t t=0; t < TASK_LIST_SIZE; t++) {
    TaskList item = taskList[t];
    if (item.taskID < 0 || item.task == nullptr) break;
    TaskData *td = item.task->getTaskData();
     switch (item.taskID) {
      case TASK_HOVER: { 
        sprintf(buffer, "(HOVER)(%0d at %10d) - get data (CH[HOVERING]: %d)", 
        item.taskID, item.task,
        td->data.ch[HOVERING]);
        log->debug(buffer, allowLog, name);
        // set write data 
        tdw->data.ch[HOVERING] = td->data.ch[HOVERING];
        break;
      }
      case TASK_STEERING: { 
        sprintf(buffer, "(STEER)(%0d at %10d) - get data (CH[xyz]: %d)", 
        item.taskID, item.task,
        td->data.ch[HOVERING]);
        log->debug(buffer, allowLog, name);
        break; 
      }
      case TASK_OPTICALFLOW: { 
        sprintf(buffer, "(OFLOW)(%0d at %10d) - get data (CH[R][P][Y]: [%d][%d][%d])", 
        item.taskID, item.task,
        td->data.ch[ROLL],
        td->data.ch[PITCH],
        td->data.ch[YAW]
        );
        log->debug(buffer, allowLog, name);        
        // set write data (note: add SDIST relative value to current value !)
        tdw->data.ch[ROLL] += td->data.ch[ROLL];        break;
        tdw->data.ch[PITCH] += td->data.ch[PITCH];        break;
        tdw->data.ch[YAW] += td->data.ch[YAW];        break;
      }
      case TASK_SURFACEDISTANCE: { 
        sprintf(buffer, "(SDIST)(%0d at %10d) - get data (CH[HOVERING]: %d)", 
        item.taskID, item.task,
        td->data.ch[HOVERING]);
        // log->debug(buffer, allowLog, name);
        // // set write data (note: add SDIST relative value to current value !)
        // tdw->data.ch[HOVERING] += td->data.ch[HOVERING];
        break;
      }

       default: {
           // do nothing
       }
     }
  }

  delay(10);
  data->data.end_millis = millis();
  log->data(tdw, allowLog, name, "UPD");
}

TaskData *Mixer::getMockedData(TaskData *td, uint8_t mode) {
  if (mode > 0) {

  }
  return this->bbd;
}