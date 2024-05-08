/***********************************************************************************************



***********************************************************************************************/

// PODRacer use CooperativeTask mechanism
#include <CoopTaskBase.h>
#include <CoopTask.h>
#include <CoopSemaphore.h>
#include <CoopMutex.h>
#include <BasicCoopTask.h>
#include <assert.h>
#include "global.h"
#include "global_utils.h"
#include "globalvars.h"

//#include "FS.h"
//#include <LittleFS.h>
//#include <ArduinoJson.h>  

// PODRacer Classes/Stuff
#include "Mixer.h"              // this is not a task; responsible to calculate new writings for receiver, based on all below tasks
#include "Receiver.h"           // this is not a task; responsible for reading/writing SBUS commands
#include "Logger.h"             // this is not a task; our logger class
#include "Blackbox.h"           // this is not a task; responsible to save data (blackbox) on SDCard
#include "WiFiData.h"           // this is not a task; responsible to save data (blackbox) on SDCard
#include "TaskBlink.h"          // task; responsible to generate blink-patterns
#include "TaskOpticalFlow.h"    // task; responsible to calculate new data based on the optical flow sensor
#include "TaskSurface.h"        // task; responsible to calculate new data based on both distance sensors
#include "TaskSteering.h"       // task; responsible to calculate a smooth steering
#include "TaskHover.h"          // task; responsible to calculate the hight of the PODRacer (hovering)



TaskList taskList ;
uint16_t ARMING_VALUE;
uint8_t CENTER_RANGE;

const char* _tname = "MAIN";         // used to log MAIN output

//HardwareSerial hsBus1(1);       // used by LIDAR Sensor in TaskSurface
HardwareSerial hsBus2(2);       // used by Receiver for SBUS

SLog logger(&Serial, 115200, LOGLEVEL);   // setup up our serial Logger


// TaskClasses
CoopTask<void>* taskBlink = nullptr;  
CoopTask<void>* taskHover = nullptr;  
CoopTask<void>* taskSurface = nullptr;  
CoopTask<void>* taskOFlow = nullptr;  
CoopTask<void>* taskSteer = nullptr;  
CoopTask<void>* taskIdle = nullptr;  

CoopSemaphore taskSema(1,1);            // start-value 1, one concurrend task taskSema(<number>,<concurrendTasks>)
Blackbox bb(&logger, BLACKBOX_CS_PIN);
Receiver receiver(&logger, "RECV", &taskSema, &hsBus2, 16, 17, true, "AEHRD23T");
Mixer mixer(&logger, "MIXER", &taskSema);


char buffer[300];
bool podracer_armed = false;
TaskData *tdr = nullptr;
TaskData *tdw = nullptr;

/***************************************************************/
/*                                                             */
/* all task do have this kind of function. This is a callback. */
/* from CoopTask. Every task need such a callback function     */
/*                                                             */
/* currently only this tasks are available                     */
/*  taskBlink     Blink patterns                               */
/*  taskHover     for hovering                                 */
/*  taskSteering  for steering ()                              */
/*  taskSurface   for measuring distance to survace            */
/*  taskOpticalFlow for measuring an optical flow              */
/***************************************************************/


/* RUN this callBack function for endless loop this task */
void callbackTaskBlinkPattern() {
  unsigned long lastMillis = millis();
  Serial.println("B1...");
  logger.info("callbackTaskBlinkPattern run...", true, _tname);
  BlinkPattern *obj = new BlinkPattern(&logger, "BLINK", TASK_BLINK);
  uint8_t blink_pattern = 0;
  obj->begin();
  for(;;) {
    blink_pattern=PATTERN_IDLE;                 // if nothing special blink to have a heart-beat signal
    if (receiver.isPreventArming()) {           // special blink pattern if arming is not possible
      Serial.println("Prevent");
      blink_pattern = PATTERN_PREVENTARMING;
    } else if (!receiver.isArmed()) {
        blink_pattern = PATTERN_DISARMED;       // blink pattern to show PODRacer is disarmed
    } else if (receiver.isArmed()) {
        blink_pattern = PATTERN_ARMED;          // blink pattern to show PODRacer is disarmed
    }
    obj->update(blink_pattern, ALLOW_LOGGING_BLINK);
    yield();
  }
}

/* RUN this callBack function for endless loop this task */
void callbackTaskHover() {
  unsigned long lastMillis = millis();
  TaskHover *obj = new TaskHover(&logger, "HOVER", TASK_HOVER, &taskSema);
  obj->begin(ALLOW_LOGGING_HOVER);
  mixer.addTask(obj, TASK_HOVER);
  for(;;) {
    // for hovering we have to set explizit the current hovering
    // value which was received by SBUS
    if (tdr != nullptr) {
      obj->setGlobalChannel(HOVERING, tdr->data.ch[HOVERING]);
    }
    obj->update(podracer_armed, ALLOW_LOGGING_HOVER);
    yield();
  }
}

/* RUN this callBack function for endless loop this task */
void callbackTaskSteering() {
  unsigned long lastMillis = millis();
  TaskSteering *obj = new TaskSteering(&logger, "STEER", TASK_STEERING, &taskSema);
  obj->begin(ALLOW_LOGGING_STEER);
  mixer.addTask(obj, TASK_STEERING);

  #if defined(USE_TASK_STEERING)
  for(;;) {


    
  }
  #endif
}

/* RUN this callBack function for endless loop this task */
void callbackTaskOpticalFlow() {
  unsigned long lastMillis = millis();
  TaskOpticalFlow *obj = new TaskOpticalFlow(&logger, "STEER", TASK_OPTICALFLOW, &taskSema);
  obj->begin(ALLOW_LOGGING_OFLOW);
  mixer.addTask(obj, TASK_OPTICALFLOW);
  for(;;) {

  }
}

/* RUN this callBack function for endless loop this task */
void callbackTaskSurface() {
  unsigned long lastMillis = millis();
  uint16_t err;
  HardwareSerial lidarSerial(1);
  TaskSurface *obj = new TaskSurface(&logger, "SDIST", TASK_SURFACEDISTANCE, &taskSema, &lidarSerial);
  obj->begin(ALLOW_LOGGING_SDIST);
  mixer.addTask(obj, TASK_SURFACEDISTANCE);
  for(;;) {
    if (obj->isInternalError()) {
      obj->update(podracer_armed, ALLOW_LOGGING_SDIST);
    }
    else {
      char bin[32];
      err = 0xCCF0;
      logger.convertValueToBinary(bin, sizeof(uint16_t), &err);
      sprintf(buffer, "*** TaskSurface error detected : [%s]", bin);
      logger.error(buffer, _tname);
      //logger.once_error(&log_once_);
    }
    yield();
  }
}

/*-----------------------------------------------------------------------------------------*/
//
//  Arduino SETUP 
//
/*-----------------------------------------------------------------------------------------*/

void setup() {
  while (!Serial);
  delay(100);
  #if !defined(ESP32)
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("not running on ESP32 device");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    assert(0);
  #endif


  sprintf(buffer, "- PODRacer-FWVersion: %s", bb.FWVersion());

  logger.info("------------------------------------------------", true, _tname);
  logger.info(buffer, true, _tname);
  logger.info("------------------------------------------------", true, _tname);
  logger.info("setup in progress....", true, _tname);
  //bb.begin();
  mixer.begin();

  taskBlink = new CoopTask<void>(F("BLINK"), callbackTaskBlinkPattern);
  taskHover = new CoopTask<void>(F("HOVER"), callbackTaskHover);
  taskSurface = new CoopTask<void>(F("SDIST"), callbackTaskSurface);
  taskOFlow = new CoopTask<void>(F("OFLOW"), callbackTaskOpticalFlow);
  taskSteer = new CoopTask<void>(F("STEER"), callbackTaskSteering);

  logger.info("CoopTasks initialized, wakeup tasks...", true, _tname);
  if (taskBlink && TASK_WAKEUP_BLINK) { if (taskBlink->wakeup()) {logger.info("...taskBlink woke up", true, _tname);} else {logger.error("...error wakup taskBlink", _tname);}}
  if (taskHover && TASK_WAKEUP_HOVER) { if (taskHover->wakeup()) {logger.info("...taskHover woke up", true, _tname);} else {logger.error("...error wakup taskHover", _tname);}}
  if (taskSurface && TASK_WAKEUP_SDIST) { if (taskSurface->wakeup()) {logger.info("...taskSurface woke up", true, _tname);} else {logger.error("...error wakup taskSurface", _tname);}}
  if (taskOFlow && TASK_WAKEUP_SDIST) { if (taskOFlow->wakeup()) {logger.info("...taskOFlow woke up", true, _tname);} else {logger.error("...error wakup taskOFlow", _tname);}}
  if (taskSteer && TASK_WAKEUP_SDIST) { if (taskSteer->wakeup()) {logger.info("...taskSteer woke up", true, _tname);} else {logger.error("...error wakup taskSteer", _tname);}}
  logger.info("all tasks running", true, _tname);

  //logger.setVisualizerMode(0);
  logger.setVisualizerMode(LOG_OUTPUT_VISUALIZER_MODE);
  logger.info("setup finished", true, _tname);
}



/*-----------------------------------------------------------------------------------------*/
//
//  Arduino PROCESSING-LOOP 
//  
// This loop calls 
//  * read recevier SBUS-Data
//  * update tasks (this updates calculate their own task data)
//  * call mixer, the mixer calculate the next sbus-write sbus-data, based on task taskdata 
//  * write SBUS-Data based on mixer calculation
//  *    
/*-----------------------------------------------------------------------------------------*/
void loop() {
  //  in every loop we read the SBUS-Receiver
  tdr = tdw = nullptr;
  // read SBUS Data from receiver
  receiver.setMock(MOCK_RECEIVER_READ);
  receiver.read(tdr, ALLOW_LOGGING_RECEIVER);
  tdr = receiver.getTaskData();
  // set global arming-flag
  podracer_armed = receiver.isArmed() ;
  podracer_armed = tdr->data.is_armed ;

  // // run all tasks
  runCoopTasks(nullptr, nullptr, nullptr);

  // now we mix all data together
  // update all task data due to latest receiver data structure
  mixer.update(tdr, ALLOW_LOGGING_MIXER);
  // get the result from mixer
  tdw = mixer.getTaskData();
  // //
  // // // update SBUS-Receiver with a mixed signal
  receiver.write(tdw);
  yield();
}