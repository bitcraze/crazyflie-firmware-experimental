#include "FreeRTOS.h"
#include "timers.h"
#include <float.h>
#include <math.h>
#include "sitaw.h"
#include "retrace.h"
#include "sound.h"
#include "commander.h"
#include "ledseq.h"

#include "debug.h"
#include "estimator_kalman.h"
#include "sequencer.h"
#include "pm.h"
#include "sensors.h"
#include "deck_digital.h"
#include "deck_constants.h"
#include "locodeck.h"

#define DEBUG_MODULE "RETRACE"

#define LED_RETRACE LED_GREEN_R


static void exitStateUninit() {}

static void enterStateWaitPosLock();
static void handleStateWaitPosLock();
static void exitStateWaitPosLock();

static void enterStatePosLocked();
static void handleStatePosLocked();
static void exitStatePosLocked();

static void enterStateTakeOff();
static void handleStateTakeOff();
static void exitStateTakeOff();

static void enterStateRecordTrace();
static void handleStateRecordTrace();
static void exitStateRecordTrace();

static void enterStateRetrace();
static void handleStateRetrace();
static void exitStateRetrace();

static void enterStatePlayPreRecorded();
static void handleStatePlayPreRecorded();
static void exitStatePlayPreRecorded();

static void enterStateLand();
static void handleStateLand();
static void exitStateLand();

static void enterStateStop();
static void handleStateStop();
static void exitStateStop();

//#define SEQUENCE_DATA_CIRCLE
//#define SEQUENCE_DATA_SPIRAL
//#define SEQUENCE_DATA_SQUARE
#define SEQUENCE_DATA_HAND
#include "sequences.h"


typedef struct {
  void (*enter)();
  void (*handle)();
  void (*exit)();
} state_handler_t;

typedef enum {
  ST_UNINIT = 0,
  ST_WAIT_POS_LOCK,
  ST_POS_LOCKED,
  ST_TAKE_OFF,
  ST_RECORD_TRACE,
  ST_RETRACE,
  ST_PLAY_PRE_RECORDED,
  ST_LAND,
  ST_STOP
} rt_state_t;

state_handler_t stateHandlers[] = {
  {enter: 0,                         handle: 0,                          exit: exitStateUninit},
  {enter: enterStateWaitPosLock,     handle: handleStateWaitPosLock,     exit: exitStateWaitPosLock},
  {enter: enterStatePosLocked,       handle: handleStatePosLocked,       exit: exitStatePosLocked},
  {enter: enterStateTakeOff,         handle: handleStateTakeOff,         exit: exitStateTakeOff},
  {enter: enterStateRecordTrace,     handle: handleStateRecordTrace,     exit: exitStateRecordTrace},
  {enter: enterStateRetrace,         handle: handleStateRetrace,         exit: exitStateRetrace},
  {enter: enterStatePlayPreRecorded, handle: handleStatePlayPreRecorded, exit: exitStatePlayPreRecorded},
  {enter: enterStateLand,            handle: handleStateLand,            exit: exitStateLand},
  {enter: enterStateStop,            handle: handleStateStop,            exit: exitStateStop},
};

static xTimerHandle timer;
static bool isInit = false;

static setpoint_t setpoint;


static void retraceTimer(xTimerHandle timer);
static void freeFallDetected();
static void tumbleDetected();
static void changeState(rt_state_t newState);
static rt_state_t state = ST_UNINIT;

static void moveSetPoint(point_t* point);

#define NR_OF_POSITIONS 2000
static point_t positions[NR_OF_POSITIONS];
static sequence_t sequence;
static sequence_t preRecorded;
static sequence_t takeOffSeq;
static sequence_t LandSeq;

#define LOCK_LENGTH 50
#define LOCK_THRESHOLD 0.001f
static uint32_t lockWriteIndex;
static float lockData[LOCK_LENGTH][3];
static void resetLockData();

static bool repeatSeq = true;
static bool startReplay = false;
static bool lowBat = false;
static bool tumbleStop = false;

typedef enum {
  MODE_MANUAL,
  MODE_RETRACE,
  MODE_PRE_RECORDED
} playMode_t;

playMode_t playMode = MODE_MANUAL;



static bool isRecButtonPressed() {
  return digitalRead(DECK_GPIO_IO2) == 0;
}

static bool isPlayButtonPressed() {
  return digitalRead(DECK_GPIO_IO3) == 0;
}

static void scanButtons() {
  if (MODE_MANUAL == playMode) {
    if (isRecButtonPressed()) {
      playMode = MODE_RETRACE;
    } else if (isPlayButtonPressed()) {
      playMode = MODE_PRE_RECORDED;
    }
  }
}


void retraceInit(void)
{
  if (isInit) {
    return;
  }


  sequenceInit(&sequence, NR_OF_POSITIONS, positions);
  sequenceInitStatic(&preRecorded, sizeof(seqDataMain) / sizeof(point_t), seqDataMain);
  sequenceInitStatic(&takeOffSeq, sizeof(seqDataTakeOff) / sizeof(point_t), seqDataTakeOff);
  sequenceInitStatic(&LandSeq, sizeof(seqDataLand) / sizeof(point_t), seqDataLand);

  setpoint.setEmergency = false;
  setpoint.resetEmergency = true;
  setpoint.xmode = 0b0111;
  setpoint.ymode = 0b0111;
  setpoint.zmode = 0b0111;

  setpoint.mode.x = modeAbs;
  setpoint.mode.y = modeAbs;
  setpoint.mode.z = modeAbs;

  sitAwRegisterFFCallback(freeFallDetected, 0);
  sitAwRegisterTumbleCallback(tumbleDetected, 0);

  resetLockData();

  changeState(ST_WAIT_POS_LOCK);

  timer = xTimerCreate("RetraceTimer", M2T(100), pdTRUE, NULL, retraceTimer);
  xTimerStart(timer, 100);

  pinMode(DECK_GPIO_IO2, INPUT_PULLUP);
  pinMode(DECK_GPIO_IO3, INPUT_PULLUP);

  isInit = true;
}

bool retraceTest(void)
{
  return isInit;
}

static void retraceTimer(xTimerHandle timer) {
  if (tumbleStop) {
    DEBUG_PRINT("Tumble stop!\n");
    changeState(ST_STOP);
  }

  if (isBatLow()) {
    lowBat = true;
  }

  scanButtons();

  stateHandlers[state].handle();
}

static void changeState(rt_state_t newState) {
  if (state != newState) {
    DEBUG_PRINT("Go to state %d\n", newState);
    stateHandlers[state].exit();
    state = newState;
    stateHandlers[state].enter();
  }
}

static void freeFallDetected() {
  startReplay = true;

  // Set a setpoint as early as possible to stop the fall. Use the current position.
  point_t point;
  point.x = getX();
  point.y = getY();
  point.z = getZ();

  moveSetPoint(&point);
}

static void tumbleDetected() {
  tumbleStop = true;
}



static bool hasLock() {
  bool result = false;

  // Store current state
  lockData[lockWriteIndex][0] = getVarPX();
  lockData[lockWriteIndex][1] = getVarPY();
  lockData[lockWriteIndex][2] = getVarPZ();

  lockWriteIndex++;
  if (lockWriteIndex >= LOCK_LENGTH) {
    lockWriteIndex = 0;
  }

  // Check if we have a lock
  int count = 0;

  float lXMax = FLT_MIN;
  float lYMax = FLT_MIN;
  float lZMax = FLT_MIN;

  float lXMin = FLT_MAX;
  float lYMin = FLT_MAX;
  float lZMin = FLT_MAX;

  for (int i = 0; i < LOCK_LENGTH; i++) {
    if (lockData[i][0] != FLT_MAX) {
      count++;

      lXMax = fmaxf(lXMax, lockData[i][0]);
      lYMax = fmaxf(lYMax, lockData[i][1]);
      lZMax = fmaxf(lZMax, lockData[i][2]);

      lXMin = fminf(lXMax, lockData[i][0]);
      lYMin = fminf(lYMin, lockData[i][1]);
      lZMin = fminf(lZMin, lockData[i][2]);
    }
  }

  uint16_t state = locodeckGetAnchorState();
  int anchorCount = 0;
  for (int i = 0; i < 8; i++) {
    if ((1 << i) & state) {
      anchorCount++;
    }
  }

  result = (count >= LOCK_LENGTH) && ((lXMax - lXMin) < LOCK_THRESHOLD) && ((lYMax - lYMin) < LOCK_THRESHOLD) && ((lZMax - lZMin) < LOCK_THRESHOLD && sensorsAreCalibrated() && anchorCount >= 4);
  return result;
}

static void resetLockData() {
    lockWriteIndex = 0;
    for (uint32_t i = 0; i < LOCK_LENGTH; i++) {
      lockData[i][0] = FLT_MAX;
      lockData[i][1] = FLT_MAX;
      lockData[i][2] = FLT_MAX;
    }
}

static void recordPosition() {
  point_t point;

  point.x = getX();
  point.y = getY();
  point.z = getZ();

  // DEBUG_PRINT("Rec (%d, %d, %d)\n", (int)(point.x * 100.0f), (int)(point.y * 100.0f), (int)(point.z * 100.0f));
  // DEBUG_PRINT("{x: %f, y: %f, z: %f},\n", (double)point.x, (double)point.y, (double)point.z);

  sequenceRecord(&sequence, &point);
}

static void moveSetPoint(point_t* point) {
  static point_t lastpoint;
  static point_t lastVelocity;

  setpoint.x[0] = point->x;
  setpoint.x[1] = (point->x - lastpoint.x)*0.1f;
  setpoint.x[2] = (setpoint.x[1] - lastVelocity.x)*0.1f;
  setpoint.y[0] = point->y;
  setpoint.y[1] = (point->y - lastpoint.y)*0.1f;
  setpoint.y[2] = (setpoint.y[1] - lastVelocity.y)*0.1f;
  setpoint.z[0] = point->z;
  setpoint.z[1] = (point->z - lastpoint.z)*0.1f;
  setpoint.z[3] = (setpoint.z[1] - lastVelocity.z)*0.1f;
  setpoint.yaw[0] = 0;
  setpoint.yaw[1] = 0;
  // DEBUG_PRINT("Set (%d, %d, %d)\n", (int)(point->x * 100.0f), (int)(point->y * 100.0f), (int)(point->z * 100.0f));

  setpoint.position = *point;

  commanderSetSetpoint(&setpoint, 3);

  lastpoint = *point;
  lastVelocity.x = setpoint.x[1];
  lastVelocity.y = setpoint.y[1];
  lastVelocity.z = setpoint.z[1];
}


// WAIT POS LOCK ----------------------------


static void enterStateWaitPosLock() {
}

static void handleStateWaitPosLock() {
  if (hasLock()) {
    DEBUG_PRINT("Position lock OK\n");
    changeState(ST_POS_LOCKED);
  }
}

static void exitStateWaitPosLock() {
}


// POSITION LOCKED ----------------------------


static void enterStatePosLocked() {
    ledseqRun(LED_RETRACE, seq_lps_lock);
}

static void handleStatePosLocked() {
  switch (playMode) {
    case MODE_MANUAL:
      break;

    case MODE_RETRACE:
      changeState(ST_RECORD_TRACE);
      break;

    case MODE_PRE_RECORDED:
      changeState(ST_TAKE_OFF);
      break;

    default:
      DEBUG_PRINT("Unexpected mode\n");
      break;
  }
}

static void exitStatePosLocked() {
    ledseqStop(LED_RETRACE, seq_lps_lock);
}


// TAKE OFF ----------------------------


static void enterStateTakeOff() {
    DEBUG_PRINT("Taking off\n");
    ledseqRun(LED_RETRACE, seq_lps_retrace);
    sequenceReset(&takeOffSeq);
}

static void handleStateTakeOff() {
  if (sequenceHasNext(&takeOffSeq)) {
    point_t* point = sequenceReplay(&takeOffSeq);
    moveSetPoint(point);
  } else {
    DEBUG_PRINT("Started!\n");
    changeState(ST_PLAY_PRE_RECORDED);
  }
}

static void exitStateTakeOff() {
    ledseqStop(LED_RETRACE, seq_lps_retrace);
}


// RECORD TRACE ----------------------------


static void enterStateRecordTrace() {
    ledseqRun(LED_RETRACE, seq_lps_lock);
}

static void handleStateRecordTrace() {
  if (startReplay) {
    DEBUG_PRINT("End of recording\n");
    changeState(ST_RETRACE);
  } else {
    recordPosition();
  }
}

static void exitStateRecordTrace() {
    ledseqStop(LED_RETRACE, seq_lps_lock);
}


// RETRACE ----------------------------


static void enterStateRetrace() {
    ledseqRun(LED_RETRACE, seq_lps_retrace);
    if (sequenceIsClosedLoop(&sequence)) {
        sequenceReset(&sequence);
        repeatSeq = true;
    } else {
        sequenceResetReverse(&sequence);
        repeatSeq = false;
    }
}

static void handleStateRetrace() {
  if (lowBat) {
    DEBUG_PRINT("Out of battery\n");
    changeState(ST_LAND);
  } else {
    if (sequenceHasNext(&sequence)) {
      point_t* point = sequenceReplay(&sequence);
      moveSetPoint(point);
    } else {
      if (repeatSeq) {
        sequenceReset(&sequence);
        point_t* point = sequenceReplay(&sequence);
        moveSetPoint(point);
      } else {
        DEBUG_PRINT("End of retrace\n");
        changeState(ST_STOP);
      }
    }
  }
}

static void exitStateRetrace() {
    ledseqStop(LED_RETRACE, seq_lps_retrace);
}

// PLAY PRE RECORDED ----------------------------


static void enterStatePlayPreRecorded() {
    ledseqRun(LED_RETRACE, seq_lps_retrace);
    sequenceReset(&preRecorded);
}

static void handleStatePlayPreRecorded() {
  if (lowBat) {
    DEBUG_PRINT("Out of battery\n");
    changeState(ST_LAND);
  } else {
    if (sequenceHasNext(&preRecorded)) {
      point_t* point = sequenceReplay(&preRecorded);
      moveSetPoint(point);
    } else {
      if (repeatSeq) {
        sequenceReset(&preRecorded);
        point_t* point = sequenceReplay(&preRecorded);
        moveSetPoint(point);
      } else {
        DEBUG_PRINT("End of play pre recorded\n");
        changeState(ST_LAND);
      }
    }
  }
}

static void exitStatePlayPreRecorded() {
    ledseqStop(LED_RETRACE, seq_lps_retrace);
}


// LAND ----------------------------


static void enterStateLand() {
    sequenceReset(&LandSeq);
}

static void handleStateLand() {
  if (sequenceHasNext(&LandSeq)) {
    point_t* point = sequenceReplay(&LandSeq);
    moveSetPoint(point);
  } else {
    DEBUG_PRINT("Landed!\n");
    changeState(ST_STOP);
  }
}

static void exitStateLand() {
}


// STOP ----------------------------

static void enterStateStop() {
  if (tumbleStop) {
    ledseqStop(SYS_LED, seq_calibrated);
    ledseqStop(SYS_LED, seq_alive);
    ledseqRun(SYS_LED, seq_lps_tumble);
  }
}

static void handleStateStop() {
  setpoint.setEmergency = true;
  setpoint.resetEmergency = false;

  setpoint.mode.x = modeDisable;
  setpoint.mode.y = modeDisable;
  setpoint.mode.z = modeDisable;
  setpoint.thrust = 0;

  commanderSetSetpoint(&setpoint, 3);
}

static void exitStateStop() {
}
