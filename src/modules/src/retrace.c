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

#define XB 1.5
#define YB 1.5
#define ZB 0.0

static point_t takeOffSeqData[] = {
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.05},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.1},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.15},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.2},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.25},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.3},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.35},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.4},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.45},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.55},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.6},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.65},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.7},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.75},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.8},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.85},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.9},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.95},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.1},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.2},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.3},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.4},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
};

static point_t landSeqData[] = {
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.4},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.3},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.2},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.1},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.9},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.8},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.7},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.6},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.5},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.45},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.4},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.35},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.3},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.25},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.2},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.15},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.1},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.05},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.0},
};

static point_t circleSeqData[] = {
    {x: XB + 0.4, y: YB + 0.0, z: ZB + 1.5},
    {x: XB + 0.397810977288, y: YB + 0.0417902661966, z: ZB + 1.5},
    {x: XB + 0.391267868256, y: YB + 0.0831231331841, z: ZB + 1.5},
    {x: XB + 0.380442287974, y: YB + 0.12354620804, z: ZB + 1.5},
    {x: XB + 0.365452723648, y: YB + 0.162617055618, z: ZB + 1.5},
    {x: XB + 0.346463237762, y: YB + 0.199908041057, z: ZB + 1.5},
    {x: XB + 0.323681672395, y: YB + 0.235011010286, z: ZB + 1.5},
    {x: XB + 0.297357374367, y: YB + 0.267541757319, z: ZB + 1.5},
    {x: XB + 0.26777846611, y: YB + 0.297144229437, z: ZB + 1.5},
    {x: XB + 0.235268692133, y: YB + 0.323494424221, z: ZB + 1.5},
    {x: XB + 0.200183875603, y: YB + 0.346303935797, z: ZB + 1.5},
    {x: XB + 0.162908023823, y: YB + 0.36532311147, z: ZB + 1.5},
    {x: XB + 0.123849125223, y: YB + 0.380343784203, z: ZB + 1.5},
    {x: XB + 0.0834346838829, y: YB + 0.391201551026, z: ZB + 1.5},
    {x: XB + 0.0421070404533, y: YB + 0.39777757245, z: ZB + 1.5},
    {x: XB + 0.000318530684293, y: YB + 0.399999873173, z: ZB + 1.5},
    {x: XB + -0.0414734654392, y: YB + 0.397844129861, z: ZB + 1.5},
    {x: XB + -0.0828115297738, y: YB + 0.391333937369, z: ZB + 1.5},
    {x: XB + -0.123243212511, y: YB + 0.380540550494, z: ZB + 1.5},
    {x: XB + -0.162325984292, y: YB + 0.36558210408, z: ZB + 1.5},
    {x: XB + -0.199632079742, y: YB + 0.346622320023, z: ZB + 1.5},
    {x: XB + -0.23475317941, y: YB + 0.323868715311, z: ZB + 1.5},
    {x: XB + -0.26730487887, y: YB + 0.297570330732, z: ZB + 1.5},
    {x: XB + -0.296930896077, y: YB + 0.268015005093, z: ZB + 1.5},
    {x: XB + -0.323306970908, y: YB + 0.235526224787, z: ZB + 1.5},
    {x: XB + -0.346144414228, y: YB + 0.200459583205, z: ZB + 1.5},
    {x: XB + -0.365193267627, y: YB + 0.163198888722, z: ZB + 1.5},
    {x: XB + -0.380245039241, y: YB + 0.124151963869, z: ZB + 1.5},
    {x: XB + -0.391134985721, y: YB + 0.0837461816728, z: ZB + 1.5},
    {x: XB + -0.397743915366, y: YB + 0.0424237880083, z: ZB + 1.5},
    {x: XB + -0.399999492691, y: YB + 0.000637061166595, z: ZB + 1.5},
    {x: XB + -0.397877030145, y: YB + -0.041156638382, z: ZB + 1.5},
    {x: XB + -0.391399758322, y: YB + -0.0824998738498, z: ZB + 1.5},
    {x: XB + -0.380638571698, y: YB + -0.12294013883, z: ZB + 1.5},
    {x: XB + -0.365711252683, y: YB + -0.16203481003, z: ZB + 1.5},
    {x: XB + -0.346781182477, y: YB + -0.199355991833, z: ZB + 1.5},
    {x: XB + -0.32405555285, y: YB + -0.234495199668, z: ZB + 1.5},
    {x: XB + -0.297783098397, y: YB + -0.267067830914, z: ZB + 1.5},
    {x: XB + -0.268251374117, y: YB + -0.296717374422, z: ZB + 1.5},
    {x: XB + -0.235783608086, y: YB + -0.323119312573, z: ZB + 1.5},
    {x: XB + -0.200735163689, y: YB + -0.345984673156, z: ZB + 1.5},
    {x: XB + -0.16348965013, y: YB + -0.365063192201, z: ZB + 1.5},
    {x: XB + -0.124454723786, y: YB + -0.380146053153, z: ZB + 1.5},
    {x: XB + -0.0840576263562, y: YB + -0.391068172383, z: ZB + 1.5},
    {x: XB + -0.0427405086609, y: YB + -0.397710006059, z: ZB + 1.5},
    {x: XB + -0.000955591244913, y: YB + -0.399998858555, z: ZB + 1.5},
    {x: XB + 0.0408397852258, y: YB + -0.397909678121, z: ZB + 1.5},
    {x: XB + 0.0821881656096, y: YB + -0.391465331075, z: ZB + 1.5},
    {x: XB + 0.122636987188, y: YB + -0.380736351526, z: ZB + 1.5},
    {x: XB + 0.161743533015, y: YB + -0.365840169374, z: ZB + 1.5},
    {x: XB + 0.199079777505, y: YB + -0.346939825025, z: ZB + 1.5},
    {x: XB + 0.234237071225, y: YB + -0.324242184893, z: ZB + 1.5},
    {x: XB + 0.2668306136, y: YB + -0.297995677227, z: ZB + 1.5},
    {x: XB + 0.296503664608, y: YB + -0.268487573034, z: ZB + 1.5},
    {x: XB + 0.322931449338, y: YB + -0.236040841866, z: ZB + 1.5},
    {x: XB + 0.345824712683, y: YB + -0.201010616879, z: ZB + 1.5},
    {x: XB + 0.364932885276, y: YB + -0.163780307864, z: ZB + 1.5},
    {x: XB + 0.380046826, y: YB + -0.124757404781, z: ZB + 1.5},
    {x: XB + 0.391001111055, y: YB + -0.0843690177356, z: ZB + 1.5},
    {x: XB + 0.397675844549, y: YB + -0.0430572022102, z: ZB + 1.5},
};

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
  sequenceInitStatic(&preRecorded, sizeof(circleSeqData) / sizeof(point_t), circleSeqData);
  sequenceInitStatic(&takeOffSeq, sizeof(takeOffSeqData) / sizeof(point_t), takeOffSeqData);
  sequenceInitStatic(&LandSeq, sizeof(landSeqData) / sizeof(point_t), landSeqData);

  setpoint.mode.x = modeAbs;
  setpoint.mode.y = modeAbs;
  setpoint.mode.z = modeAbs;
  setpoint.mode.yaw = modeAbs;
  setpoint.mode.roll = modeDisable;
  setpoint.mode.pitch = modeDisable;

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

  result = (count >= LOCK_LENGTH) && ((lXMax - lXMin) < LOCK_THRESHOLD) && ((lYMax - lYMin) < LOCK_THRESHOLD) && ((lZMax - lZMin) < LOCK_THRESHOLD && sensorsAreCalibrated());
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

  sequenceRecord(&sequence, &point);
}

static void moveSetPoint(point_t* point) {
  setpoint.position.x = point->x;
  setpoint.position.y = point->y;
  setpoint.position.z = point->z;

  // DEBUG_PRINT("Set (%d, %d, %d)\n", (int)(point->x * 100.0f), (int)(point->y * 100.0f), (int)(point->z * 100.0f));

  commanderSetSetpoint(&setpoint, 3);
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
  setpoint.thrust = 0;
  setpoint.mode.z = modeDisable;

  commanderSetSetpoint(&setpoint, 3);
}

static void exitStateStop() {
}
