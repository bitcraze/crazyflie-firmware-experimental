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

#define DEBUG_MODULE "RETRACE"

#define LED_RETRACE LED_GREEN_R

typedef enum {
  ST_UNINIT = 0,
  ST_WAIT_POS_LOCK,
  ST_TAKE_OFF,
  ST_RECORD_TRACE,
  ST_RETRACE,
  ST_PLAY_PRE_RECORDED,
  ST_LAND,
  ST_STOP
} rt_state_t;


static void exitStateUninit() {}

static void enterStateWaitPosLock();
static void handleStateWaitPosLock();
static void exitStateWaitPosLock();

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

void (*stateEnter[])() = {
  0,
  enterStateWaitPosLock,
  enterStateTakeOff,
  enterStateRecordTrace,
  enterStateRetrace,
  enterStatePlayPreRecorded,
  enterStateLand,
  enterStateStop,
};

void (*stateHandle[])() = {
  0,
  handleStateWaitPosLock,
  handleStateTakeOff,
  handleStateRecordTrace,
  handleStateRetrace,
  handleStatePlayPreRecorded,
  handleStateLand,
  handleStateStop,
};

void (*stateExit[])() = {
  exitStateUninit,
  exitStateWaitPosLock,
  exitStateTakeOff,
  exitStateRecordTrace,
  exitStateRetrace,
  exitStatePlayPreRecorded,
  exitStateLand,
  exitStateStop,
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


#define XB 4.7
#define YB 1.9
#define ZB 1.0

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
};

static point_t landSeqData[] = {
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.95},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.9},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.85},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.8},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.75},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.7},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.65},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.6},
    {x: XB + 0.0, y: YB + 0.0, z: ZB + 0.55},
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
    {x: XB + 0.5, y: YB + 0.0, z: ZB + 1.0},
    {x: XB + 0.497263721611, y: YB + 0.0522378327457, z: ZB + 1.0},
    {x: XB + 0.48908483532, y: YB + 0.10390391648, z: ZB + 1.0},
    {x: XB + 0.475552859968, y: YB + 0.154432760049, z: ZB + 1.0},
    {x: XB + 0.45681590456, y: YB + 0.203271319523, z: ZB + 1.0},
    {x: XB + 0.433079047203, y: YB + 0.249885051322, z: ZB + 1.0},
    {x: XB + 0.404602090494, y: YB + 0.293763762857, z: ZB + 1.0},
    {x: XB + 0.371696717959, y: YB + 0.334427196649, z: ZB + 1.0},
    {x: XB + 0.334723082637, y: YB + 0.371430286797, z: ZB + 1.0},
    {x: XB + 0.294085865166, y: YB + 0.404368030277, z: ZB + 1.0},
    {x: XB + 0.250229844504, y: YB + 0.432879919746, z: ZB + 1.0},
    {x: XB + 0.203635029779, y: YB + 0.456653889337, z: ZB + 1.0},
    {x: XB + 0.154811406529, y: YB + 0.475429730253, z: ZB + 1.0},
    {x: XB + 0.104293354854, y: YB + 0.489001938783, z: ZB + 1.0},
    {x: XB + 0.0526338005666, y: YB + 0.497221965563, z: ZB + 1.0},
    {x: XB + 0.000398163355367, y: YB + 0.499999841466, z: ZB + 1.0},
    {x: XB + -0.051841831799, y: YB + 0.497305162326, z: ZB + 1.0},
    {x: XB + -0.103514412217, y: YB + 0.489167421711, z: ZB + 1.0},
    {x: XB + -0.154054015639, y: YB + 0.475675688117, z: ZB + 1.0},
    {x: XB + -0.202907480366, y: YB + 0.4569776301, z: ZB + 1.0},
    {x: XB + -0.249540099678, y: YB + 0.433277900028, z: ZB + 1.0},
    {x: XB + -0.293441474262, y: YB + 0.404835894139, z: ZB + 1.0},
    {x: XB + -0.334131098588, y: YB + 0.371962913415, z: ZB + 1.0},
    {x: XB + -0.371163620096, y: YB + 0.335018756366, z: ZB + 1.0},
    {x: XB + -0.404133713635, y: YB + 0.294407780984, z: ZB + 1.0},
    {x: XB + -0.432680517785, y: YB + 0.250574479007, z: ZB + 1.0},
    {x: XB + -0.456491584533, y: YB + 0.203998610902, z: ZB + 1.0},
    {x: XB + -0.475306299051, y: YB + 0.155189954836, z: ZB + 1.0},
    {x: XB + -0.488918732152, y: YB + 0.104682727091, z: ZB + 1.0},
    {x: XB + -0.497179894208, y: YB + 0.0530297350104, z: ZB + 1.0},
    {x: XB + -0.499999365864, y: YB + 0.000796326458243, z: ZB + 1.0},
    {x: XB + -0.497346287681, y: YB + -0.0514457979775, z: ZB + 1.0},
    {x: XB + -0.489249697903, y: YB + -0.103124842312, z: ZB + 1.0},
    {x: XB + -0.475798214623, y: YB + -0.153675173537, z: ZB + 1.0},
    {x: XB + -0.457139065853, y: YB + -0.202543512537, z: ZB + 1.0},
    {x: XB + -0.433476478096, y: YB + -0.249194989792, z: ZB + 1.0},
    {x: XB + -0.405069441062, y: YB + -0.293118999585, z: ZB + 1.0},
    {x: XB + -0.372228872997, y: YB + -0.333834788642, z: ZB + 1.0},
    {x: XB + -0.335314217647, y: YB + -0.370896718028, z: ZB + 1.0},
    {x: XB + -0.294729510107, y: YB + -0.403899140717, z: ZB + 1.0},
    {x: XB + -0.250918954611, y: YB + -0.432480841445, z: ZB + 1.0},
    {x: XB + -0.204362062663, y: YB + -0.456328990252, z: ZB + 1.0},
    {x: XB + -0.155568404732, y: YB + -0.475182566441, z: ZB + 1.0},
    {x: XB + -0.105072032945, y: YB + -0.488835215479, z: ZB + 1.0},
    {x: XB + -0.0534256358262, y: YB + -0.497137507574, z: ZB + 1.0},
    {x: XB + -0.00119448905614, y: YB + -0.499998573194, z: ZB + 1.0},
    {x: XB + 0.0510497315323, y: YB + -0.497387097652, z: ZB + 1.0},
    {x: XB + 0.102735207012, y: YB + -0.489331663844, z: ZB + 1.0},
    {x: XB + 0.153296233985, y: YB + -0.475920439408, z: ZB + 1.0},
    {x: XB + 0.202179416268, y: YB + -0.457300211718, z: ZB + 1.0},
    {x: XB + 0.248849721882, y: YB + -0.433674781281, z: ZB + 1.0},
    {x: XB + 0.292796339031, y: YB + -0.405302731116, z: ZB + 1.0},
    {x: XB + 0.333538267, y: YB + -0.372494596534, z: ZB + 1.0},
    {x: XB + 0.37062958076, y: YB + -0.335609466293, z: ZB + 1.0},
    {x: XB + 0.403664311672, y: YB + -0.295051052332, z: ZB + 1.0},
    {x: XB + 0.432280890853, y: YB + -0.251263271099, z: ZB + 1.0},
    {x: XB + 0.456166106595, y: YB + -0.20472538483, z: ZB + 1.0},
    {x: XB + 0.475058532499, y: YB + -0.155946755976, z: ZB + 1.0},
    {x: XB + 0.488751388819, y: YB + -0.105461272169, z: ZB + 1.0},
    {x: XB + 0.497094805686, y: YB + -0.0538215027628, z: ZB + 1.0},
};

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

  stateHandle[state]();
}

static void changeState(rt_state_t newState) {
  if (state != newState) {
    DEBUG_PRINT("Go to state %d\n", newState);
    stateExit[state]();
    state = newState;
    stateEnter[state]();
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

  DEBUG_PRINT("Rec (%d, %d, %d)\n", (int)(point.x * 100.0f), (int)(point.y * 100.0f), (int)(point.z * 100.0f));

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
    changeState(ST_TAKE_OFF);
  }
}

static void exitStateWaitPosLock() {
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
    sequenceReset(&sequence);
}

static void handleStateRetrace() {
  if (sequenceHasNext(&sequence)) {
    point_t* point = sequenceReplay(&sequence);
    moveSetPoint(point);
  } else {
    DEBUG_PRINT("End of retrace\n");
    changeState(ST_LAND);
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
