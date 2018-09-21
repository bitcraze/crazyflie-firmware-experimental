#include <float.h>
#include <math.h>

#include "FreeRTOS.h"
#include "timers.h"
#include "deck_digital.h"
#include "deck_constants.h"
#include "sensors.h"
#include "estimator_kalman.h"
#include "crtp_commander_high_level.h"
#include "commander.h"
#include "pm.h"
#include "stabilizer.h"

#define DEBUG_MODULE "APP"
#include "debug.h"



static xTimerHandle timer;
static bool isInit = false;

static void appTimer(xTimerHandle timer);

#define LOCK_LENGTH 50
#define LOCK_THRESHOLD 0.001f
static uint32_t lockWriteIndex;
static float lockData[LOCK_LENGTH][3];
static void resetLockData();
static bool hasLock();

// Note: duration should be at the end!
// x0-x7, y0-y7, z0-z7, yaw0-yaw7, duration
static float sequence[] = {
    0.000000,-0.000000,0.000000,-0.000000,0.830443,-0.276140,-0.384219,0.180493,  -0.000000,0.000000,-0.000000,0.000000,-1.356107,0.688430,0.587426,-0.329106,  1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  1.050000,
    0.396058,0.918033,0.128965,-0.773546,0.339704,0.034310,-0.026417,-0.030049,   -0.445604,-0.684403,0.888433,1.493630,-1.361618,-0.139316,0.158875,0.095799,  1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.710000,
    0.922409,0.405715,-0.582968,-0.092188,-0.114670,0.101046,0.075834,-0.037926,  -0.291165,0.967514,0.421451,-1.086348,0.545211,0.030109,-0.050046,-0.068177,  1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.620000,
    0.923174,-0.431533,-0.682975,0.177173,0.319468,-0.043852,-0.111269,0.023166,  0.289869,0.724722,-0.512011,-0.209623,-0.218710,0.108797,0.128756,-0.055461,  1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.700000,
    0.405364,-0.834716,0.158939,0.288175,-0.373738,-0.054995,0.036090,0.078627,   0.450742,-0.385534,-0.954089,0.128288,0.442620,0.055630,-0.060142,-0.076163,  1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.560000,
    0.001062,-0.646270,-0.012560,-0.324065,0.125327,0.119738,0.034567,-0.063130,  0.001593,-1.031457,0.015159,0.820816,-0.152665,-0.130729,-0.045679,0.080444,  1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.560000,
    -0.402804,-0.820508,-0.132914,0.236278,0.235164,-0.053551,-0.088687,0.031253, -0.449354,-0.411507,0.902946,0.185335,-0.239125,-0.041696,0.016857,0.016709,  1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.700000,
    -0.921641,-0.464596,0.661875,0.286582,-0.228921,-0.051987,0.004669,0.038463,  -0.292459,0.777682,0.565788,-0.432472,-0.060568,-0.082048,-0.009439,0.041158, 1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.620000,
    -0.923935,0.447832,0.627381,-0.259808,-0.042325,-0.032258,0.001420,0.005294,  0.288570,0.873350,-0.515586,-0.730207,-0.026023,0.288755,0.215678,-0.148061,  1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.710000,
    -0.398611,0.850510,-0.144007,-0.485368,-0.079781,0.176330,0.234482,-0.153567, 0.447039,-0.532729,-0.855023,0.878509,0.775168,-0.391051,-0.713519,0.391628,  1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,  1.053185,
};


void appInit() {
  if (isInit) {
    return;
  }

  timer = xTimerCreate("AppTimer", M2T(100), pdTRUE, NULL, appTimer);
  xTimerStart(timer, 100);

  pinMode(DECK_GPIO_IO3, INPUT_PULLUP);

  setControllerType(ControllerTypeMellinger);

  commanderEnableHighLevel(true);
  crtpCommanderHighLevelDefineTrajectory(1, 0, sequence, sizeof(sequence));

  resetLockData();

  isInit = true;
}

static bool isButtonPressed() {
  return !digitalRead(DECK_GPIO_IO3);
}

enum State {
  STATE_IDLE = 0,
  STATE_WAIT_FOR_POSITION_LOCK,
  STATE_TAKING_OFF,
  STATE_GOING_TO_INITIAL_POSITION,
  STATE_RUNNING_TRAJECTORY,

  // Low battery states
  STATE_GOING_TO_PAD,
  STATE_LANDING,
  STATE_EXHAUSTED,
};

static enum State state = STATE_IDLE;

#define TAKE_OFF_HEIGHT 0.5
#define SEQUENCE_SPEED 1.0

static void appTimer(xTimerHandle timer) {
  if (isBatLow() & (state < STATE_GOING_TO_PAD)) {
    DEBUG_PRINT("Battery low, going to pad...\n");
    crtpCommanderHighLevelGoTo(0.7, -0.7, 0.4, 0.0, 2.0, false, 0);
    state = STATE_GOING_TO_PAD;
  }

  switch(state) {
    case STATE_IDLE:
      if (isButtonPressed()) {
        DEBUG_PRINT("Let's go! Waiting for position lock...\n");
        state = STATE_WAIT_FOR_POSITION_LOCK;
      }
      break;
    case STATE_WAIT_FOR_POSITION_LOCK:
      if (hasLock()) {
        DEBUG_PRINT("Position lock acquired, taking off..\n");
        crtpCommanderHighLevelTakeOff(TAKE_OFF_HEIGHT, 1.0, 0);
        state = STATE_TAKING_OFF;
      }
      break;
    case STATE_TAKING_OFF:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Hovering, going to initial position...\n");
        crtpCommanderHighLevelGoTo(sequence[0], sequence[8], sequence[16], sequence[24], 2.0, false, 0);
        state = STATE_GOING_TO_INITIAL_POSITION;
      }
      break;
    case STATE_GOING_TO_INITIAL_POSITION:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("At initial position, starting trajectory...\n");
        crtpCommanderHighLevelStartTrajectory(1, SEQUENCE_SPEED, false, false, 0);
        state = STATE_RUNNING_TRAJECTORY;
      }
      break;
    case STATE_RUNNING_TRAJECTORY:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Trajectory finished, restarting...\n");
        crtpCommanderHighLevelStartTrajectory(1, SEQUENCE_SPEED, false, false, 0);
      }
      break;
    case STATE_GOING_TO_PAD:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("At pad, landing...\n");
        crtpCommanderHighLevelLand(0.02, 0.5, 0);
        state = STATE_LANDING;
      }
      break;
    case STATE_LANDING:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Landed. Feed me!\n");
        crtpCommanderHighLevelStop();
        state = STATE_EXHAUSTED;
      }
      break;
    default:
      break;
  }
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

  // TODO krri Check that all anchors are received?

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
