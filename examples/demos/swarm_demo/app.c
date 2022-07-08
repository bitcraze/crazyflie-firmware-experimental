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
#include "ledseq.h"
#include "log.h"
#include "param.h"
#include "supervisor.h"
#include "controller.h"
#include "ledseq.h"
#include "pptraj.h"
#include "lighthouse_position_est.h"
#include "lighthouse_core.h"

#define DEBUG_MODULE "APP"
#include "debug.h"


static xTimerHandle timer;
static bool isInit = false;

static void appTimer(xTimerHandle timer);

#define LED_LOCK         LED_GREEN_R
#define LOCK_LENGTH 50
#define LOCK_THRESHOLD 0.001f
#define MAX_PAD_ERR 0.005
#define TAKE_OFF_HEIGHT 0.2f
#define LANDING_HEIGHT 0.12f
#define SEQUENCE_SPEED 1.0f
#define DURATION_TO_INITIAL_POSITION 1.5
#define TRAJECTORY_SEGMENT_SIZE_BYTES 132

#define SAFETY_LAND_HEIGHT 0.05f
#define SAFETY_LAND_DURATION 3.0 // in seconds

static uint32_t lockWriteIndex;
static float lockData[LOCK_LENGTH][3];
static void resetLockData();
static bool hasLock();

// static bool getFirstWaypointofTraj(uint8_t traj_id,float wp[4]);

static bool takeOffWhenReady = false;
static float goToInitialPositionWhenReady = -1.0f;
static bool terminateTrajectoryAndLand = false;

static float padX = 0.0;
static float padY = 0.0;
static float padZ = 0.0;

static uint32_t landingTimeCheckCharge = 0;

static float stabilizeEndTime;

#define NO_PROGRESS -2000.0f
static float currentProgressInTrajectory = NO_PROGRESS;
static uint32_t trajectoryStartTime = 0;
static uint32_t timeWhenToGoToInitialPosition = 0;
// static float trajectoryDurationMs = 0.0f;

static float trajecory_center_offset_x = 0.0f;
static float trajecory_center_offset_y = 0.0f;
static float trajecory_center_offset_z = 0.0f;

static uint32_t now = 0;
static uint32_t flightTime = 0;

static int start_trajectory_result = 0;
// The nr of trajectories to fly
// static uint8_t trajectoryCount = 255;
static uint8_t remainingTrajectories = 0;

// The latest trajectory id
static uint8_t latestTrajectoryId = 255;
static uint8_t prevTrajectoryId = 255;

static uint8_t start_trajectory=0;
static uint8_t safety_land_flag=0;
static uint8_t reset_crash_state_flag = 0;

static uint8_t is_safety_landing=0;

// Log and param ids
static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;
static logVarId_t logIdKalmanVarPX;
static logVarId_t logIdKalmanVarPY;
static logVarId_t logIdKalmanVarPZ;
static logVarId_t logIdPmState;
static logVarId_t logIdlighthouseEstBs0Rt;
static logVarId_t logIdlighthouseEstBs1Rt;

static paramVarId_t paramIdStabilizerController;
static paramVarId_t paramIdCommanderEnHighLevel;
static paramVarId_t paramIdLighthouseMethod;
static paramVarId_t paramIdTrajcount;

//#define USE_MELLINGER

#define TRAJ_Y_OFFSET 0.35

enum State {
  // Initialization
  STATE_IDLE = 0,
  STATE_WAIT_FOR_POSITION_LOCK,

  STATE_WAIT_FOR_TAKE_OFF, // Charging
  STATE_TAKING_OFF,
  STATE_HOVERING,
  STATE_WAITING_TO_RECEIVE_TRAJECTORY,
  STATE_WAITING_TO_START_TRAJECTORY,
  STATE_RUNNING_TRAJECTORY,
  STATE_GOING_TO_PAD,
  STATE_WAITING_AT_PAD,
  STATE_LANDING,
  STATE_CHECK_CHARGING,
  STATE_REPOSITION_ON_PAD,
  STATE_CRASHED,
};

static enum State state = STATE_IDLE;

ledseqStep_t seq_lock_def[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_lock = {
  .sequence = seq_lock_def,
  .led = LED_LOCK,
};

const uint8_t trajectoryId = 1;



static float getX() { return logGetFloat(logIdStateEstimateX); }
static float getY() { return logGetFloat(logIdStateEstimateY); }
static float getZ() { return logGetFloat(logIdStateEstimateZ); }
static float getVarPX() { return logGetFloat(logIdKalmanVarPX); }
static float getVarPY() { return logGetFloat(logIdKalmanVarPY); }
static float getVarPZ() { return logGetFloat(logIdKalmanVarPZ); }
static bool isBatLow() { return logGetInt(logIdPmState) == lowPower; }
static bool isCharging() { return logGetInt(logIdPmState) == charging; }
static bool isLighthouseAvailable() { return logGetFloat(logIdlighthouseEstBs0Rt) >= 0.0f || logGetFloat(logIdlighthouseEstBs1Rt) >= 0.0f; }



#ifdef USE_MELLINGER
static void enableMellingerController() { paramSetInt(paramIdStabilizerController, ControllerTypeMellinger); }
#endif
static void enableHighlevelCommander() { paramSetInt(paramIdCommanderEnHighLevel, 1); }

static void defineLedSequence() {
  ledseqRegisterSequence(&seq_lock);
}

void appMain() {
  if (isInit) {
    return;
  }

  DEBUG_PRINT("This is a demo app\n");

  // Get log and param ids
  logIdStateEstimateX = logGetVarId("stateEstimate", "x");
  logIdStateEstimateY = logGetVarId("stateEstimate", "y");
  logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
  logIdKalmanVarPX = logGetVarId("kalman", "varPX");
  logIdKalmanVarPY = logGetVarId("kalman", "varPY");
  logIdKalmanVarPZ = logGetVarId("kalman", "varPZ");
  logIdPmState = logGetVarId("pm", "state");
  logIdlighthouseEstBs0Rt = logGetVarId("lighthouse", "estBs0Rt");
  logIdlighthouseEstBs1Rt = logGetVarId("lighthouse", "estBs1Rt");

  paramIdStabilizerController = paramGetVarId("stabilizer", "controller");
  paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
  paramIdLighthouseMethod = paramGetVarId("lighthouse", "method");
  paramIdTrajcount =paramGetVarId("app","trajcount");

  timer = xTimerCreate("AppTimer", M2T(20), pdTRUE, NULL, appTimer);
  xTimerStart(timer, 20);

  pinMode(DECK_GPIO_IO3, INPUT_PULLUP);

  #ifdef USE_MELLINGER
    enableMellingerController();
  #endif

  enableHighlevelCommander();
  // defineTrajectory();
  defineLedSequence();
  resetLockData();

  isInit = true;
}

static void appTimer(xTimerHandle timer) {
  uint32_t previous = now;
  now = xTaskGetTickCount();
  uint32_t delta = now - previous;

  if(supervisorIsTumbled()) {
    state = STATE_CRASHED;
  }

  if (isBatLow()) {
    terminateTrajectoryAndLand = true;
  }

  if (start_trajectory_result != 0) {
          DEBUG_PRINT("Error starting trajectory: %d\n", start_trajectory_result);
  }

  // safety landing check from control tower
  if (safety_land_flag==1){
    if (is_safety_landing==0){
      crtpCommanderHighLevelLand(0,SAFETY_LAND_DURATION);
      is_safety_landing=1;
    }else{
      // it is already landing
      if (crtpCommanderHighLevelIsTrajectoryFinished()){
        crtpCommanderHighLevelStop();
        state = STATE_IDLE;

        safety_land_flag=0;
        is_safety_landing=0;
      }
    }

    return;
  }

  // state crash resetting from control tower
  if (reset_crash_state_flag && !supervisorIsTumbled()) {
    reset_crash_state_flag = false;
    state = STATE_IDLE;
    resetLockData();
    enableHighlevelCommander();
    crtpCommanderHighLevelInit();
  }

  
  switch(state) {
    case STATE_IDLE:
      DEBUG_PRINT("Let's go! Waiting for position lock...\n");
      // prevent from sending the same trajectory again after retaking off
      prevTrajectoryId=latestTrajectoryId;
      state = STATE_WAIT_FOR_POSITION_LOCK;
      break;
    case STATE_WAIT_FOR_POSITION_LOCK:
      if (hasLock()) {
        DEBUG_PRINT("Position lock acquired, ready for take off..\n");
        ledseqRun(&seq_lock);
        state = STATE_WAIT_FOR_TAKE_OFF;
      }
      break;
    case STATE_WAIT_FOR_TAKE_OFF:
      trajectoryStartTime = 0;
      if (takeOffWhenReady) {
        takeOffWhenReady = false;
        DEBUG_PRINT("Taking off!\n");

        padX = getX();
        padY = getY();
        padZ = getZ();
        DEBUG_PRINT("Base position: (%f, %f, %f)\n", (double)padX, (double)padY, (double)padZ);

        terminateTrajectoryAndLand = false;
        crtpCommanderHighLevelTakeoff(padZ + TAKE_OFF_HEIGHT, 1.0);
        state = STATE_TAKING_OFF;
      }
      break;
    case STATE_TAKING_OFF:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Hovering, waiting for command to start\n");
        ledseqStop(&seq_lock);
        state = STATE_HOVERING;

      }
      flightTime += delta;
      break;
    case STATE_HOVERING:
      if (terminateTrajectoryAndLand) {
          terminateTrajectoryAndLand = false;
          DEBUG_PRINT("Terminating hovering\n");
          state = STATE_GOING_TO_PAD;
      } else {
        if (goToInitialPositionWhenReady >= 0.0f) {
          float delayMs = goToInitialPositionWhenReady *  3000.0f;
          timeWhenToGoToInitialPosition = now + delayMs;
          trajectoryStartTime = now + delayMs;
          goToInitialPositionWhenReady = -1.0f;
          DEBUG_PRINT("Waiting to go to initial position for %d ms\n", (int)delayMs);
          state = STATE_WAITING_TO_RECEIVE_TRAJECTORY;
          prevTrajectoryId=latestTrajectoryId;// prevent from sending the same trajectory again after retaking off
        }
      }
      flightTime += delta;
      break;
    case STATE_WAITING_TO_RECEIVE_TRAJECTORY:
      // latest trajectory id will be updated as soon as the trajectory is defined
      if (prevTrajectoryId==latestTrajectoryId) {
        // DEBUG_PRINT("Previous Traj id same with latest: %d\n", prevTrajectoryId);
        flightTime += delta;
        break;
      }

      prevTrajectoryId=latestTrajectoryId;
      state = STATE_WAITING_TO_START_TRAJECTORY;

      uint8_t temp= remainingTrajectories-1;
      paramSet(paramIdTrajcount.index,  &temp);

      flightTime += delta;
      break;
    case STATE_WAITING_TO_START_TRAJECTORY:
      if (start_trajectory==0) {// wait until receive start_trajectory signal
        flightTime += delta;
        break;
      }

      start_trajectory=0;
      DEBUG_PRINT("Starting traj with remain traj: %d\n", remainingTrajectories);

      start_trajectory_result = crtpCommanderHighLevelStartTrajectory(latestTrajectoryId, SEQUENCE_SPEED, false, false);
      state = STATE_RUNNING_TRAJECTORY;
      
      flightTime += delta;
      break;
    case STATE_RUNNING_TRAJECTORY:
      if (terminateTrajectoryAndLand){
        crtpCommanderHighLevelStop();
        terminateTrajectoryAndLand = false;
        DEBUG_PRINT("Terminating trajectory, going to pad mlk...\n");
        float timeToPadPosition = 2.0;
        crtpCommanderHighLevelGoTo(padX, padY, padZ + LANDING_HEIGHT, 0.0, timeToPadPosition, false);
        currentProgressInTrajectory = NO_PROGRESS;
        state = STATE_GOING_TO_PAD;
      }

      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Trajectory finished, remaining trajectories: %d\n", remainingTrajectories);
        if (terminateTrajectoryAndLand || (remainingTrajectories == 255)) { // 255 means all trajectories are finished and we are going to land
          terminateTrajectoryAndLand = false;
          DEBUG_PRINT("Terminating trajectory, going to pad...\n");
          float timeToPadPosition = 2.0;
          crtpCommanderHighLevelGoTo(padX, padY, padZ + LANDING_HEIGHT, 0.0, timeToPadPosition, false);
          currentProgressInTrajectory = NO_PROGRESS;
          state = STATE_GOING_TO_PAD;
        } else {
          if (remainingTrajectories >= 0) {
            float delayMs=3000.0f;
            timeWhenToGoToInitialPosition = now + delayMs;
            DEBUG_PRINT("Going to state WAITING_TO_RECEIVE_TRAJECTORY \n");
            state=STATE_WAITING_TO_RECEIVE_TRAJECTORY;
            
          }
        }
      }
      flightTime += delta;
      break;
    case STATE_GOING_TO_PAD:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Over pad, stabalizing position\n");
        stabilizeEndTime = now + 5000;
        state = STATE_WAITING_AT_PAD;
      }
      flightTime += delta;
      break;
    case STATE_WAITING_AT_PAD:
      if (now > stabilizeEndTime || ((fabs(padX - getX()) < MAX_PAD_ERR) && (fabs(padY - getY()) < MAX_PAD_ERR))) {
        if (now > stabilizeEndTime) {
          DEBUG_PRINT("Warning: timeout!\n");
        }

        DEBUG_PRINT("Landing...\n");
        crtpCommanderHighLevelLand(padZ, 1.0);
        state = STATE_LANDING;
      }
      flightTime += delta;
      break;
    case STATE_LANDING:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Landed. Feed me!\n");
        crtpCommanderHighLevelStop();
        landingTimeCheckCharge = now + 3000;
        state = STATE_CHECK_CHARGING;
      }
      flightTime += delta;
      break;
    case STATE_CHECK_CHARGING:
      if (now > landingTimeCheckCharge) {
        DEBUG_PRINT("isCharging: %d\n", isCharging());
        if (isCharging()) {
          ledseqRun(&seq_lock);
          state = STATE_IDLE;
        } else {
          DEBUG_PRINT("Not charging. Try to reposition on pad.\n");
          crtpCommanderHighLevelTakeoff(padZ + LANDING_HEIGHT, 1.0);
          state = STATE_REPOSITION_ON_PAD;
        }
      }
      break;
    case STATE_REPOSITION_ON_PAD:
      if (crtpCommanderHighLevelIsTrajectoryFinished()) {
        DEBUG_PRINT("Over pad, stabalizing position\n");
        crtpCommanderHighLevelGoTo(padX, padY, padZ + LANDING_HEIGHT, 0.0, 1.5, false);
        state = STATE_GOING_TO_PAD;
      }
      flightTime += delta;
      break;
    case STATE_CRASHED:
      crtpCommanderHighLevelStop();
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

  result =
    (count >= LOCK_LENGTH) &&
    ((lXMax - lXMin) < LOCK_THRESHOLD) &&
    ((lYMax - lYMin) < LOCK_THRESHOLD) &&
    ((lZMax - lZMin) < LOCK_THRESHOLD &&
    isLighthouseAvailable() &&  // Make sure we have a deck and the Lighthouses are powered
    sensorsAreCalibrated());

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


PARAM_GROUP_START(app)
  PARAM_ADD(PARAM_UINT8, takeoff, &takeOffWhenReady)
  PARAM_ADD(PARAM_FLOAT, start, &goToInitialPositionWhenReady)
  PARAM_ADD(PARAM_UINT8, stop, &terminateTrajectoryAndLand)
  PARAM_ADD(PARAM_FLOAT, offsx, &trajecory_center_offset_x)
  PARAM_ADD(PARAM_FLOAT, offsy, &trajecory_center_offset_y)
  PARAM_ADD(PARAM_FLOAT, offsz, &trajecory_center_offset_z)
  PARAM_ADD(PARAM_UINT8, trajcount, &remainingTrajectories)
  PARAM_ADD(PARAM_UINT8, curr_traj_id, &latestTrajectoryId)
  PARAM_ADD(PARAM_UINT8, start_traj, &start_trajectory)

  PARAM_ADD(PARAM_UINT8, safety_land, &safety_land_flag)
  PARAM_ADD(PARAM_UINT8, reset_crash_state, &reset_crash_state_flag)

PARAM_GROUP_STOP(app)

LOG_GROUP_START(app)
  LOG_ADD(LOG_UINT8, state, &state)
  LOG_ADD(LOG_FLOAT, prgr, &currentProgressInTrajectory)
  LOG_ADD(LOG_UINT32, uptime, &now)
  LOG_ADD(LOG_UINT32, flighttime, &flightTime)
LOG_GROUP_STOP(app)
