/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * peer_to_peer.c - App layer application of simple demonstartion peer to peer
 *  communication. Two crazyflies need this program in order to send and receive.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#include "log.h"
#include "float.h"
#include <math.h>
#include <stdlib.h>
#include "estimator_kalman.h"
#include "ledseq.h"
#include "timers.h"
// #include "math3d.h" //TODO: import all functions and structs from math3d.h instead of mine
#include "param.h"
#include "crtp_commander_high_level.h"
#include "sensors.h"
#include "pm.h"
#include "supervisor.h"
#include "peer_localization.h"
#include "settings.h"

#include "p2p_interface.h"
#include "positions.h"
#include "common.h"
#include "param_log_interface.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"

static xTimerHandle sendPosTimer;
static xTimerHandle stateTransitionTimer;

static bool isInit = false;

static enum State state = STATE_IDLE;

static P2PPacket p_reply;

//Landing to pad
static uint32_t stabilizeEndTime;
static float landingTimeCheckCharge;

static uint8_t my_id;

Position positions_to_go[]={
    [0].x=+1 , [0].y=+1 ,[0].z = TAKE_OFF_HEIGHT,
    [1].x=+1 , [1].y=-1 ,[1].z = TAKE_OFF_HEIGHT,
    [2].x=-1 , [2].y=+1 ,[2].z = TAKE_OFF_HEIGHT,
    [3].x=-1 , [3].y=-1 ,[3].z = TAKE_OFF_HEIGHT,

};

static Position my_pos;
static Position next_wp={0,0,0};

static float previous[3];
static float padX = 0.0;
static float padY = 0.0;
static float padZ = 0.0;

static uint32_t now = 0;
static uint32_t hovering_start_time = 0;
static uint32_t position_lock_start_time = 0;

// LEDs Interface
static bool seq_estim_stuck_running = false;
static uint8_t seq_crash_running = 0;

ledseqStep_t seq_flashing_def[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(300)},

  {    0, LEDSEQ_LOOP},
  
};

ledseqContext_t seq_estim_stuck = {
  .sequence = seq_flashing_def,
  .led = LED_ESTIMATOR_STUCK,
};

ledseqContext_t seq_crash = {
  .sequence = seq_flashing_def,
  .led = LED_CRASH,
};


static float lockData[LOCK_LENGTH][3];
static bool takeOffWhenReady = false;
static bool terminateTrajectoryAndLand = false;
static uint32_t lockWriteIndex;

Position offset = {0, 0, 0};
static void resetLockData() {
    lockWriteIndex = 0;
    for (uint32_t i = 0; i < LOCK_LENGTH; i++) {
      lockData[i][0] = FLT_MAX;
      lockData[i][1] = FLT_MAX;
      lockData[i][2] = FLT_MAX;
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

static void initPacket(){
    p_reply.port=0x00;
    // Get the current address of the crazyflie and obtain
    //   the last two digits and send it as the first byte
    //   of the payload
    uint64_t address = configblockGetRadioAddress();
    my_id =(uint8_t)((address) & 0x00000000ff);
    p_reply.data[0]=my_id;
}


static bool outOfBounds() {
    return my_pos.x > MAX_X_BOUND || my_pos.x < MIN_X_BOUND 
        || my_pos.y > MAX_Y_BOUND || my_pos.y < MIN_Y_BOUND 
        || my_pos.z > MAX_Z_BOUND || my_pos.z < MIN_Z_BOUND;
}

static void gotoNextWaypoint(float x,float y,float z,float duration){
    next_wp.x = x;
    next_wp.y = y;
    next_wp.z = z;
    const float yaw = 0.0f;
    const bool relative = false;
    crtpCommanderHighLevelGoTo(next_wp.x, next_wp.y, next_wp.z, yaw, duration,relative);

}

static bool reachedNextWaypoint(){
    return DISTANCE3D(my_pos,next_wp) < WP_THRESHOLD  && getVelMagnitude() < WP_VEL_THRESHOLD; ;
}

// timers
static void sendPosition(xTimerHandle timer) {
    // Send the position to the other crazyflies via P2P

    /*
        PACKET FORMAT:
        [0]     --> id
        [1]     --> counter
        [2]     --> state
        [3-14]  --> x,y,z
        [15]    --> compressed Voltage    
    */
    if (state <= STATE_WAIT_FOR_TAKE_OFF || state >= STATE_REGOING_TO_PAD )
        return;
        
    static uint8_t counter=0;
    initPacket();

    my_pos.x=getX();
    my_pos.y=getY();
    my_pos.z=getZ();
    memcpy(&p_reply.data[3], &my_pos, sizeof(Position));
    
    // position stuck handling (while testing position broadcasting seem to be stuck after resetting the estimator)
    if (previous[0]==my_pos.x && previous[1]==my_pos.y && previous[2]==my_pos.z) {
        // DEBUG_PRINT("Same value detected\n");
        if (!seq_estim_stuck_running){
            ledseqRun(&seq_estim_stuck);
            seq_estim_stuck_running=1;
        }

        // logResetAll();//TODO: it seems to fix the problem, but I'm not sure about it
        initLogIds();
        
    }else{
        if (seq_estim_stuck_running)            
            ledseqStop(&seq_estim_stuck);
        seq_estim_stuck_running=0;
    }
    
    previous[0]=my_pos.x;
    previous[1]=my_pos.y;
    previous[2]=my_pos.z;
    
    p_reply.data[1] = counter++;
    p_reply.data[2] = (uint8_t) state;
    p_reply.data[15] = compressVoltage( getVoltage() );

    //get current position and send it as the payload
    p_reply.size=sizeof(Position) + 4;//+4 for the id,counter,state and Voltage

    radiolinkSendP2PPacketBroadcast(&p_reply);
}

static void stateTransition(xTimerHandle timer){
    // In the following checks , sequence of checks is important 
    
    if(supervisorIsTumbled()) {
        state = STATE_CRASHED;
    }
    else if (outOfBounds()) {
        DEBUG_PRINT("Out of bounds, stopping\n");
        crtpCommanderHighLevelLand(padZ, 1.0);
        state = STATE_LANDING;
    }
    else if (isBatLow()) {
        DEBUG_PRINT("Battery low, stopping\n");
        terminateTrajectoryAndLand=1;
    }

    now = xTaskGetTickCount();
    uint32_t dt;
    switch(state) {
        case STATE_IDLE:
        DEBUG_PRINT("Let's go! Waiting for position lock...\n");
        //reseting
        resetLockData();
        if (seq_crash_running == 1){
            ledseqStop(&seq_crash);
            seq_crash_running=0;
        }
        
        position_lock_start_time = now;
        state = STATE_WAIT_FOR_POSITION_LOCK;
        break;
        case STATE_WAIT_FOR_POSITION_LOCK:
        dt=now-position_lock_start_time;
        if (hasLock() || dt > POSITION_LOCK_TIMEOUT) {
            DEBUG_PRINT("Position lock acquired, ready for take off..\n");
            // ledseqRun(&seq_lock);
            state = STATE_WAIT_FOR_TAKE_OFF;
        }
        break;
        case STATE_WAIT_FOR_TAKE_OFF:
            if (takeOffWhenReady || peerLocalizationGetNumNeighbors() > 0 ) {
                takeOffWhenReady = false;
                padX = getX();
                padY = getY();
                padZ = getZ();
                DEBUG_PRINT("Base position: (%f, %f, %f)\n", (double)padX, (double)padY, (double)padZ);

                terminateTrajectoryAndLand = false;
                DEBUG_PRINT("Taking off...\n");
                crtpCommanderHighLevelTakeoff(padZ + TAKE_OFF_HEIGHT, 1.0);
                state = STATE_TAKING_OFF;
            }
        break;
        case STATE_TAKING_OFF:
            if (crtpCommanderHighLevelIsTrajectoryFinished()) {
                DEBUG_PRINT("Hovering, waiting for command to start\n");
                // ledseqStop(&seq_lock);
                state = STATE_HOVERING;
                DEBUG_PRINT("Enabling Collision Avoidance\n");
                enableCollisionAvoidance();

                hovering_start_time = now;
            }
            break;
        case STATE_HOVERING:
            dt = now - hovering_start_time ;
            if (terminateTrajectoryAndLand || dt > HOVERING_TIME) {
                gotoNextWaypoint(padX,padY,padZ+GO_TO_PAD_HEIGHT,GO_TO_PAD_DURATION);
                state = STATE_GOING_TO_PAD;
                break;
            }
            
            for (uint8_t i = 0; i < MAX_ADDRESS; i++) {
                uint8_t state=getCopterState(i);
                if (state!=255 && state<STATE_HOVERING){
                    break;
                } 
            }

            uint8_t curr_pos_id = getIdWithClosestDistance(my_pos,positions_to_go,4);
            
            uint8_t other_pos_id;
            if(curr_pos_id==0)
                other_pos_id = 3;
            else if (curr_pos_id==1)
                other_pos_id = 2;
            else if (curr_pos_id==2)
                other_pos_id = 1;
            else if (curr_pos_id==3)
                other_pos_id = 0;
            else
                break;

            gotoNextWaypoint((positions_to_go[other_pos_id]).x , (positions_to_go[other_pos_id]).y , (positions_to_go[other_pos_id]).z , DELTA_DURATION);
            state=STATE_GOING_TO_DELTA_POINT;
            break;
        case STATE_GOING_TO_DELTA_POINT:
            if (reachedNextWaypoint()) {
                if (terminateTrajectoryAndLand) {
                    gotoNextWaypoint(padX,padY,padZ+TAKE_OFF_HEIGHT,GO_TO_PAD_DURATION);
                    state = STATE_GOING_TO_PAD;
                }else{
                    state = STATE_HOVERING;                    
                }
            }
            break;
        case STATE_GOING_TO_PAD:
            if (reachedNextWaypoint()) {
                DEBUG_PRINT("Over pad,starting lowering\n");
                crtpCommanderHighLevelGoTo(padX, padY, padZ + LANDING_HEIGHT, 0.0, GO_TO_PAD_DURATION, false);
                state = STATE_WAITING_AT_PAD;
            }
            break;
        case STATE_REGOING_TO_PAD:
            if (reachedNextWaypoint() || now > stabilizeEndTime) {
                DEBUG_PRINT("Over pad,starting lowering\n");
                crtpCommanderHighLevelGoTo(padX, padY, padZ + LANDING_HEIGHT, 0.0, GO_TO_PAD_DURATION, false);
                stabilizeEndTime = now + STABILIZE_TIMEOUT;
                state = STATE_WAITING_AT_PAD;
            }
            break;
        case STATE_WAITING_AT_PAD:
            if (now > stabilizeEndTime || ((fabs(padX - getX()) < MAX_PAD_ERR) && (fabs(padY - getY()) < MAX_PAD_ERR))) {
                if (now > stabilizeEndTime) {
                DEBUG_PRINT("Warning: timeout!\n");
                }

                DEBUG_PRINT("Landing...\n");
                crtpCommanderHighLevelLand(padZ, LANDING_DURATION);
                state = STATE_LANDING;
            }
            break;
        case STATE_LANDING:
            if (crtpCommanderHighLevelIsTrajectoryFinished()) {
                DEBUG_PRINT("Landed. Feed me!\n");
                crtpCommanderHighLevelStop();
                landingTimeCheckCharge = now + 3000;
                state = STATE_CHECK_CHARGING;
            }
            break;
        case STATE_CHECK_CHARGING:
            if (now > landingTimeCheckCharge) {
                DEBUG_PRINT("isCharging: %d\n", isCharging());
                if (isCharging()) {
                // ledseqRun(&seq_lock);
                state = STATE_WAIT_FOR_TAKE_OFF;
                } else {
                DEBUG_PRINT("Not charging. Try to reposition on pad.\n");
                crtpCommanderHighLevelTakeoff(padZ + LANDING_HEIGHT + 0.2f, 1.0);
                state = STATE_REPOSITION_ON_PAD;
                }
            }
            break;
        case STATE_REPOSITION_ON_PAD:
            if (crtpCommanderHighLevelIsTrajectoryFinished()) {
                DEBUG_PRINT("Over pad, stabilizing position\n");
                gotoNextWaypoint(padX, padY, padZ + LANDING_HEIGHT, 1.5);
                stabilizeEndTime = now + 3000;
                state = STATE_REGOING_TO_PAD;
            }
            break;
        case STATE_CRASHED:
            crtpCommanderHighLevelStop();
            if (seq_crash_running!=1){
                DEBUG_PRINT("Crashed, running crash sequence\n");
                ledseqRun(&seq_crash);
                seq_crash_running=1;
            }
        break;
        
        default:
        break;
    }
}

void appMain()
{
    if (isInit) {
        return;
    }

    DEBUG_PRINT("Waiting for activation ...\n");
    // Get log and param ids
    initParamLogInterface();

    ledseqRegisterSequence(&seq_estim_stuck);    
    ledseqRegisterSequence(&seq_crash);

    initPacket();
    initOtherStates();

    initCollisionAvoidance();
    enableHighlevelCommander();
    
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(p2pcallbackHandler);
    
    previous[0]=0.0f;
    previous[1]=0.0f;
    previous[2]=0.0f;

    sendPosTimer = xTimerCreate("SendPosTimer", M2T(BROADCAST_PERIOD_MS), pdTRUE, NULL, sendPosition);
    xTimerStart(sendPosTimer, 20);

    stateTransitionTimer = xTimerCreate("AppTimer", M2T(CALC_NEXT_PERIOD_MS), pdTRUE, NULL, stateTransition);
    xTimerStart(stateTransitionTimer, 20);

    isInit = true;
}


PARAM_GROUP_START(app)
  PARAM_ADD(PARAM_UINT8, takeoff, &takeOffWhenReady)
  PARAM_ADD(PARAM_UINT8, stop, &terminateTrajectoryAndLand)
  PARAM_ADD(PARAM_FLOAT, offsx, &offset.x)
  PARAM_ADD(PARAM_FLOAT, offsy, &offset.y)
  PARAM_ADD(PARAM_FLOAT, offsz, &offset.z)
PARAM_GROUP_STOP(app)

LOG_GROUP_START(app)
  LOG_ADD(LOG_UINT8, state, &state)
LOG_GROUP_STOP(app)
