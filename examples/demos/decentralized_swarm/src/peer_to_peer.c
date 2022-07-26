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
// #include "sensors.h"
#include "pm.h"
#include "supervisor.h"
// #include "peer_localization.h"
#include "settings.h"

#include "p2p_interface.h"
#include "positions.h"
#include "common.h"
#include "param_log_interface.h"
#include "movement.h"

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

static float previous[3];
static float padX = 0.0;
static float padY = 0.0;
static float padZ = 0.0;

static uint32_t now = 0;
static uint32_t hovering_start_time = 0;
static uint32_t position_lock_start_time = 0;
static uint32_t random_time_for_next_event = 0;

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


static bool takeOffWhenReady = false;
static bool terminateTrajectoryAndLand = false;

Position offset = {0, 0, 0};

static void initPacket(){
    p_reply.port=0x00;
    // Get the current address of the crazyflie and obtain
    //   the last two digits and send it as the first byte
    //   of the payload
    uint64_t address = configblockGetRadioAddress();
    my_id =(uint8_t)((address) & 0x00000000ff);
    p_reply.data[0]=my_id;
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

static bool selfIsFlying(void){
    return state>STATE_PREPARING_FOR_TAKE_OFF && state<STATE_GOING_TO_PAD;
}

static uint8_t flying_copters_number(){
    //Number of flying copters including the current one (if it is flying)
    uint8_t flying_drones = peerLocalizationGetNumNeighbors();
    if (selfIsFlying())
        flying_drones++;

    return flying_drones;
}

static bool needExtraCopters(void) {
    uint8_t flying_drones = flying_copters_number();
    return flying_drones >= 0 && flying_drones < DESIRED_FLYING_COPTERS;
}

static bool needLessCopters(void){
    return flying_copters_number() > DESIRED_FLYING_COPTERS;
}

static bool allFlyingCoptersHovering(void){
    for (uint8_t i = 0; i < MAX_ADDRESS; i++) {
        uint8_t state=getCopterState(i);
        if (state==255 || !peerLocalizationIsIDActive(i) ){// if not active
            continue;
        }

        if (state > STATE_WAIT_FOR_TAKE_OFF  && state<STATE_GOING_TO_PAD){//TODO:maybe change STATE_WAIT_FOR_TAKE_OFF to STATE_PREPARING_FOR_TAKE_OFF
            if (state!=STATE_HOVERING && state!=STATE_GOING_TO_RANDOM_POINT )
                return false;
        } 
    }
    return true;
}

static void startTakeOffSequence(){
    takeOffWhenReady = false;
    padX = getX();
    padY = getY();
    padZ = getZ();
    DEBUG_PRINT("Base position: (%f, %f, %f)\n", (double)padX, (double)padY, (double)padZ );
    
    terminateTrajectoryAndLand = false;
    DEBUG_PRINT("Taking off...\n");
    crtpCommanderHighLevelTakeoff(padZ + TAKE_OFF_HEIGHT, 1.0);
}

static void stateTransition(xTimerHandle timer){
    // In the following checks , sequence of checks is important 
    
    if(supervisorIsTumbled()) {
        state = STATE_CRASHED;
    }
    else if (outOfBounds(my_pos)) {
        if (state != STATE_LANDING){
            crtpCommanderHighLevelLand(padZ, SAFETY_LANDING_DURATION);
            state = STATE_LANDING;
        }
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
            if (!chargedForTakeoff()){
                break;
            }

            if (takeOffWhenReady) {
                startTakeOffSequence();
                state = STATE_TAKING_OFF;
            }

            if (needExtraCopters() && atLeastOneCopterHasFlown() ){// at least one copter is flying and  more copters needed
                random_time_for_next_event = now + (rand() % (TAKE_OFF_TIME_MAX - TAKE_OFF_TIME_MIN)) + TAKE_OFF_TIME_MIN;
                DEBUG_PRINT("Preparing for take off...\n");
                state=STATE_PREPARING_FOR_TAKE_OFF;
            }
            
        break;
        case STATE_PREPARING_FOR_TAKE_OFF:
            if (!needExtraCopters()){ // another copter took off,no need to take off finally
                DEBUG_PRINT("Another copter took off, no need to take off finally\n");
                state=STATE_WAIT_FOR_TAKE_OFF;
            }
            else if (now > random_time_for_next_event){
                DEBUG_PRINT("Taking off...\n");
                startTakeOffSequence();
                state=STATE_TAKING_OFF;
            }

            break;  
        case STATE_TAKING_OFF:
            if ( needLessCopters() ){// more than desired copters are flying,need to land
                DEBUG_PRINT("More copters than desired are flying while taking off, need to land\n");
                random_time_for_next_event = now + (rand() % (TAKE_OFF_TIME_MAX - TAKE_OFF_TIME_MIN)) + TAKE_OFF_TIME_MIN;
                state = STATE_PREPARING_FOR_LAND;
                break;
            }

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
                gotoNextWaypoint(padX,padY,padZ+TAKE_OFF_HEIGHT,GO_TO_PAD_DURATION);
                state = STATE_GOING_TO_PAD;
                break;
            }
            
            if ( needLessCopters() ){
                DEBUG_PRINT("More copters than desired are flying while hovering, need to land\n");
                random_time_for_next_event = now + (rand() % (TAKE_OFF_TIME_MAX - TAKE_OFF_TIME_MIN)) + TAKE_OFF_TIME_MIN;
                state = STATE_PREPARING_FOR_LAND;
                break;
            }

            // wait for all flying copters to be hovering
            if (allFlyingCoptersHovering()){
                DEBUG_PRINT("All copters are hovering, going to next Waypoint\n");
                Position new_pos = getRandomPositionOnCircle();
                gotoNextWaypoint(new_pos.x,new_pos.y,new_pos.z,DELTA_DURATION);

                state = STATE_GOING_TO_RANDOM_POINT;
            }
            break;
        case STATE_GOING_TO_RANDOM_POINT:
            if (reachedNextWaypoint(my_pos)) {
                DEBUG_PRINT("Reached next waypoint\n");
                state = STATE_HOVERING;                    
            }
            break;
        case STATE_PREPARING_FOR_LAND:
            if (!needLessCopters()){ // another copter landed , no need to land finally
                DEBUG_PRINT("Another copter landed, no need to land finally\n");
                state=STATE_HOVERING;
            }
            else if (now > random_time_for_next_event){
                DEBUG_PRINT("Going to pad...\n");
                gotoNextWaypoint(padX,padY,padZ+TAKE_OFF_HEIGHT,GO_TO_PAD_DURATION);
                state=STATE_GOING_TO_PAD;
            } 

        case STATE_GOING_TO_PAD:
            if (reachedNextWaypoint(my_pos)) {
                DEBUG_PRINT("Over pad,starting lowering\n");
                crtpCommanderHighLevelGoTo(padX, padY, padZ + LANDING_HEIGHT, 0.0, GO_TO_PAD_DURATION, false);
                state = STATE_WAITING_AT_PAD;
            }
            break;
        case STATE_REGOING_TO_PAD:
            if (reachedNextWaypoint(my_pos) || now > stabilizeEndTime) {
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
                if( outOfBounds(my_pos) ){
                    DEBUG_PRINT("Landed because of out of bounds, going to crashed state \n");
                    state = STATE_CRASHED;
                }

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
