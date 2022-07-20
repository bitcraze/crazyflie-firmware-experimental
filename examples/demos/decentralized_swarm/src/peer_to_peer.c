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
#include "positions.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"

#define BROADCAST_FEQUENCY_HZ 10
#define BROADCAST_PERIOD_MS (1000 / BROADCAST_FEQUENCY_HZ)

#define CALC_NEXT_FEQUENCY_HZ 3
#define CALC_NEXT_PERIOD_MS (1000 / CALC_NEXT_FEQUENCY_HZ)

#define INTER_DIST 0.6f
#define MAX_ADDRESS 10 //all copter addresses must be between 0 and max(MAX_ADDRESS,9)
#define LED_CRASH        LED_GREEN_R

// NEXT DELTA
#define MAXIMUM_NEXT_DELTA 0.2f

static xTimerHandle timer,timer2;
static bool isInit = false;


ledseqStep_t seq_crash_def[] = {
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

ledseqContext_t seq_crash = {
  .sequence = seq_crash_def,
  .led = LED_CRASH,
};

bool seq_crash_running = false;

// Log and param ids
static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;

static uint8_t my_id;

static Position others_pos[MAX_ADDRESS];
static uint32_t others_timestamp[MAX_ADDRESS]={0};

// Getting current position
static Position my_pos;

static float getX() { return (float) logGetFloat(logIdStateEstimateX)/1.0f; }
static float getY() { return (float) logGetFloat(logIdStateEstimateY)/1.0f; }
static float getZ() { return (float) logGetFloat(logIdStateEstimateZ)/1.0f; }

static void initializeOtherPositions() {
    for (int i = 0; i < MAX_ADDRESS; i++) {
        others_pos[i].x = FLT_MAX;
        others_pos[i].x = FLT_MAX;
        others_pos[i].y = FLT_MAX;
    }
}

void printOtherPositions() {
    for (int i = 0; i < MAX_ADDRESS; i++) {
        if (others_pos[i].x == FLT_MAX || others_pos[i].y == FLT_MAX || others_pos[i].y == FLT_MAX){
            continue;
        }
        DEBUG_PRINT("CF %d: %.2f %.2f %.2f timestamp: %lu\n", i, (double)others_pos[i].x, (double)others_pos[i].y,(double) others_pos[i].z,others_timestamp[i]);
        
    }
}

void p2pcallbackHandler(P2PPacket *p)
{
    // Parse the data from the other crazyflie and print it
    // uint8_t rssi = p->rssi;
    uint8_t received_id = p->data[0];
    // uint8_t counter = p->data[1];

    static Position pos_received;
    memcpy(&pos_received, &(p->data[2]), sizeof(Position));
    // DEBUG_PRINT("===================================================\n");
    // DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d  with counter: %d --> (%.2f , %.2f , %.2f)\n", rssi, received_id, counter,(double)pos_received.x,(double)pos_received.y,(double)pos_received.z);
    // // DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d  with counter: %d \n", rssi, received_id, counter);

    // // Store the position of the other crazyflie
    others_pos[received_id].x=pos_received.x;
    others_pos[received_id].y=pos_received.y;
    others_pos[received_id].z=pos_received.z;

    
    uint32_t now_ms = T2M(xTaskGetTickCount());
    // uint32_t delta = now_ms - others_timestamp[received_id];
    
    others_timestamp[received_id] = now_ms;

    // printOtherPositions();
}

// Initialize the p2p packet 
static P2PPacket p_reply;
static float previous[3];

static void initPacket(){
    p_reply.port=0x00;
    // Get the current address of the crazyflie and obtain
    //   the last two digits and send it as the first byte
    //   of the payload
    uint64_t address = configblockGetRadioAddress();
    my_id =(uint8_t)((address) & 0x00000000ff);
    p_reply.data[0]=my_id;
}

static void initLogIds(){
    logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logIdStateEstimateY = logGetVarId("stateEstimate", "y");
    logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
}

Position getNextDeltaPosition(uint8_t *copters_used) {
    Position p_i = my_pos;
    *copters_used = 0;
    Position delta_final={0,0,0};
    for (int j = 0; j < MAX_ADDRESS; j++) {
        if (others_pos[j].x == FLT_MAX || others_pos[j].y == FLT_MAX || others_pos[j].y == FLT_MAX){
            continue;
        }
        *copters_used = *copters_used + 1;

        Position p_j = others_pos[j];
        PRINT_POSITION_3D(p_j);
        float mag = getVectorMagnitude(subtractVectors3D(p_i, p_j));
        Position delta = subtractVectors3D(p_i, p_j);
        float scalar = (mag-INTER_DIST) / mag;
        MUL_VECTOR_3D_WITH_SCALAR(delta, scalar);
        ADD_VECTORS_3D(delta_final, delta);
    }

    MUL_VECTOR_3D_WITH_SCALAR(delta_final, -1.0f);
    
    // slace down the delta to a maximum of MAXIMUM_NEXT_DELTA
    float mag = getVectorMagnitude(delta_final);
    if (mag > MAXIMUM_NEXT_DELTA) {
        MUL_VECTOR_3D_WITH_SCALAR(delta_final, MAXIMUM_NEXT_DELTA / mag);
    }


    return delta_final;
}

static void sendPositionTimer(xTimerHandle timer) {
    static uint8_t counter=0;
    initPacket();

    my_pos.x=getX();
    my_pos.y=getY();
    my_pos.z=getZ();
    

    memcpy(&p_reply.data[2], &my_pos, sizeof(Position));
    
    if (previous[0]==my_pos.x && previous[1]==my_pos.y && previous[2]==my_pos.z) {
        // DEBUG_PRINT("Same value detected\n");
        if (!seq_crash_running){
            ledseqRun(&seq_crash);
            seq_crash_running=1;
        }

        logResetAll();//TODO: it seems to fix the problem, but I'm not sure about it
        initLogIds();
        
    }else{
        if (seq_crash_running)            
            ledseqStop(&seq_crash);
        seq_crash_running=0;
    }
    
    previous[0]=my_pos.x;
    previous[1]=my_pos.y;
    previous[2]=my_pos.z;
    
    p_reply.data[1] = counter++;
    
    //get current position and send it as the payload
    // DEBUG_PRINT("MY POSITION: "); PRINT_POSITION_3D(my_pos);
    p_reply.size=sizeof(Position)+2;//+2 for the id and counter
    radiolinkSendP2PPacketBroadcast(&p_reply);
}

static void calculateNextTimer(xTimerHandle timer){
    DEBUG_PRINT("===================================================\n");    
    uint8_t copters_used;
    Position delta = getNextDeltaPosition(&copters_used);
    DEBUG_PRINT("curr: %.2f %.2f %.2f copters used: %d --> Delta: %.2f %.2f %.2f\n", (double)my_pos.x,(double)my_pos.y,(double)my_pos.z, copters_used , (double)delta.x, (double)delta.y, (double)delta.z);
}


void appMain()
{
    if (isInit) {
        return;
    }

    DEBUG_PRINT("Waiting for activation ...\n");
    // Get log and param ids
    logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logIdStateEstimateY = logGetVarId("stateEstimate", "y");
    logIdStateEstimateZ = logGetVarId("stateEstimate", "z");

    initializeOtherPositions();

    ledseqRegisterSequence(&seq_crash);    

    initPacket();

    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(p2pcallbackHandler);
    
    // Position delta;
    
    previous[0]=0.0f;
    previous[1]=0.0f;
    previous[2]=0.0f;

    timer = xTimerCreate("AppTimer", M2T(BROADCAST_PERIOD_MS), pdTRUE, NULL, sendPositionTimer);
    xTimerStart(timer, 20);

    timer2 = xTimerCreate("AppTimer", M2T(CALC_NEXT_PERIOD_MS), pdTRUE, NULL, calculateNextTimer);
    xTimerStart(timer2, 20);

    isInit = true;

    
}

