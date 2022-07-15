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


#define DEBUG_MODULE "P2P"
#include "debug.h"

#define BROADCAST_FEQUENCY_HZ 2
#define BROADCAST_PERIOD_MS (1000 / BROADCAST_FEQUENCY_HZ)
#define INTER_DIST 0.6f
#define MAX_ADDRESS 9 //all copter addresses must be between 0 and max(MAX_ADDRESS,9)

#define ADD_VECTORS_3D(a, b) {a.x += b.x; a.y += b.y; a.z += b.z;}
#define SUB_VECTORS_3D(a, b) {a.x -= b.x; a.y -= b.y; a.z -= b.z;}
#define MUL_VECTORS_3D(a, b) {a.x *= b.x; a.y *= b.y; a.z *= b.z;}
#define MUL_VECTOR_3D_WITH_SCALAR(v, scalar) {v.x *= scalar; v.y *= scalar; v.z *= scalar;}
#define PRINT_POSITION_3D(pos) {DEBUG_PRINT("(%f , %f , %f)\n", (double) pos.x, (double) pos.y, (double) pos.z);}

typedef struct Position_struct {
    float x;
    float y;
    float z;
} Position;

// Log and param ids
static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;

static uint8_t my_id;

static Position others_pos[MAX_ADDRESS];
static uint32_t others_timestamp[MAX_ADDRESS]={0};

// Getting current position
// static float getX() { return ( rand() % 200)/(-1.585f) ;}//logGetFloat(logIdStateEstimateX); }
// static float getY() { return ( rand() % 200)/(1.585f) ;}//logGetFloat(logIdStateEstimateY); }
// static float getZ() { return ( rand() % 200)/(-1.585f) ;}//logGetFloat(logIdStateEstimateZ); }

static float getX() { return (float) logGetFloat(logIdStateEstimateX)/1.0f; }
static float getY() { return (float) logGetFloat(logIdStateEstimateY)/1.0f; }
static float getZ() { return (float) logGetFloat(logIdStateEstimateZ)/1.0f; }



//The following functions are used to return a new Position struct
//If you want to save the result of teh calculation, you can use macros
Position addVectors3D(Position a, Position b) {
    Position result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

Position subtractVectors3D(Position a, Position b) {
    Position result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

Position multiplyVectorWithScalar(Position a, float scalar) {
    Position result;
    result.x = a.x * scalar;
    result.y = a.y * scalar;
    result.z = a.z * scalar;
    return result;
}

float getVectorMagnitude(Position a) {
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}


static void initializeOtherPositions() {
    for (int i = 0; i < MAX_ADDRESS; i++) {
        others_pos[i].x = FLT_MAX;
        others_pos[i].x = FLT_MAX;
        others_pos[i].y = FLT_MAX;
    }
}

// Position getNextDeltaPosition() {
//     Position p_i = {getX(), getY(), getZ()};

//     Position delta_final={0,0,0};
//     for (int j = 0; j < MAX_ADDRESS; j++) {
//         if (others_pos[j].x == FLT_MAX || others_pos[j].y == FLT_MAX || others_pos[j].y == FLT_MAX){
//             continue;
//         }

//         Position p_j = others_pos[j];
//         float mag = getVectorMagnitude(subtractVectors3D(p_i, p_j));
//         Position delta = subtractVectors3D(p_i, p_j);
//         float scalar = (mag-INTER_DIST) / mag;
//         MUL_VECTOR_3D_WITH_SCALAR(delta, scalar);
//         ADD_VECTORS_3D(delta_final, delta);
//     }

//     MUL_VECTOR_3D_WITH_SCALAR(delta_final, -1.0f);
    
//     return delta_final;
// }

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
    uint8_t other_id = p->data[0];
    uint8_t counter = p->data[1];

    static Position pos_received;
    memcpy(&pos_received, &(p->data[2]), sizeof(Position));

    DEBUG_PRINT("============================================================================\n");
    uint8_t rssi = p->rssi;
    for (int i = 2; i < 2+12; i++) {
        DEBUG_PRINT("%d ", p->data[i]);
    }
    DEBUG_PRINT("\n");

    DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d  with counter: %d --> (%.2f , %.2f , %.2f)\n", rssi, other_id, counter,(double)pos_received.x,(double)pos_received.y,(double)pos_received.z);
    // DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d  with counter: %d \n", rssi, other_id, counter);

    // Store the position of the other crazyflie
    others_pos[other_id].x=pos_received.x;
    others_pos[other_id].y=pos_received.y;
    others_pos[other_id].z=pos_received.z;

    
    uint32_t now_ms = T2M(xTaskGetTickCount());
    // uint32_t delta = now_ms - others_timestamp[other_id];
    
    others_timestamp[other_id] = now_ms;

    // printOtherPositions();
}

void appMain()
{
    DEBUG_PRINT("Waiting for activation ...\n");
    // Get log and param ids
    logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logIdStateEstimateY = logGetVarId("stateEstimate", "y");
    logIdStateEstimateZ = logGetVarId("stateEstimate", "z");

    initializeOtherPositions();

    // Initialize the p2p packet 
    static P2PPacket p_reply;
    p_reply.port=0x00;
    
    // Get the current address of the crazyflie and obtain
    //   the last two digits and send it as the first byte
    //   of the payload
    uint64_t address = configblockGetRadioAddress();
    my_id =(uint8_t)((address) & 0x00000000ff);
    p_reply.data[0]=my_id;
    
    
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(p2pcallbackHandler);

    
    uint8_t counter = 0;
    // Position delta;
    while(1) {

        vTaskDelay(M2T(BROADCAST_PERIOD_MS));
        
        p_reply.data[1] = counter++;
        
        Position my_pos;

        //get current position and send it as the payload
        my_pos.x=getX();
        my_pos.y=getY();
        my_pos.z=getZ();

        DEBUG_PRINT("MY POSITION: "); PRINT_POSITION_3D(my_pos);


        memcpy(&p_reply.data[2], &my_pos, sizeof(Position));

        DEBUG_PRINT("Sending POSITION: ");
        for (int i = 2; i < 2+12; i++) {
            DEBUG_PRINT("%d ", p_reply.data[i]);
        }
        DEBUG_PRINT("\n");

        p_reply.size=sizeof(Position)+2;//+2 for the id and counter

        radiolinkSendP2PPacketBroadcast(&p_reply);
    }
}

