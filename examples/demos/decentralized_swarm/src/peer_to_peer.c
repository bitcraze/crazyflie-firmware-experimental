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

#define DEBUG_MODULE "P2P"
#include "debug.h"


#define INTER_DIST 0.6f
#define MAX_ADDRESS 9 //all copter addresses must be between 0 and max(MAX_ADDRESS,9)

typedef struct Position_struct {
    float x;
    float y;
    float z;
} Position;

//The following functions are used to return a new Position struct
//If you want 
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

// Log and param ids
static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;

static uint8_t my_id;

static Position others_pos[MAX_ADDRESS];

static float getX() { return logGetFloat(logIdStateEstimateX); }
static float getY() { return logGetFloat(logIdStateEstimateY); }
static float getZ() { return logGetFloat(logIdStateEstimateZ); }

static void initializeOtherPositions() {
    for (int i = 0; i < MAX_ADDRESS; i++) {
        others_pos[i].x = FLT_MAX;
        others_pos[i].x = FLT_MAX;
        others_pos[i].y = FLT_MAX;
    }
}

static Position getNextDeltaPosition() {
    Position p_i = {getX(), getY(), getZ()};

    Position delta_final={0,0,0};
    for (int j = 0; j < MAX_ADDRESS; j++) {
        if (others_pos[j].x == FLT_MAX || others_pos[j].y == FLT_MAX || others_pos[j].y == FLT_MAX){
            continue;
        }

        Position p_j = others_pos[j];
        float mag = getVectorMagnitude(subtractVectors3D(p_i, p_j));
        Position delta = subtractVectors3D(p_i, p_j);
        float scalar = (mag-INTER_DIST) / mag;
        delta = multiplyVectorWithScalar(delta, scalar);

    }

    return delta_final;
}



void p2pcallbackHandler(P2PPacket *p)
{
  // Parse the data from the other crazyflie and print it
  uint8_t other_id = p->data[0];
  static Position pos;
  memcpy(&pos, &p->data[1], sizeof(Position));

  uint8_t rssi = p->rssi;

  DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d, x:%.2f y:%.2f z:%.2f \n", rssi, other_id, (double)pos.x, (double)pos.y, (double)pos.z);
  
  // Store the position of the other crazyflie
  others_pos[other_id] = pos;
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

  Position pos;

  while(1) {
    // Send a message every 2 seconds
    //   Note: if they are sending at the exact same time, there will be message collisions, 
    //    however since they are sending every 2 seconds, and they are not started up at the same
    //    time and their internal clocks are different, there is not really something to worry about

    vTaskDelay(M2T(2000));
    
    //get current position and send it as the payload
    pos.x=getX();
    pos.y=getY();
    pos.z=getZ();

    memcpy(&p_reply.data[1], &pos, sizeof(Position));
    p_reply.size=sizeof(Position)+1;

    getNextDeltaPosition();

    radiolinkSendP2PPacketBroadcast(&p_reply);
  }
}

