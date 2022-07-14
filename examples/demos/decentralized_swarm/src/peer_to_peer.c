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

#define DEBUG_MODULE "P2P"
#include "debug.h"

#define MESSAGE "hello mlka"
#define MESSAGE_LENGHT 10

// Log and param ids
static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;

static float getX() { return logGetFloat(logIdStateEstimateX); }
static float getY() { return logGetFloat(logIdStateEstimateY); }
static float getZ() { return logGetFloat(logIdStateEstimateZ); }




void p2pcallbackHandler(P2PPacket *p)
{
  // Parse the data from the other crazyflie and print it
  uint8_t other_id = p->data[0];
  static float pos[3];
  memcpy(&pos, &p->data[1], sizeof(float)*3);
  
  uint8_t rssi = p->rssi;

  DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d, x:%.2f y:%.2f z:%.2f \n", rssi, other_id, (double)pos[0], (double)pos[1], (double)pos[2]);
}

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");
  // Get log and param ids
  logIdStateEstimateX = logGetVarId("stateEstimate", "x");
  logIdStateEstimateY = logGetVarId("stateEstimate", "y");
  logIdStateEstimateZ = logGetVarId("stateEstimate", "z");

  // Initialize the p2p packet 
  static P2PPacket p_reply;
  p_reply.port=0x00;
  
  // Get the current address of the crazyflie and obtain
  //   the last two digits and send it as the first byte
  //   of the payload
  uint64_t address = configblockGetRadioAddress();
  uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
  p_reply.data[0]=my_id;
  
  //Put a string in the payload
  char *str=MESSAGE;
  memcpy(&p_reply.data[1], str, sizeof(char)*MESSAGE_LENGHT);
  
  // Set the size, which is the amount of bytes the payload with ID and the string 
  p_reply.size=sizeof(char)*MESSAGE_LENGHT+1;
  
  // Register the callback function so that the CF can receive packets as well.
  p2pRegisterCB(p2pcallbackHandler);

  float pos[3];

  while(1) {
    // Send a message every 2 seconds
    //   Note: if they are sending at the exact same time, there will be message collisions, 
    //    however since they are sending every 2 seconds, and they are not started up at the same
    //    time and their internal clocks are different, there is not really something to worry about

    vTaskDelay(M2T(2000));
    
    //get current position and send it as the payload
    pos[0]=getX();
    pos[1]=getY();
    pos[2]=getZ();
    memcpy(&p_reply.data[1], pos, sizeof(float)*3);
    p_reply.size=sizeof(float)*3+1;

    radiolinkSendP2PPacketBroadcast(&p_reply);
  }
}

