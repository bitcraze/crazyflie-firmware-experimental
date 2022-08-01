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

#include "choose_app.h"
#ifdef BUILD_SNIFFER_APP

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
// #include "debug.h"

#include "p2p_interface.h"
#include "param_log_interface.h"
#include "common.h"

enum State state=STATE_SNIFFING;

extern copter_t copters[MAX_ADDRESS];
static copter_t prev_copters[MAX_ADDRESS];
static bool prev_copters_active[MAX_ADDRESS];
static P2PPacket p_reply;

void update_copter_list(void)
{   
    bool printed_bar = false;
    for (int i = 0; i < MAX_ADDRESS; i++)
    {   
        bool isAlive = peerLocalizationIsIDActive(i);
        bool changed_activity = prev_copters_active[i]!=isAlive;
        bool changed_state = prev_copters[i].state != copters[i].state; 
         
        if(  changed_state || changed_activity ){
            if (!printed_bar){
                DEBUG_PRINT("========================================================\n");
                printed_bar = true;
            }
            // DEBUG_PRINT("Copter %d state changed to %d\n", i, copters[i].state);
            // printOtherCopters();
            prev_copters[i] = copters[i];
            prev_copters_active[i] = isAlive; 
        }    
    }
}

/*
    PACKET FORMAT:
    [0]     --> id
    [1]     --> counter
    [2]     --> state
    [3-14]  --> x,y,z
    [15]    --> compressed Voltage 
    [16]    --> terminateApp 
*/

static void initPacket(){
    p_reply.port=0x00;
    // Get the current address of the crazyflie and obtain
    //   the last two digits and send it as the first byte
    //   of the payload
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    p_reply.data[0]=my_id;

    p_reply.size=sizeof(Position) + 5;//+5 for the id,counter,state ,Voltage and terminateApp
    Position pos = {20,20,20}; //Set the position to a non valid value to indicate that the copter is not interfering with the collision avoidance
    memcpy(&p_reply.data[3], &pos, sizeof(Position));
    p_reply.data[15] = 0;


}

static void prepare_packet(){
    static uint8_t counter=0;

    p_reply.data[1] = counter++;
    p_reply.data[2] = STATE_UNKNOWN;
    p_reply.data[16] = 0;

}

static void sendTakeOffPacket() {
    prepare_packet();
    p_reply.data[2] = STATE_HOVERING; //indicate that atr least one copter is flying

    radiolinkSendP2PPacketBroadcast(&p_reply);
}

static void sendTerminatePacket(){
    prepare_packet();
    p_reply.data[16] = 1; // terminateApp flag

    radiolinkSendP2PPacketBroadcast(&p_reply);
}

static bool at_least_one_flying(void){
    for (int i = 0; i < MAX_ADDRESS; i++)
    {   
        bool isFlying = copters[i].state >= STATE_TAKING_OFF && copters[i].state <= STATE_GOING_TO_PAD;
        if ( isFlying && isCopterIdActive(i) ){
            return true;
        }
    }

    return false;
}

static uint8_t othersActiveNumber(){
    uint8_t count=0;
    for (int i = 0; i < MAX_ADDRESS; i++)
    {   
        if ( isCopterIdActive(i) && !copters[i].terminateApp ){
            count++;
        }
    }
    return count;
}

void appMain()
{
    DEBUG_PRINT("Running Decentralized swarm sniffer ...\n");
    
    initOtherStates();
    initPacket();
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(p2pcallbackHandler);

    while(1) {
        vTaskDelay(M2T(SNIFFER_PRINT_PERIOD_MS));
        // printOtherCopters();
        // update_copter_list();

        if (getTakeOffWhenReady()){
            DEBUG_PRINT("Take off when ready\n");
            if (at_least_one_flying()){
                setTakeOffWhenReady(false);
            }else{
                sendTakeOffPacket();
            }
        }
        else if (getTerminateApp()){
            DEBUG_PRINT("Terminate app\n");
            if (othersActiveNumber() == 0){
                setTerminateApp(false);
            }else{
                sendTerminatePacket();
            }

        }

    }
}

//LOGS
#define add_copter_log(i)   LOG_GROUP_START(id_##i)\
                            LOG_ADD(LOG_UINT8, state, &copters[i].state)\
                            LOG_ADD(LOG_UINT8, voltage, &copters[i].battery_voltage)\
                            LOG_GROUP_STOP(id_i)

add_copter_log(1)
add_copter_log(2)
add_copter_log(3)
add_copter_log(4)
add_copter_log(5)
add_copter_log(6)
add_copter_log(7)
add_copter_log(8)
add_copter_log(9)



#endif // BUILD_SNIFFER_APP
