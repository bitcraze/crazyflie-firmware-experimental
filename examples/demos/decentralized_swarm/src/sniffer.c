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
extern copter_t copters[MAX_ADDRESS];
static copter_t prev_copters[MAX_ADDRESS];
static bool prev_copters_active[MAX_ADDRESS];

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
            printOtherCopters();
            prev_copters[i] = copters[i];
            prev_copters_active[i] = isAlive; 
        }    
    }
}

void appMain()
{
    DEBUG_PRINT("Running Decentralized swarm sniffer ...\n");
    
   initOtherStates();
    
    // Register the callback function so that the CF can receive packets as well.
    p2pRegisterCB(p2pcallbackHandler);

    while(1) {
        vTaskDelay(M2T(SNIFFER_PRINT_PERIOD_MS));
        // printOtherCopters();
        update_copter_list();
    }
}

#endif // BUILD_SNIFFER_APP