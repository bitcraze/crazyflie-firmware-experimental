/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 * p2p_interface.c
 * Peer to peer interface communication.
 */

#include "p2p_interface.h"

extern enum  State state;

copter_t copters[MAX_ADDRESS];
// uint8_t otherStates[MAX_ADDRESS];//array of states of the other drones

uint8_t getCopterState(uint8_t copter_id){
    return copters[copter_id].state;
}

void p2pcallbackHandler(P2PPacket *p)
{   
    #ifdef BUILD_PILOT_APP
        if (p->port == 0x01){ // If packet is only for the sniffer,ignore it
            return;
        }
    #endif
    // Parse the data from the other crazyflie and print it
    // uint8_t rssi = p->rssi;
    uint8_t received_id = p->data[0];
    // uint8_t counter = p->data[1];
    uint32_t now_ms = T2M(xTaskGetTickCount());

    copters[received_id].id = received_id;
    copters[received_id].counter = p->data[1];
    copters[received_id].state = p->data[2];
    copters[received_id].battery_voltage = p->data[15];
    copters[received_id].terminateApp = p->data[16] == 1;

    copters[received_id].timestamp = now_ms;

    if (copters[received_id].terminateApp){
        DEBUG_PRINT("Copter %d has requested to terminate the application\n", received_id);
        if (!getTerminateApp() && 
            state != STATE_SNIFFING && state != STATE_CRASHED &&
            state != STATE_WAIT_FOR_POSITION_LOCK && state != STATE_WAIT_FOR_STOPPED_TERMINATION_BROADCAST){
            setTerminateApp(true);
        }
    }

    positionMeasurement_t pos_measurement;
    memcpy(&pos_measurement.pos, &(p->data[3]), sizeof(Position));
    
    // DEBUG_PRINT("===================================================\n");
    // DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d  with counter: %d --> (%.2f , %.2f , %.2f)\n", rssi, received_id, counter,(double)pos_received.x,(double)pos_received.y,(double)pos_received.z);    
    
    pos_measurement.source =  MeasurementSourceLighthouse;
    pos_measurement.stdDev = 0.01f; //
    peerLocalizationTellPosition(received_id,&pos_measurement);//TODO: if id is 0--> PROBLEM WITH THE LOGIC OF THE PEER LOCALIZATION (maybe add 1 to the id)
}

void initOtherStates(){
    for(int i=0;i<MAX_ADDRESS;i++){
        copters[i].state = STATE_UNKNOWN;
    }
}

bool isAlive(uint8_t copter_id){
    uint32_t dt = T2M(xTaskGetTickCount()) - copters[copter_id].timestamp;
    return dt < ALIVE_TIMEOUT_MS;
}

uint8_t compressVoltage(float voltage){
    return (uint8_t) ((voltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 255);
}

float decompressVoltage(uint8_t voltage){
    return (voltage / 255.0f) * (VOLTAGE_MAX - VOLTAGE_MIN) + VOLTAGE_MIN;
}

bool atLeastOneCopterHasFlown(void){
    for(int i = 0; i < MAX_ADDRESS; i++){
        if(copters[i].state != STATE_UNKNOWN){
            return true;
        }
    }
    return false;
}

void printOtherCopters(void){
    for(int i = 0; i<MAX_ADDRESS; i++){
        if (copters[i].state != STATE_UNKNOWN){
            if (!peerLocalizationIsIDActive(i)){
                DEBUG_PRINT("Copter %d is not active\n",i);
            }else{
                peerLocalizationOtherPosition_t *pos = peerLocalizationGetPositionByID(i);
                DEBUG_PRINT("Copter %d : %.2f , %.2f , %.2f --> %d with latest counter %d \n",i,(double)pos->pos.x,(double)pos->pos.y,(double)pos->pos.z,copters[i].state,copters[i].counter);
            }
        }
    }
}

uint8_t otherCoptersActiveNumber(void){
    uint8_t nr=0;
    for(int i = 0; i < MAX_ADDRESS; i++){
        // if they are active and not requesting to terminate the application
        if (isCopterIdActive(i) && !copters[i].terminateApp && copters[i].state != STATE_TAKING_OFF){
            nr++;
        }
    }
    return nr;
}

bool selfIsFlying(void){
    return state > STATE_PREPARING_FOR_TAKE_OFF && state < STATE_GOING_TO_PAD && !getTerminateApp();
}

bool isCopterIdActive(uint8_t copter_id){
    uint32_t now = T2M(xTaskGetTickCount());
    uint32_t dt = now - copters[copter_id].timestamp;
    return dt < ALIVE_TIMEOUT_MS;
}

bool isCopterFlying(uint8_t copter_id){
    bool state_condition = copters[copter_id].state >=  STATE_TAKING_OFF && copters[copter_id].state < STATE_GOING_TO_PAD;
    return state_condition && isCopterIdActive(copter_id); ;
}

uint8_t getMinimumFlyingCopterId(void){
    uint8_t min_id = 11;
    for(int i = 1; i < MAX_ADDRESS; i++){
        if (isCopterFlying(i) && copters[i].id < min_id){
            min_id = copters[i].id;
        }
    }

    return min_id;
}

bool isAnyOtherCopterExecutingTrajectory(void){
    for(int i = 1; i < MAX_ADDRESS; i++){
        bool trajectory_condition = copters[i].state == STATE_GOING_TO_TRAJECTORY_START ||
                                    copters[i].state == STATE_EXECUTING_TRAJECTORY;

        if (isCopterFlying(i) && trajectory_condition){
            return true;
        }
    }
    return false;
}

bool appTerminationStillBeingSent(void){
    for(int i = 1; i < MAX_ADDRESS; i++){
        if (isCopterIdActive(i) && copters[i].terminateApp){
            return true;
        }
    }

    return false;
}

bool needExtraCopters(void) {
    uint8_t flying_copters = 0;
    
    for (int i = 1; i < MAX_ADDRESS; i++) {
        if (isCopterFlying(i)) {
            flying_copters ++;
        }
    }

    return flying_copters < DESIRED_FLYING_COPTERS;
}

bool needLessCopters(void){
    uint8_t flying_copters = 0;
    
    for (int i = 1; i < MAX_ADDRESS; i++) {
        uint8_t state = copters[i].state;
        bool isFlying = state > STATE_TAKING_OFF && state < STATE_GOING_TO_PAD;
        if (isCopterIdActive(i) && isFlying){
            flying_copters ++;
        }
    }

    if (selfIsFlying()){
        flying_copters ++;
    }

    return flying_copters > DESIRED_FLYING_COPTERS;
}
