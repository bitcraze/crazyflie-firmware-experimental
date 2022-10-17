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

#include "FreeRTOS.h"
#include "task.h"

#include "ds_p2p_interface.h"
#include "param_log_interface.h"


extern enum  State state;

// State of peers
copter_full_state_t copters[MAX_ADDRESS];

// Higher level control
static uint8_t desiredFlyingCopters = INITIAL_DESIRED_FLYING_COPTERS;
static bool isControlDataSetYet = false;
static int32_t controlDataTimeMs = 0;  // Valid if isControlDataSetYet == true

static uint8_t counter = 0;

uint8_t getCopterState(uint8_t copter_id){
    return copters[copter_id].state;
}


void p2pcallbackHandler(P2PPacket *p) {
    static copter_message_t rxMessage;

    if (p->port != 0x00){
        return;
    }

    uint32_t nowMs = T2M(xTaskGetTickCount());

    memcpy(&rxMessage, p->data, sizeof(rxMessage));

    uint8_t received_id = rxMessage.fullState.id;
    memcpy(&copters[received_id], &rxMessage.fullState, sizeof(copter_full_state_t));

    if (rxMessage.isControlDataValid) {
        int32_t newControlDataTimeMs = nowMs - rxMessage.ageOfControlDataMs;
        if ( ! isControlDataSetYet || newControlDataTimeMs > controlDataTimeMs) {
            controlDataTimeMs = newControlDataTimeMs;
            desiredFlyingCopters = rxMessage.desiredFlyingCopters;
            isControlDataSetYet = true;
        }
    }

    if (0 == desiredFlyingCopters) {
        // Request to terminate
        if (!getTerminateApp() &&
            state != STATE_SNIFFING &&
            state != STATE_CRASHED &&
            state != STATE_WAIT_FOR_POSITION_LOCK){
            setTerminateApp(true);
        }
    }

    // If not a message from the sniffer, send the position to the peer localization system to handle collision avoidance
    if (received_id > 0) {
        positionMeasurement_t pos_measurement;
        memcpy(&pos_measurement.pos, &rxMessage.fullState.position, sizeof(Position));

        // DEBUG_PRINT("===================================================\n");
        // DEBUG_PRINT("[RSSI: -%d dBm] Message from CF nr. %d  with counter: %d --> (%.2f , %.2f , %.2f)\n", rssi, received_id, counter,(double)pos_received.x,(double)pos_received.y,(double)pos_received.z);

        pos_measurement.source =  MeasurementSourceLighthouse;
        pos_measurement.stdDev = 0.01f;

        peerLocalizationTellPosition(received_id, &pos_measurement);
    }
}

void broadcastToPeers(const copter_full_state_t* state, const uint32_t nowMs) {
    static P2PPacket packet;
    static copter_message_t txMessage;

    memcpy(&txMessage.fullState, state, sizeof(txMessage.fullState));
    txMessage.fullState.counter = counter;

    txMessage.isControlDataValid = isControlDataSetYet;
    if (isControlDataSetYet) {
        txMessage.desiredFlyingCopters = desiredFlyingCopters;
        txMessage.ageOfControlDataMs = nowMs - controlDataTimeMs;
    }

    packet.port = 0;
    memcpy(packet.data, &txMessage, sizeof(txMessage));

    packet.size = sizeof(txMessage);
    radiolinkSendP2PPacketBroadcast(&packet);

    counter += 1;
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
        if (isCopterIdActive(i)  && copters[i].state != STATE_TAKING_OFF){
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

bool needExtraCopters(void) {
    uint8_t flying_copters = 0;

    for (int i = 1; i < MAX_ADDRESS; i++) {
        if (isCopterFlying(i)) {
            flying_copters ++;
        }
    }

    return flying_copters < desiredFlyingCopters;
}

bool needLessCopters(void){
    uint8_t flying_copters = 0;

    for (int i = 1; i < MAX_ADDRESS; i++) {
        if (isCopterFlying(i)) {
            flying_copters ++;
        }
    }

    if (selfIsFlying()){
        flying_copters ++;
    }

    return flying_copters > desiredFlyingCopters;
}

uint8_t getDesiredFlyingCopters() {
    return desiredFlyingCopters;
}
