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
 * p2p_interface.h
 * Peer to peer interface communication.
 */

#ifndef P2P_INTERFACE_H
#define P2P_INTERFACE_H

#include <float.h>
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "estimator_kalman.h"

#include "choose_app.h"
#include "common.h"
#include "positions.h"
#include "radiolink.h"
#include "peer_localization.h"
#include "settings.h"
#include "debug.h"
#include "param_log_interface.h"

/*
    PACKET FORMAT:
    [0]     --> id
    [1]     --> counter
    [2]     --> state
    [3-14]  --> x,y,z
    [15]    --> compressed Voltage    
*/
#define MAX_ADDRESS 10

#define VOLTAGE_MAX 4.2f
#define VOLTAGE_MIN 3.0f

typedef struct packet_struct {
    uint8_t id;
    uint8_t counter;
    uint8_t state;
    uint8_t battery_voltage; //normalized to 0-255 (0-3.3V) 
    bool terminateApp;

    uint32_t timestamp;

} copter_t;

uint8_t getCopterState(uint8_t copter_id);

void p2pcallbackHandler(P2PPacket *p);

void initOtherStates();

bool isAlive(uint8_t copter_id);

uint8_t compressVoltage(float voltage);

float decompressVoltage(uint8_t voltage);

//Returns true if at least one copter has flown (doesn't mean that it is alive)
bool atLeastOneCopterHasFlown(void);

void printOtherCopters(void);

uint8_t otherCoptersActiveNumber(void);

bool isCopterIdActive(uint8_t copter_id);

//Returns true if the copter is active and flying
bool isCopterFlying(uint8_t copter_id);

uint8_t getMinimumFlyingCopterId(void);

bool isAnyOtherCopterExecutingTrajectory(void);

bool appTerminationStillBeingSent(void);

bool selfIsFlying(void);

bool needExtraCopters(void);

bool needLessCopters(void);

#endif // P2P_INTERFACE_H
