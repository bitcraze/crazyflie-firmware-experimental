#ifndef P2P_INTERFACE_H
#define P2P_INTERFACE_H

#include <float.h>
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "estimator_kalman.h"

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
#endif // P2P_INTERFACE_H