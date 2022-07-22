#ifndef P2P_INTERFACE_H
#define P2P_INTERFACE_H

#include <float.h>
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "estimator_kalman.h"

#include "positions.h"
#include "radiolink.h"
#include "peer_localization.h"
#include "settings.h"

#define MAX_ADDRESS 10

typedef struct packet_struct {
    uint8_t id;
    uint8_t counter;
    uint8_t state;
    uint32_t timestamp;

} copter_t;

uint8_t getCopterState(uint8_t copter_id);

void p2pcallbackHandler(P2PPacket *p);

void initOtherStates();

#endif // P2P_INTERFACE_H