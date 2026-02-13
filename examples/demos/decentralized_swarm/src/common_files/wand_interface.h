#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "radiolink.h"

void wandInit(void);

// Handle incoming P2P packets (wand uses port 0x01)
void wandHandleP2PPacket(P2PPacket *p);

// Update internal state (timeouts). Pass xTaskGetTickCount().
void wandUpdate(uint32_t nowTicks);

bool wandIsGrasped(void);

// Returns the current wand target in world coordinates
void wandGetSetpoint(float *x, float *y, float *z);
