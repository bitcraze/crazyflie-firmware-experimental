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

// Returns the current grasp attempt score (0.0 to 100.0). >0 means drone is in wand range.
float wandGetAttemptScore(void);

// Returns the current wand target in world coordinates
void wandGetSetpoint(float *x, float *y, float *z);

// Enable or disable wand signal processing. When disabled, wand is ignored and score is reset.
void wandSetEnabled(bool enabled);

// Force-clear the grasped flag and score (use when yielding the grasp to another drone).
void wandForceRelease(void);
