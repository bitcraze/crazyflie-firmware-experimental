#pragma once

void initPilot();
void pilotTimerCb(xTimerHandle timer);

int getFlightCycleTimeMs();
int getFullFlightTimeMs();
bool isPilotReadyForFlight();
void takeOffWithDelay(const uint32_t delayMs);
bool hasPilotLanded();
bool hasCrashed();
