#pragma once

void initPilot();
void pilotTimerCb(xTimerHandle timer);

int getFlightCycleTimeMs();
int getFullFlightTimeMs();
bool isPilotReadyForFlight();
void takeOffAt(const uint32_t takeOffTime);
bool hasPilotLanded();
bool hasCrashed();
void pilotSetActivation(const uint8_t isActive);
