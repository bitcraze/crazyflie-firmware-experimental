#pragma once

#include <math.h>
#include "FreeRTOS.h"

#include "param.h"
#include "log.h"
#include "pm.h"
#include "settings.h"
#include "debug.h"

float getX();
float getY();
float getZ();
float getVx(); 
float getVy(); 
float getVz(); 
float getVelMagnitude();
float getVarPX(); 
float getVarPY(); 
float getVarPZ(); 
bool isBatLow(); 
float getVoltage();
bool isCharging(); 
bool isLighthouseAvailable(); 
void enableHighlevelCommander(); 
void enableCollisionAvoidance(); 

// initialize logs and params
void initCollisionAvoidance();
void initLogIds();

void initParamLogInterface();

void setTerminateTrajectoryAndLand(bool value);
void setTakeOffWhenReady(bool value);
void setTerminateApp(bool value);

bool getTerminateTrajectoryAndLand();
bool getTakeOffWhenReady();
bool getTerminateApp();
