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
 * param_log_interface.h - Parameter log interface
 * 
 */
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
