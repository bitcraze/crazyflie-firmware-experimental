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
 * movement.h - Movement functions for the pilot
 * 
 */

#ifndef  MOVEMENT_H
#define  MOVEMENT_H

#include <float.h>
#include <math.h>

#include "FreeRTOS.h"
#include "positions.h"
#include "crtp_commander_high_level.h"
#include "param_log_interface.h"
#include "sensors.h"
#include "param_log_interface.h"
#include "peer_localization.h"

// This function will move the Crazyflie to the next waypoint via the high level commander.
// It is used as a function because it works with the reachedNextWaypoint() since  
// crtpCommanderHighLevelIsTrajectoryFinished() is not working properly with the collision avoidance enabled.
void gotoNextWaypoint(float x,float y,float z,float duration);

// This function will move the Crazyflie to the next waypoint via the high level commander.
// It is used as a function because it works with the reachedNextWaypoint() since  
// crtpCommanderHighLevelIsTrajectoryFinished() is not working properly with the collision avoidance enabled.
void gotoChargingPad(float x,float y,float z,float duration);

bool reachedNextWaypoint(Position my_pos);

bool outOfBounds(Position my_pos);

void resetLockData();

bool hasLock();

bool chargedForTakeoff();

bool noCopterFlyingAbove();

#endif // MOVEMENT_H
