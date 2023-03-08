/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 Bitcraze AB
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
 *
 * human_tracker.c - Tracker that is carried by a human, will be avoided by the pilots
 */

#include "choose_app.h"
#if (BUILD_TYPE == BUILD_TYPE_HUMAN_TRACKER_APP)
#include "app.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "configblock.h"
#include "ds_p2p_interface.h"
#include "param_log_interface.h"


static xTimerHandle sendPosTimer;
static uint8_t my_id;

static void broadcastData(xTimerHandle timer) {
    uint32_t nowMs = T2M(xTaskGetTickCount());

    copter_full_state_t fullState;

    fullState.id = my_id;
    // fullState.counter - set when transmitted
    fullState.state = STATE_I_AM_A_HUMAN;
    fullState.battery_voltage = 0;
    fullState.timestamp = nowMs;
    fullState.position.x = getX();
    fullState.position.y = getY();
    fullState.position.z = getZ();

    broadcastToPeers(&fullState, nowMs);
}


void appMain()
{
    DEBUG_PRINT("**** Running human tracker\n");

    uint64_t address = configblockGetRadioAddress();
    my_id = (uint8_t)((address) & 0x00000000ff);

    // Get log and param ids
    initParamLogInterface();

    initP2P();

    sendPosTimer = xTimerCreate("SendPosTimer", M2T(BROADCAST_PERIOD_MS), pdTRUE, NULL, broadcastData);
    xTimerStart(sendPosTimer, 20);
}

#endif // BUILD_TYPE_HUMAN_TRACKER_APP
