/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * persistant_log.c - Storage for flight logging data.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "storage.h"
#include "configblock.h"
#include "log.h"
#include "param.h"

#define DEBUG_MODULE "APP"
#include "debug.h"

typedef struct {
    uint32_t h;
    uint32_t m;
    uint32_t s;
} humanTime_t;

// Counters
static uint32_t takeOffCount = 0;
static uint32_t landingCount = 0;
static uint32_t flightCount = 0;
static uint32_t persistentFlightCount = 10;

// Times
static uint32_t timeOverflowCounter = 0;
static uint32_t alive_time_ms = 0;

static uint32_t takeOffTime_ms = 0;
static uint32_t lastFlightTime_ms = 0;
static uint32_t totalFlightTime_ms = 0;
static uint32_t persistentTotalFlightTime_ms = 12000;

static uint8_t resetData = 0;
static bool pendingStorageWrite = false;

static const char* PERSISTENT_FLIGHT_COUNT_STORAGE_KEY = "app/totFlight";
static const char* PERSISTENT_FLIGHT_TIME_STORAGE_KEY = "app/totTime";

static void msToHumanTime(const uint32_t time_ms, humanTime_t* humanTime) {
    uint32_t time_s = time_ms / 1000;

    humanTime->h = time_s / (60 * 60);
    time_s -= humanTime->h * 60 * 60;
    humanTime->m = time_s / 60;
    time_s -= humanTime->m * 60;
    humanTime->s = time_s;
}

void storeTotalFlights() {
    if (!pendingStorageWrite) {
        return;
    }
    storageStore(PERSISTENT_FLIGHT_COUNT_STORAGE_KEY, &persistentFlightCount, sizeof(uint32_t));
    storageStore(PERSISTENT_FLIGHT_TIME_STORAGE_KEY, &persistentTotalFlightTime_ms, sizeof(uint32_t));
    humanTime_t totalTimeHt;
    msToHumanTime(persistentTotalFlightTime_ms, &totalTimeHt);
    DEBUG_PRINT("Stored flight time %lu:%lu:%lu, flights %lu\n", totalTimeHt.h, totalTimeHt.m, totalTimeHt.s, persistentFlightCount);
    pendingStorageWrite = false;
}

void getTotalFlightsFromStorage() {
    storageFetch(PERSISTENT_FLIGHT_COUNT_STORAGE_KEY, (void*)&persistentFlightCount, sizeof(uint32_t));
    storageFetch(PERSISTENT_FLIGHT_TIME_STORAGE_KEY, (void*)&persistentTotalFlightTime_ms, sizeof(uint32_t));
    humanTime_t totalTimeHt;
    msToHumanTime(persistentTotalFlightTime_ms, &totalTimeHt);
    DEBUG_PRINT("Total flight time %lu:%lu:%lu\n", totalTimeHt.h, totalTimeHt.m, totalTimeHt.s);
    DEBUG_PRINT("Total flights %lu\n", persistentFlightCount);
}

static void resetDataCallback() {
     persistentFlightCount = 0;
     persistentTotalFlightTime_ms = 0.0f;
     storeTotalFlights();
}

void updateAliveTime() {
    uint32_t previous_now_ms = alive_time_ms;
    alive_time_ms = T2M( xTaskGetTickCount() );
    if (alive_time_ms < previous_now_ms) {
        timeOverflowCounter++;
    }
}

void updateTakeOffTime() {
    takeOffTime_ms = alive_time_ms;
}

void updateFlightTime() {
     landingCount++;
     flightCount++;
     persistentFlightCount++;
     lastFlightTime_ms = alive_time_ms - takeOffTime_ms;
     totalFlightTime_ms += lastFlightTime_ms;
     persistentTotalFlightTime_ms += lastFlightTime_ms;
     pendingStorageWrite = true;
}


LOG_GROUP_START(app_counters)
  LOG_ADD(LOG_UINT32, takeOff, &takeOffCount)
  LOG_ADD(LOG_UINT32, land, &landingCount)
  LOG_ADD(LOG_UINT32, flight, &flightCount)
  LOG_ADD(LOG_UINT32, prsFlight, &persistentFlightCount)
LOG_GROUP_STOP(appcounters)

LOG_GROUP_START(app_time)
  LOG_ADD(LOG_UINT32, lastFlight, &lastFlightTime_ms)
  LOG_ADD(LOG_UINT32, totalFlight, &totalFlightTime_ms)
  LOG_ADD(LOG_UINT32, alive, &alive_time_ms)
  LOG_ADD(LOG_UINT32, overflowCnt, &timeOverflowCounter)
  LOG_ADD(LOG_UINT32, prsFlight, &persistentTotalFlightTime_ms)
LOG_GROUP_STOP(app_time)

PARAM_GROUP_START(app)
  // USER COMMANDS
  PARAM_ADD_WITH_CALLBACK(LOG_UINT8, resetData, &resetData, resetDataCallback)
PARAM_GROUP_STOP(app)
