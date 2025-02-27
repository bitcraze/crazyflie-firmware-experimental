/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 * pilot.c - App for each copter of the Decentralized Swarm
 */

#include "choose_app.h"
#ifdef BUILD_PILOT_APP

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "radiolink.h"
#include "configblock.h"
#include "log.h"
#include "float.h"
#include "estimator_kalman.h"
#include "ledseq.h"
#include "timers.h"
#include "param.h"
#include "crtp_commander_high_level.h"
#include "pm.h"
#include "supervisor.h"
#include "settings.h"
#include "ds_p2p_interface.h"
#include "positions.h"
#include "common.h"
#include "param_log_interface.h"
#include "movement.h"
#include "led_control.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"

#define RED_LED 0x60, 0x00, 0x00
#define GREEN_LED 0x00, 0x60, 0x00
#define BLUE_LED  0x00, 0x00, 0x60
#define YELLOW_LED  0x60, 0x60, 0x00
#define ORANGE_LED  0x60, 0x30, 0x00
#define WHITE_LED 0x00, 0x00, 0x00, 0x60


static xTimerHandle sendPosTimer;
static xTimerHandle stateTransitionTimer;

static bool isInit = false;

// the state of the copter
enum State state = STATE_IDLE;

// Landing to pad
static uint32_t stabilizeEndTime_ms;
static float landingTimeCheckCharge_ms;

static uint8_t my_id;

Position positions_to_go[] = {
    [0].x = +1,
    [0].y = +1,
    [0].z = TAKE_OFF_HEIGHT,
    [1].x = +1,
    [1].y = -1,
    [1].z = TAKE_OFF_HEIGHT,
    [2].x = -1,
    [2].y = +1,
    [2].z = TAKE_OFF_HEIGHT,
    [3].x = -1,
    [3].y = -1,
    [3].z = TAKE_OFF_HEIGHT,

};

static Position my_pos;

static float previous[3];
static float padX = 0.0;
static float padY = 0.0;
static float padZ = 0.0;

static uint32_t now_ms = 0;
static uint32_t position_lock_start_time_ms = 0;
static uint32_t random_time_for_next_event_ms = 0;

static bool isCrashInitialized = false;

// LEDs Interface
ledseqStep_t seq_flashing_def[] = {
    {true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    {true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    {true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    {true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    {true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    {true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(50)},
    {true, LEDSEQ_WAITMS(50)},
    {false, LEDSEQ_WAITMS(300)},

    {0, LEDSEQ_LOOP},

};

ledseqContext_t seq_estim_stuck = {
    .sequence = seq_flashing_def,
    .led = LED_ESTIMATOR_STUCK,
};

ledseqContext_t seq_crash = {
    .sequence = seq_flashing_def,
    .led = LED_CRASH,
};

uint32_t get_next_random_timeout(uint32_t now_ms)
{
    uint32_t extra = (rand() % (TAKE_OFF_TIME_MAX - TAKE_OFF_TIME_MIN)) + TAKE_OFF_TIME_MIN;
    uint32_t timeout = now_ms + extra;
    DEBUG_PRINT("Next random timeout dt: %lu \n", extra);
    return timeout;
}

// timers
static void broadcastData(xTimerHandle timer)
{
    uint32_t nowMs = T2M(xTaskGetTickCount());

    copter_full_state_t fullState;

    fullState.id = my_id;
    // fullState.counter - set when transmitted
    fullState.state = state;
    fullState.battery_voltage = compressVoltage(getVoltage());
    fullState.timestamp = nowMs;
    fullState.position.x = getX();
    fullState.position.y = getY();
    fullState.position.z = getZ();

    broadcastToPeers(&fullState, nowMs);
}

static void startTakeOffSequence()
{
    // take multiple samples for the pad position
    Position pad_sampler = {0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0; i < NUMBER_OF_PAD_SAMPLES; i++)
    {
        pad_sampler.x += getX();
        pad_sampler.y += getY();
        pad_sampler.z += getZ();
        vTaskDelay(50); // check if it interferes with the other tasks
    }
    MUL_VECTOR_3D_WITH_SCALAR(pad_sampler, 1.0f / NUMBER_OF_PAD_SAMPLES);

    padX = pad_sampler.x;
    padY = pad_sampler.y;
    padZ = pad_sampler.z;
    DEBUG_PRINT("Base position: (%f, %f, %f)\n", (double)padX, (double)padY, (double)padZ);

    DEBUG_PRINT("Taking off...\n");
    crtpCommanderHighLevelTakeoff(padZ + TAKE_OFF_HEIGHT, 1.0);
}

static bool shouldFlySpecialTrajectory()
{
    int random_number = -1;

    if (SPECIAL_TRAJ_PROBABILITY > 0.0f)
    {
        int special_traj_prob_length = (int)(1.0f / SPECIAL_TRAJ_PROBABILITY);
        random_number = rand() % special_traj_prob_length;
        // DEBUG_PRINT("special_traj_prob_length %i\n", special_traj_prob_length);
        // DEBUG_PRINT("Random number: %i\n", random_number);
    }

    uint8_t minimumFlyingCopterId = getMinimumFlyingCopterId();
    bool noOneElseIsFlyingTrajectory = !isAnyOtherCopterExecutingTrajectory();
    return (EXECUTE_TRAJ && (random_number == 0) && (my_id <= minimumFlyingCopterId) && noOneElseIsFlyingTrajectory);
}

static void stateTransition(xTimerHandle timer)
{
    // In the following checks , sequence of checks is important

    my_pos.x = getX();
    my_pos.y = getY();
    my_pos.z = getZ();

    if (supervisorIsTumbled())
    {
        state = STATE_CRASHED;
        ledSetRGB(RED_LED);
    }
    else if (isBatLow() && (state == STATE_HOVERING ||
                            state == STATE_GOING_TO_RANDOM_POINT ||
                            state == STATE_EXECUTING_TRAJECTORY))
    {
        DEBUG_PRINT("Battery low, landing\n");
        gotoChargingPad(padX, padY, padZ);
        state = STATE_GOING_TO_PAD;
        ledSetRGB(RED_LED);
    }

    now_ms = T2M(xTaskGetTickCount());
    switch (state)
    {
    case STATE_IDLE:
        DEBUG_PRINT("Let's go! Waiting for position lock...\n");
        resetLockData();
        position_lock_start_time_ms = now_ms;
        state = STATE_WAIT_FOR_POSITION_LOCK;
        ledSetRGB(ORANGE_LED);
        break;
    case STATE_WAIT_FOR_POSITION_LOCK:
        ledSetRGB(ORANGE_LED);
        if (hasLock())
        {
            DEBUG_PRINT("Position lock acquired, ready for take off..\n");
            state = STATE_WAIT_FOR_TAKE_OFF;
        }
        break;
    case STATE_WAIT_FOR_TAKE_OFF: // This is the main state when not flying
        if (!chargedForTakeoff())
        {
            ledSetRGB(RED_LED);
            // do nothing, wait for the battery to be charged
        }
        else if (needMoreTakeoffQueuedCopters(state))
        {
            DEBUG_PRINT("More copters needed, entering queue...\n");
            state = STATE_QUEUED_FOR_TAKE_OFF;
            ledSetRGB(ORANGE_LED);
        }
        break;
    case STATE_QUEUED_FOR_TAKE_OFF:
        ledSetRGB(ORANGE_LED);
        if (!chargedForTakeoff())
        {
            state = STATE_WAIT_FOR_TAKE_OFF;
            ledSetRGB(RED_LED);
        }
        else if (needLessTakeoffQueuedCopters(state))
        {
            DEBUG_PRINT("Too many copters in queue, leaving queue...\n");
            state = STATE_WAIT_FOR_TAKE_OFF;
            ledSetRGB(RED_LED);
        }
        else if (needMoreCopters(state))
        {
            DEBUG_PRINT("More copters needed, preparing for take off...\n");
            if (supervisorRequestArming(true))
            {
                random_time_for_next_event_ms = get_next_random_timeout(now_ms);
                state = STATE_PREPARING_FOR_TAKE_OFF;
                ledSetRGB(ORANGE_LED);
            }
        }
        break;
    case STATE_PREPARING_FOR_TAKE_OFF:
        ledSetRGB(ORANGE_LED);
        supervisorRequestArming(true) // since copters flying above can delay take-off a lot, make sure we remain armed
        if (!needMoreCopters(state))
        {
            DEBUG_PRINT("Don't need more copters after all, going back to wait state\n");
            if (supervisorRequestArming(false))
            {
                state = STATE_WAIT_FOR_TAKE_OFF;
                ledSetRGB(RED_LED);
            }
        }
        else if (now_ms > random_time_for_next_event_ms && noCopterFlyingAbove())
        {
            DEBUG_PRINT("Taking off...\n");
            startTakeOffSequence();
            state = STATE_TAKING_OFF;
            ledSetRGB(GREEN_LED);
        }
        break;
    case STATE_TAKING_OFF:
        ledSetRGB(GREEN_LED);
        if (crtpCommanderHighLevelIsTrajectoryFinished())
        {
            DEBUG_PRINT("Hovering, waiting for command to start\n");
            enableCollisionAvoidance();
            state = STATE_HOVERING;
        }
        break;
    case STATE_HOVERING:
        ledSetColorFromXYZ(getX(), getY(), getZ());
        if (needMoreLandingQueuedCopters(state))
        {
            DEBUG_PRINT("More copters than desired are flying while hovering, need to land\n");
            random_time_for_next_event_ms = get_next_random_timeout(now_ms);
            state = STATE_PREPARING_FOR_LAND;
        }
        else
        {
            if (shouldFlySpecialTrajectory())
            {
                DEBUG_PRINT("Special trajectory\n");
                gotoNextWaypoint(CENTER_X_BOX, CENTER_Y_BOX, SPECIAL_TRAJ_START_HEIGHT, NO_YAW, DELTA_DURATION);
                state = STATE_GOING_TO_TRAJECTORY_START;
            }
            else
            {
                PositionWithYaw new_pos = RANDOMIZATION_METHOD(&my_pos);
                DEBUG_PRINT("Normal new waypoint (%.2f, %.2f, %.2f)\n", (double)new_pos.x, (double)new_pos.y, (double)new_pos.z);
                gotoNextWaypoint(new_pos.x, new_pos.y, new_pos.z, new_pos.yaw, DELTA_DURATION);
                state = STATE_GOING_TO_RANDOM_POINT;
            }
        }
        break;
    case STATE_GOING_TO_TRAJECTORY_START:
        ledSetColorFromXYZ(getX(), getY(), getZ());
        if (reachedNextWaypoint(my_pos))
        {
            DEBUG_PRINT("Reached trajectory start\n");
            startTrajectory(my_pos);
            disableCollisionAvoidance();
            state = STATE_EXECUTING_TRAJECTORY;
        }
        break;
    case STATE_EXECUTING_TRAJECTORY:
        ledSetColorFromXYZ(BLUE_LED);
        if (crtpCommanderHighLevelIsTrajectoryFinished())
        {
            DEBUG_PRINT("Finished trajectory execution\n");
            enableCollisionAvoidance();
            state = STATE_HOVERING;
        }
        break;
    case STATE_GOING_TO_RANDOM_POINT:
        ledSetColorFromXYZ(getX(), getY(), getZ());
        if (reachedNextWaypoint(my_pos))
        {
            DEBUG_PRINT("Reached next waypoint\n");
            state = STATE_HOVERING;
        }
        break;
    case STATE_PREPARING_FOR_LAND:
        ledSetColorFromXYZ(getX(), getY(), getZ());
        if (needLessLandingQueuedCopters(state))
        { // another copter landed , no need to land after all
            DEBUG_PRINT("Another copter landed, no need to land finally\n");
            state = STATE_HOVERING;
        }
        else if (now_ms > random_time_for_next_event_ms && !isAnyOtherCopterExecutingTrajectory())
        {
            DEBUG_PRINT("Going to pad...\n");
            gotoChargingPad(padX, padY, padZ);
            state = STATE_GOING_TO_PAD;
        }
        break;
    case STATE_GOING_TO_PAD:
        ledSetColorFromXYZ(getX(), getY(), getZ());
        if (reachedNextWaypoint(my_pos))
        {
            DEBUG_PRINT("Over pad,starting lowering\n");
            disableCollisionAvoidance();
            crtpCommanderHighLevelLand(padZ, LANDING_DURATION);
            state = STATE_LANDING;
        }
        break;
    case STATE_LANDING:
        ledSetRGB(RED_LED);
        if (crtpCommanderHighLevelIsTrajectoryFinished())
        {
            if (outOfBounds(my_pos))
            {
                DEBUG_PRINT("Landed because of out of bounds, going to crashed state \n");
                state = STATE_CRASHED;
            }
            else
            {
                DEBUG_PRINT("Landed. Feed me!\n");
                crtpCommanderHighLevelStop();
                landingTimeCheckCharge_ms = now_ms + 4000;
                state = STATE_CHECK_CHARGING;
            }
        }
        break;
    case STATE_CHECK_CHARGING:
        ledSetRGB(RED_LED);
        if (now_ms > landingTimeCheckCharge_ms)
        {
            DEBUG_PRINT("isCharging: %d\n", isCharging());
            if (isCharging())
            {
                if (supervisorRequestArming(false))
                {
                    state = STATE_WAIT_FOR_TAKE_OFF;
                }
            }
            else if (noCopterFlyingAbove())
            {
                DEBUG_PRINT("Not charging. Try to reposition on pad.\n");
                crtpCommanderHighLevelTakeoff(padZ + (TAKE_OFF_HEIGHT/2), 1.0);
                state = STATE_REPOSITION_ON_PAD;
            }
        }
        break;
    case STATE_REPOSITION_ON_PAD:
        ledSetRGB(RED_LED);
        if (crtpCommanderHighLevelIsTrajectoryFinished())
        {
            DEBUG_PRINT("Over pad, stabilizing position\n");
            gotoChargingPad(padX, padY, padZ);
            stabilizeEndTime_ms = now_ms + STABILIZE_TIMEOUT;
            state = STATE_GOING_TO_PAD;
        }
        break;
    case STATE_CRASHED:
        ledSetRGB(RED_LED);
        if (!isCrashInitialized)
        {
            crtpCommanderHighLevelStop();
            DEBUG_PRINT("Crashed, running crash sequence\n");
            ledseqRun(&seq_crash);
            isCrashInitialized = true;
            // maybe need to disarm here
        }
        break;

    default:
        break;
    }
}

void appMain()
{
    if (isInit)
    {
        return;
    }

    uint64_t address = configblockGetRadioAddress();
    my_id = (uint8_t)((address) & 0x00000000ff);

    DEBUG_PRINT("Waiting for activation ...\n");
    // Get log and param ids
    initParamLogInterface();

    ledseqRegisterSequence(&seq_estim_stuck);
    ledseqRegisterSequence(&seq_crash);

    initP2P();
    initOtherStates();
    ledControlInit();
    ledSetRGBW(WHITE_LED);

    srand(my_id); // provide a unique seed for the random number generator

    initCollisionAvoidance();
    enableHighlevelCommander();
    defineTrajectory();

    previous[0] = 0.0f;
    previous[1] = 0.0f;
    previous[2] = 0.0f;

    sendPosTimer = xTimerCreate("SendPosTimer", M2T(BROADCAST_PERIOD_MS), pdTRUE, NULL, broadcastData);
    xTimerStart(sendPosTimer, 20);

    stateTransitionTimer = xTimerCreate("AppTimer", M2T(CALC_NEXT_PERIOD_MS), pdTRUE, NULL, stateTransition);
    xTimerStart(stateTransitionTimer, 20);

    isInit = true;
}

LOG_GROUP_START(app)
LOG_ADD(LOG_UINT8, state, &state)
LOG_GROUP_STOP(app)

#endif // BUILD_PILOT_APP
