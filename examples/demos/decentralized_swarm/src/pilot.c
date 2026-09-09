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

// Trajectory synchronization state
static uint8_t claimed_trajectory_slot = 0;
static uint32_t claimed_start_time_global = 0;
static uint8_t last_flown_slot = 0;
static uint32_t last_trajectory_end_time = 0;
static uint32_t claiming_start_time_ms = 0;

// Trajectory probability boost parameter
static float trajectory_join_probability_boost = 0.60f;

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
    
    // Trajectory synchronization data
    fullState.trajectory_slot = claimed_trajectory_slot;
    fullState.trajectory_start_time_global = claimed_start_time_global;

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

static bool shouldAttemptTrajectory()
{
    if (!EXECUTE_TRAJ) {
        return false;
    }

    // Don't attempt if already claimed a slot (Bug #17 fix)
    if (claimed_trajectory_slot != 0) {
        return false;
    }

    // Base probability: 10% (SPECIAL_TRAJ_PROBABILITY)
    // If someone else flying: add boost (default 20% → 30% total)
    float probability = SPECIAL_TRAJ_PROBABILITY;
    if (isAnyoneExecutingTrajectory()) {
        probability += trajectory_join_probability_boost;
    }

    // Check probability
    if (probability >= 1.0f) {
        // 100% probability, always attempt
    } else {
        int random_threshold = (int)(1.0f / probability);
        if (random_threshold < 1) random_threshold = 1;  // Safety check (Bug #28 fix)
        int random_number = rand() % random_threshold;

        if (random_number != 0) {
            return false;  // Probability check failed
        }
    }

    // Try to find available slot (slot system handles coordination)
    uint8_t slot;
    uint32_t start_time;
    if (findAvailableSlot(getGlobalTime(), &slot, &start_time)) {
        claimed_trajectory_slot = slot;
        claimed_start_time_global = start_time;
        DEBUG_PRINT("Claiming slot %d at global time %lu\n", slot, start_time);
        return true;
    }

    return false;  // No available slots
}

static void stateTransition(xTimerHandle timer)
{
    // In the following checks , sequence of checks is important

    my_pos.x = getX();
    my_pos.y = getY();
    my_pos.z = getZ();

    if (supervisorIsCrashed())
    {
        state = STATE_CRASHED;
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
        supervisorRequestArming(true); // since copters flying above can delay take-off a lot, make sure we remain armed
        if (!needMoreCopters(state))
        {
            DEBUG_PRINT("Don't need more copters after all, going back to wait state\n");
            if (supervisorRequestArming(false))
            {
                state = STATE_WAIT_FOR_TAKE_OFF;
                ledSetRGB(RED_LED);
            }
        }
        else if (now_ms > random_time_for_next_event_ms && noCopterFlyingAbove(my_pos))
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
            // Release any claimed trajectory slot (Bug #26 fix)
            if (claimed_trajectory_slot != 0) {
                DEBUG_PRINT("Releasing claimed slot %d due to landing requirement\n", claimed_trajectory_slot);
                claimed_trajectory_slot = 0;
                claimed_start_time_global = 0;
            }
            random_time_for_next_event_ms = get_next_random_timeout(now_ms);
            state = STATE_PREPARING_FOR_LAND;
        }
        else
        {
            if (shouldAttemptTrajectory())
            {
                DEBUG_PRINT("Attempting trajectory, claiming slot %d at global time %lu (start: %lu)\n", 
                           claimed_trajectory_slot, getGlobalTime(), claimed_start_time_global);
                claiming_start_time_ms = now_ms;
                state = STATE_CLAIMING_TRAJECTORY_SLOT;
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
    case STATE_CLAIMING_TRAJECTORY_SLOT:
        ledSetColorFromXYZ(getX(), getY(), getZ());

        // Wait for gossip propagation (500ms = 5 broadcast cycles at 10 Hz)
        // Balances responsiveness with robustness to packet loss
        if (now_ms - claiming_start_time_ms > 500)
        {
            // Check for conflicts (another drone claimed same slot/time)
            bool has_conflict = false;
            uint8_t conflicting_id = 0;
            
            for (uint8_t i = 1; i < MAX_ADDRESS; i++) {
                if (i == my_id || !isAlive(i)) {
                    continue;
                }
                
                // Check if same slot claimed
                if (copters[i].trajectory_slot == claimed_trajectory_slot) {
                    // Same slot: times must be at least 0.9x trajectory duration apart
                    // (allows next cycle to start before previous fully completes)
                    extern float getTrajectoryDuration();
                    uint32_t min_separation_ms = (uint32_t)(getTrajectoryDuration() * 900.0f);  // 0.9x in ms
                    int32_t time_diff = (int32_t)(copters[i].trajectory_start_time_global - claimed_start_time_global);

                    if (abs(time_diff) < (int32_t)min_separation_ms) {
                        has_conflict = true;
                        conflicting_id = i;
                        DEBUG_PRINT("Slot conflict: ID %d also claimed slot %d (time diff: %ld ms < min %lu ms)\n",
                                   i, claimed_trajectory_slot, time_diff, min_separation_ms);
                        break;
                    }
                }
            }
            
            if (has_conflict) {
                if (my_id < conflicting_id) {
                    // We win, proceed to waiting position
                    DEBUG_PRINT("Slot conflict with ID %d, we win (ID %d < %d)\n", conflicting_id, my_id, conflicting_id);
                    
                    // Move to our waiting position
                    float wait_x, wait_y, wait_z;
                    getWaitingPosition(claimed_trajectory_slot, getMaxSimultaneousTrajectories(), 
                                      &wait_x, &wait_y, &wait_z);
                    DEBUG_PRINT("Moving to waiting position (%.2f, %.2f, %.2f)\n", 
                               (double)wait_x, (double)wait_y, (double)wait_z);
                    gotoNextWaypoint(wait_x, wait_y, wait_z, NO_YAW, DELTA_DURATION);
                    state = STATE_WAITING_FOR_TRAJECTORY_START;
                } else {
                    // We lose, back off
                    DEBUG_PRINT("Slot conflict with ID %d, we lose (ID %d > %d), backing off\n", 
                               conflicting_id, my_id, conflicting_id);
                    claimed_trajectory_slot = 0;
                    claimed_start_time_global = 0;
                    state = STATE_HOVERING;
                }
            } else {
                // No conflict, move to waiting position
                DEBUG_PRINT("No slot conflict, moving to waiting position\n");
                
                float wait_x, wait_y, wait_z;
                getWaitingPosition(claimed_trajectory_slot, getMaxSimultaneousTrajectories(), 
                                  &wait_x, &wait_y, &wait_z);
                DEBUG_PRINT("Moving to waiting position (%.2f, %.2f, %.2f)\n", 
                           (double)wait_x, (double)wait_y, (double)wait_z);
                gotoNextWaypoint(wait_x, wait_y, wait_z, NO_YAW, DELTA_DURATION);
                state = STATE_WAITING_FOR_TRAJECTORY_START;
            }
        }
        break;
    case STATE_WAITING_FOR_TRAJECTORY_START:
        ledSetColorFromXYZ(getX(), getY(), getZ());
        {
            uint32_t global_now = getGlobalTime();
            uint32_t time_to_reach_start = 3000;  // 3s to reach trajectory start from waiting position
            int32_t time_until_start = (int32_t)(claimed_start_time_global - global_now);

            // Moving toward waiting position, but check timing regardless of position
            // Check if it's time to move to trajectory start
            if (global_now >= claimed_start_time_global - time_to_reach_start) {
                DEBUG_PRINT("Time to start (T-%ld ms), moving to trajectory start for slot %d\n",
                           time_until_start, claimed_trajectory_slot);
                gotoNextWaypoint(CENTER_X_BOX, CENTER_Y_BOX, SPECIAL_TRAJ_START_HEIGHT,
                               NO_YAW, DELTA_DURATION);
                state = STATE_GOING_TO_TRAJECTORY_START;
            } else if (time_until_start % 5000 < 100) {
                // Debug: print every 5 seconds while waiting
                DEBUG_PRINT("Waiting at position, T-%ld ms until start\n", time_until_start);
            }
        }
        break;
    case STATE_GOING_TO_TRAJECTORY_START:
        ledSetColorFromXYZ(getX(), getY(), getZ());
        {
            uint32_t global_now = getGlobalTime();
            bool at_position = reachedNextWaypoint(my_pos);  // Includes timeout fallback

            // Must reach position AND time must have arrived
            if (at_position && global_now >= claimed_start_time_global) {
                DEBUG_PRINT("Starting trajectory at slot %d (global time: %lu)\n",
                           claimed_trajectory_slot, global_now);
                startTrajectory(my_pos);
                disableCollisionAvoidance();
                state = STATE_EXECUTING_TRAJECTORY;
            } else {
                // Check for problems
                int32_t wait_time = claimed_start_time_global - global_now;
                if (wait_time > 5000) {
                    // Something wrong, abort
                    DEBUG_PRINT("Warning: Start time too far in future (%ld ms), aborting\n", wait_time);
                    claimed_trajectory_slot = 0;
                    enableCollisionAvoidance();
                    state = STATE_HOVERING;
                } else if (!at_position && global_now >= claimed_start_time_global) {
                    // Time arrived but not at position - this shouldn't happen with 3s buffer
                    DEBUG_PRINT("Warning: Start time reached but not at position, aborting slot %d\n",
                               claimed_trajectory_slot);
                    claimed_trajectory_slot = 0;
                    enableCollisionAvoidance();
                    state = STATE_HOVERING;
                }
            }
        }
        break;
    case STATE_EXECUTING_TRAJECTORY:
        ledSetColorFromXYZ(BLUE_LED);
        if (crtpCommanderHighLevelIsTrajectoryFinished())
        {
            DEBUG_PRINT("Finished trajectory execution in slot %d\n", claimed_trajectory_slot);
            
            // Record for immediate re-flight detection
            last_flown_slot = claimed_trajectory_slot;
            last_trajectory_end_time = getGlobalTime();
            
            // Free the slot
            claimed_trajectory_slot = 0;
            claimed_start_time_global = 0;
            
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
        else if (now_ms > random_time_for_next_event_ms)
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
            // if (outOfBounds(my_pos))
            // {
            //     DEBUG_PRINT("Landed because of out of bounds, going to crashed state \n");
            //     state = STATE_CRASHED;
            // }
            // else
            // {
            if (supervisorRequestArming(false)){
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
            else if (noCopterFlyingAbove(my_pos))
            {
                if (supervisorRequestArming(true)){
                vTaskDelay(500);
                DEBUG_PRINT("Not charging. Try to reposition on pad.\n");
                crtpCommanderHighLevelTakeoff(padZ + (LANDING_HEIGHT), 1.0);
                state = STATE_REPOSITION_ON_PAD;
            }
        }
        }
        break;
    case STATE_REPOSITION_ON_PAD:
        if (crtpCommanderHighLevelIsTrajectoryFinished())
        {
            DEBUG_PRINT("Over pad, stabilizing position\n");
            gotoChargingPad(padX, padY, padZ);
            stabilizeEndTime_ms = now_ms + STABILIZE_TIMEOUT;
            state = STATE_GOING_TO_PAD;
        }
        break;
    case STATE_CRASHED:
        if (!(supervisorIsCrashed()))
        {
            DEBUG_PRINT("Crash recovery successful, going to wait for position lock\n");
            resetLockData();
            position_lock_start_time_ms = now_ms;
            state = STATE_WAIT_FOR_POSITION_LOCK;
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
LOG_ADD(LOG_UINT8, trajSlot, &claimed_trajectory_slot)
LOG_ADD(LOG_UINT32, trajStart, &claimed_start_time_global)
LOG_GROUP_STOP(app)

PARAM_GROUP_START(trajSync)
PARAM_ADD(PARAM_FLOAT, joinBoost, &trajectory_join_probability_boost)
PARAM_GROUP_STOP(trajSync)

#endif // BUILD_PILOT_APP
