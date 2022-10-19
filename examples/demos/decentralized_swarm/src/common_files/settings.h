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
 * settings.h - Settings for the decentralized swarm
 *
 */

#pragma once

#define EXECUTE_TRAJ true

// Flight Settings
#define TAKE_OFF_HEIGHT 1.0f
#define INITIAL_DESIRED_FLYING_COPTERS 0
#define CHARGED_FOR_TAKEOFF_VOLTAGE 4.0f

// Randomizing takeoff times
#define TAKE_OFF_TIME_MAX 8000
#define TAKE_OFF_TIME_MIN 500

#define LED_ESTIMATOR_STUCK        LED_GREEN_R
#define LED_CRASH                  LED_GREEN_R

// Go to next waypoint
#define WP_THRESHOLD              0.1f  //distance threshold to consider a waypoint reached
#define WP_VEL_THRESHOLD          0.4f //velocity threshold to consider a waypoint reached
#define REACHED_WP_TIMEOUT        7000 //ms
#define COPTER_FLYING_ABOVE_THRESHOLD 0.3f //radius of cylinder that defines if a copter is flying above


// P2P Interface
#define MAX_ADDRESS 10 //all copter addresses must be between 0 and max(MAX_ADDRESS,9)

#define BROADCAST_FREQUENCY_HZ 10
#define CALC_NEXT_FREQUENCY_HZ 3

#define SNIFFER_PRINT_FREQUENCY_HZ 1

#define ALIVE_TIMEOUT_MS 1000 //ms after not receiving data from a copter ,it is considered dead

// position lock settings
#define LOCK_LENGTH 50
#define LOCK_THRESHOLD 0.001f
#define POSITION_LOCK_TIMEOUT 10000

// NEXT DELTA
#define DELTA_DURATION 4.0f //sec duration to go to next delta

//Landing to charging pad
#define NUMBER_OF_PAD_SAMPLES 10 //number of samples to take to estimate the landing pad
#define MAX_PAD_ERR 0.01
#define LANDING_HEIGHT 0.13f
#define LANDING_DURATION 3 //sec
#define GO_TO_PAD_HEIGHT 0.5f
#define GO_TO_PAD_DURATION 3.0f //sec duration to go to charging pad
#define STABILIZE_TIMEOUT 4000 //ms
#define REACHED_CHARGING_PAD_TIMEOUT 10000 //ms

// BOUNDS DEFINITIONS
#define MIN_X_BOUND -0.69f
#define MAX_X_BOUND  1.10f

#define MIN_Y_BOUND -1.30f
#define MAX_Y_BOUND  1.38f

#define MIN_Z_BOUND -0.3f
#define MAX_Z_BOUND  1.7f

// getRandomPositionOnCircle or getRandomPositionOnBox
// #define RANDOMIZATION_METHOD getRandomPositionOnCircle
#define RANDOMIZATION_METHOD getRandomPositionOnBox

// RANDOM POSITIONS ON CIRCLE
#define NUMBER_OF_RANDOM_POINTS_ON_CIRCLE  6
#define CIRCLE_RADIUS 0.5f

// RANDOM POSITIONS ON SQUARE
// Uses the bounds definition with a margin
#define NUMBER_OF_RANDOM_POINTS_ON_BOX 5
#define RANDOM_BOX_MARGIN 0.20f
#define MIN_BOX_DISTANCE 0.8f   // Minimum distance from current position when randomizing positions

#define MIN_X_BOX (MIN_X_BOUND + RANDOM_BOX_MARGIN)
#define MAX_X_BOX (MAX_X_BOUND - RANDOM_BOX_MARGIN)
#define CENTER_X_BOX ((MIN_X_BOX + MAX_X_BOX) / 2.0f)

#define MIN_Y_BOX (MIN_Y_BOUND + RANDOM_BOX_MARGIN)
#define MAX_Y_BOX (MAX_Y_BOUND - RANDOM_BOX_MARGIN)
#define CENTER_Y_BOX ((MIN_Y_BOX + MAX_Y_BOX) / 2.0f)

// Random trajectory execution
#define SPECIAL_TRAJ_PROBABILITY 0.1f //probability of executing the special trajectory
#define SPECIAL_TRAJ_START_HEIGHT 0.7f

//utils
#define BROADCAST_PERIOD_MS (1000 / BROADCAST_FREQUENCY_HZ)
#define CALC_NEXT_PERIOD_MS (1000 / CALC_NEXT_FREQUENCY_HZ)
#define SNIFFER_PRINT_PERIOD_MS (1000 / SNIFFER_PRINT_FREQUENCY_HZ)

// Collision avoidance
#define COLLISION_AVOIDANCE_ELLIPSOID_XY_RADIUS 0.3f //radius of the ellipsoid used to avoid collisions
#define COLLISION_AVOIDANCE_HORIZON 1.5f //horizon of the ellipsoid used to avoid collisions
#define COLLISION_AVOIDANCE_MAX_VELOCITY 0.3f //maximum velocity to avoid collisions

#define COLLISION_AVOIDANCE_BBOX_MIN_X  MIN_X_BOX
#define COLLISION_AVOIDANCE_BBOX_MAX_X  MAX_X_BOX

#define COLLISION_AVOIDANCE_BBOX_MIN_Y  MIN_Y_BOX
#define COLLISION_AVOIDANCE_BBOX_MAX_Y  MAX_Y_BOX

#define COLLISION_AVOIDANCE_BBOX_MIN_Z  MIN_Z_BOUND
#define COLLISION_AVOIDANCE_BBOX_MAX_Z  MAX_Z_BOUND
