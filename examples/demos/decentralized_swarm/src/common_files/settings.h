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

// Flight Settings
#define TAKE_OFF_HEIGHT 1.0f
#define DESIRED_FLYING_COPTERS 3
#define CHARGED_FOR_TAKEOFF_VOLTAGE 4.0f

// RANDOM POSITIONS ON CIRCLE
#define NUMBER_OF_RANDOM_POINTS_ON_CIRCLE  4
#define CIRCLE_RADIUS 1.5f

// Randomizing takeoff times
#define TAKE_OFF_TIME_MAX 3000
#define TAKE_OFF_TIME_MIN 500

#define LED_ESTIMATOR_STUCK        LED_GREEN_R
#define LED_CRASH                  LED_GREEN_R

// Go to next waypoint
#define WP_THRESHOLD              0.1f  //distance threshold to consider a waypoint reached
#define WP_VEL_THRESHOLD          0.4f //velocity threshold to consider a waypoint reached
#define REACHED_WP_TIMEOUT        7000 //ms 
#define COPTER_FLYING_ABOVE_THRESHOLD 0.3f //radius of cylinder that defines if a copter is flying above

// Collision avoidance
#define  COLLISION_AVOIDANCE_ELLIPSOID_XY_RADIUS 0.3f //radius of the ellipsoid used to avoid collisions
#define COLLISION_AVOIDANCE_HORIZON 1.5f //horizon of the ellipsoid used to avoid collisions
#define COLLISION_AVOIDANCE_MAX_VELOCITY 0.3f //maximum velocity to avoid collisions

#define COLLISION_AVOIDANCE_BBOX_MIN_X  -2.0f
#define COLLISION_AVOIDANCE_BBOX_MAX_X   2.0f

#define COLLISION_AVOIDANCE_BBOX_MIN_Y  -2.0f
#define COLLISION_AVOIDANCE_BBOX_MAX_Y   2.0f

#define COLLISION_AVOIDANCE_BBOX_MIN_Z   0.1f
#define COLLISION_AVOIDANCE_BBOX_MAX_Z   2.0f


// P2P Interface
#define MAX_ADDRESS 10 //all copter addresses must be between 0 and max(MAX_ADDRESS,9)

#define BROADCAST_FREQUENCY_HZ 15
#define CALC_NEXT_FREQUENCY_HZ 3
#define COPTER_STATUS_FREQUENCY_HZ 5 

#define SNIFFER_PRINT_FREQUENCY_HZ 5

#define ALIVE_TIMEOUT_MS 1000 //ms after not receiving data from a copter ,it is considered dead

// position lock settings
#define LOCK_LENGTH 50
#define LOCK_THRESHOLD 0.001f
#define POSITION_LOCK_TIMEOUT 10000

// NEXT DELTA
#define INTER_DIST 0.8f //distance between crazyflies

#define MAXIMUM_NEXT_DELTA 0.2f
#define DELTA_DURATION 4.0f //sec duration to go to next delta

#define HOVERING_TIME 18000 //ms
#define POSITION_UPDATE_TIMEOUT_MS 1500 //ms Timeout to ignore position updates from another copter

//Landing to charging pad
#define NUMBER_OF_PAD_SAMPLES 10 //number of samples to take to estimate the landing pad
#define MAX_PAD_ERR 0.01
#define LANDING_HEIGHT 0.13f
#define LANDING_DURATION 2 //sec 
#define GO_TO_PAD_HEIGHT 0.5f
#define GO_TO_PAD_DURATION 3.0f //sec duration to go to charging pad
#define STABILIZE_TIMEOUT 4000 //ms
#define REACHED_CHARGING_PAD_TIMEOUT 10000 //ms

// BOUNDS DEFINITIONS
#define SAFETY_LANDING_DURATION 6 //sec

#define MIN_X_BOUND -2.5f
#define MAX_X_BOUND  2.5f

#define MIN_Y_BOUND -2.5f
#define MAX_Y_BOUND  2.5f

#define MIN_Z_BOUND -0.3f
#define MAX_Z_BOUND  2.0f

//utils
#define BROADCAST_PERIOD_MS (1000 / BROADCAST_FREQUENCY_HZ)
#define CALC_NEXT_PERIOD_MS (1000 / CALC_NEXT_FREQUENCY_HZ)
#define SNIFFER_PRINT_PERIOD_MS (1000 / SNIFFER_PRINT_FREQUENCY_HZ)
#define COPTER_STATUS_PERIOD_MS (1000 / COPTER_STATUS_FREQUENCY_HZ)
