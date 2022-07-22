#pragma once
#define TAKE_OFF_HEIGHT 1.0f

#define LED_ESTIMATOR_STUCK        LED_GREEN_R
#define LED_CRASH                  LED_GREEN_R

#define WP_THRESHOLD              0.1f  //distance threshold to consider a waypoint reached
#define WP_VEL_THRESHOLD          0.3f //velocity threshold to consider a waypoint reached

#define  COLLISION_AVOIDANCE_ELLIPSOID_XY_RADIUS 0.4f //radius of the ellipsoid used to avoid collisions
#define COLLISION_AVOIDANCE_HORIZON 1.5f //horizon of the ellipsoid used to avoid collisions
#define COLLISION_AVOIDANCE_MAX_VELOCITY 0.3f //maximum velocity to avoid collisions
// P2P Interface
#define MAX_ADDRESS 10 //all copter addresses must be between 0 and max(MAX_ADDRESS,9)

#define BROADCAST_FEQUENCY_HZ 15
#define CALC_NEXT_FEQUENCY_HZ 3
#define ALIVE_TIMEOUT_MS 1000 //ms after not receiving data from a copter ,it is considered dead

// position lock settings
#define LOCK_LENGTH 50
#define LOCK_THRESHOLD 0.001f
#define POSITION_LOCK_TIMEOUT 10000

// NEXT DELTA
#define INTER_DIST 0.8f //distance between crazyflies

#define MAXIMUM_NEXT_DELTA 0.2f
#define DELTA_DURATION 6.0f //sec duration to go to next delta

#define HOVERING_TIME 8000 //ms
#define POSITION_UPDATE_TIMEOUT_MS 1500 //ms Timeout to ignore position updates from another copter

//Landing to charging pad
#define MAX_PAD_ERR 0.005
#define LANDING_HEIGHT 0.12f
#define LANDING_DURATION 3 //sec 
#define GO_TO_PAD_HEIGHT 0.5f
#define GO_TO_PAD_DURATION 3.0f //sec duration to go to charging pad

// BOUNDS DEFINITIONS
#define MIN_X_BOUND -2.5f
#define MAX_X_BOUND  2.0f

#define MIN_Y_BOUND -2.0f
#define MAX_Y_BOUND  2.0f

#define MIN_Z_BOUND -0.4f
#define MAX_Z_BOUND  1.5f

//utils
#define BROADCAST_PERIOD_MS (1000 / BROADCAST_FEQUENCY_HZ)
#define CALC_NEXT_PERIOD_MS (1000 / CALC_NEXT_FEQUENCY_HZ)
