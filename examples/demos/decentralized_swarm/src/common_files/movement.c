#ifndef  MOVEMENT_H
#define  MOVEMENT_H

#include "movement.h"

static Position next_wp;
static float lockData[LOCK_LENGTH][3];
static uint32_t lockWriteIndex;
static uint32_t timeOfReachingWaypointTimeout;

void gotoNextWaypoint(float x,float y,float z,float duration){
    // This function will move the Crazyflie to the next waypoint via the high level commander.
    // It is used as a function because it works with the reachedNextWaypoint() since  
    // crtpCommanderHighLevelIsTrajectoryFinished() is not working properly with the collision avoidance enabled.
    
    next_wp.x = x;
    next_wp.y = y;
    next_wp.z = z;
    const float yaw = 0.0f;
    const bool relative = false;
    timeOfReachingWaypointTimeout = xTaskGetTickCount() + REACHED_WP_TIMEOUT ;
    crtpCommanderHighLevelGoTo(next_wp.x, next_wp.y, next_wp.z, yaw, duration,relative);
}

bool reachedNextWaypoint(Position my_pos){
    bool close_and_stabilized = DISTANCE3D(my_pos,next_wp) < WP_THRESHOLD  && getVelMagnitude() < WP_VEL_THRESHOLD;
    bool time_out = xTaskGetTickCount() > timeOfReachingWaypointTimeout;
    return close_and_stabilized || time_out;
}

bool outOfBounds(Position my_pos) {
    return my_pos.x > MAX_X_BOUND || my_pos.x < MIN_X_BOUND 
        || my_pos.y > MAX_Y_BOUND || my_pos.y < MIN_Y_BOUND 
        || my_pos.z > MAX_Z_BOUND || my_pos.z < MIN_Z_BOUND;
}


void resetLockData() {
    lockWriteIndex = 0;
    for (uint32_t i = 0; i < LOCK_LENGTH; i++) {
      lockData[i][0] = FLT_MAX;
      lockData[i][1] = FLT_MAX;
      lockData[i][2] = FLT_MAX;
    }
}

bool hasLock() {
  bool result = false;

  // Store current state
  lockData[lockWriteIndex][0] = getVarPX();
  lockData[lockWriteIndex][1] = getVarPY();
  lockData[lockWriteIndex][2] = getVarPZ();

  lockWriteIndex++;
  if (lockWriteIndex >= LOCK_LENGTH) {
    lockWriteIndex = 0;
  }

  // Check if we have a lock
  int count = 0;

  float lXMax = FLT_MIN;
  float lYMax = FLT_MIN;
  float lZMax = FLT_MIN;

  float lXMin = FLT_MAX;
  float lYMin = FLT_MAX;
  float lZMin = FLT_MAX;

  for (int i = 0; i < LOCK_LENGTH; i++) {
    if (lockData[i][0] != FLT_MAX) {
      count++;

      lXMax = fmaxf(lXMax, lockData[i][0]);
      lYMax = fmaxf(lYMax, lockData[i][1]);
      lZMax = fmaxf(lZMax, lockData[i][2]);

      lXMin = fminf(lXMax, lockData[i][0]);
      lYMin = fminf(lYMin, lockData[i][1]);
      lZMin = fminf(lZMin, lockData[i][2]);
    }
  }

  result =
    (count >= LOCK_LENGTH) &&
    ((lXMax - lXMin) < LOCK_THRESHOLD) &&
    ((lYMax - lYMin) < LOCK_THRESHOLD) &&
    ((lZMax - lZMin) < LOCK_THRESHOLD &&
    isLighthouseAvailable() &&  // Make sure we have a deck and the Lighthouses are powered
    sensorsAreCalibrated());

  return result;
}

bool chargedForTakeoff() {
  return getVoltage() > CHARGED_FOR_TAKEOFF_VOLTAGE;
}
#endif // MOVEMENT_H