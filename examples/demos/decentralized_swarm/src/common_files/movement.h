#include <float.h>
#include <math.h>

#include "FreeRTOS.h"
#include "positions.h"
#include "crtp_commander_high_level.h"
#include "param_log_interface.h"
#include "sensors.h"
#include "param_log_interface.h"
#include "peer_localization.h"


void gotoNextWaypoint(float x,float y,float z,float duration);

void gotoChargingPad(float x,float y,float z,float duration);

bool reachedNextWaypoint(Position my_pos);

bool outOfBounds(Position my_pos);

void resetLockData();

bool hasLock();

bool chargedForTakeoff();

bool noCopterFlyingAbove();