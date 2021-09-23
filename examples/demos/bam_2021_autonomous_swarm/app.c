
// Define to build the sniffer flavour of the firmware, disable to build for flying members
#define BUILD_SNIFFER 0

#include "FreeRTOS.h"
#include "timers.h"
#include "pilot.h"
#include "tower.h"
#include "sniffer.h"

#define DEBUG_MODULE "APP"
#include "debug.h"


static bool isInit = false;



#if BUILD_SNIFFER
void appMain() {
  if (isInit) {
    return;
  }

  DEBUG_PRINT("This is a demo app for an autonomous swarm - sniffer\n");
  initSniffer();

  isInit = true;
}


#else

static xTimerHandle pilotTimer;
static xTimerHandle towerTimer;

void appMain() {
  if (isInit) {
    return;
  }

  DEBUG_PRINT("This is a demo app for an autonomous swarm - flyer\n");

  initPilot();

  pilotTimer = xTimerCreate("PilotTimer", M2T(20), pdTRUE, NULL, pilotTimerCb);
  xTimerStart(pilotTimer, 20);

  towerTimer = xTimerCreate("TowerTimer", M2T(1000), pdTRUE, NULL, towerTimerCb);
  xTimerStart(towerTimer, 20);

  isInit = true;
}
#endif
