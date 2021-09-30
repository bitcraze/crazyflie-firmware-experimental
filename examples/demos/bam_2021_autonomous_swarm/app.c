
// Define to build the sniffer flavour of the firmware, disable to build for flying members
#include "FreeRTOS.h"
#include "timers.h"
#include "pilot.h"
#include "tower.h"
#include "sniffer.h"

#define DEBUG_MODULE "APP"
#include "debug.h"


static bool isInit = false;



#if BUILD_SNIFFER
#warning "Building sniffer"
void appMain() {
  if (isInit) {
    return;
  }

  DEBUG_PRINT("This is a demo app for an autonomous swarm - sniffer\n");
  initSniffer();

  isInit = true;
}


#else
#warning "Building flyer"

static xTimerHandle pilotTimer;
static xTimerHandle towerTimer;

void appMain() {
  if (isInit) {
    return;
  }

  DEBUG_PRINT("This is a demo app for an autonomous swarm - flyer\n");

  initPilot();
  initTower();

  pilotTimer = xTimerCreate("PilotTimer", M2T(21), pdTRUE, NULL, pilotTimerCb);
  xTimerStart(pilotTimer, 100);

  towerTimer = xTimerCreate("TowerTimer", M2T(10), pdTRUE, NULL, towerTimerCb);
  xTimerStart(towerTimer, 100);

  isInit = true;
}
#endif
