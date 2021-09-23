#include <string.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "radiolink.h"
#include "app_channel.h"


#include "tower.h"

#define DEBUG_MODULE "TOWER"
#include "debug.h"

void initTower() {
  DEBUG_PRINT("Tower initialization\n");
}

void towerTimerCb(xTimerHandle timer) {
    static P2PPacket pk;
    pk.port = 0;
    pk.size = 11;
    memcpy(pk.data, "Hello World", 11);
    radiolinkSendP2PPacketBroadcast(&pk);
}
