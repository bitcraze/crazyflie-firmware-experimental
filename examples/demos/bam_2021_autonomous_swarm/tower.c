#include <string.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "radiolink.h"
#include "app_channel.h"
#include "protocol.h"


#include "tower.h"

#define DEBUG_MODULE "TOWER"
#include "debug.h"


static void sendProposition(const uint8_t nodeId, const uint32_t proposalNr);
static void sendTowerMessage(const uint8_t* msg, const uint8_t size);

void initTower() {
  DEBUG_PRINT("Tower initialization\n");
}

void towerTimerCb(xTimerHandle timer) {
  sendProposition(2, 17);
}

static void sendProposition(const uint8_t nodeId, const uint32_t proposalNr) {
  static Proposition msg = {
    .msgType = 1
  };

  msg.nodeId = nodeId;
  msg.proposalNr = proposalNr;

  sendTowerMessage((uint8_t*)&msg, sizeof(msg));
}

static void sendTowerMessage(const uint8_t* msg, const uint8_t size) {
  // Not re-entrant safe
  static P2PPacket pk;

  pk.port = 0;
  pk.size = size;
  memcpy(pk.data, msg, size);
  radiolinkSendP2PPacketBroadcast(&pk);
}
