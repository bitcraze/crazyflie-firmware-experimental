#include <string.h>
#include "FreeRTOS.h"
#include "timers.h"
#include "radiolink.h"
#include "app_channel.h"
#include "protocol.h"
#include "configblock.h"


#include "tower.h"

#define DEBUG_MODULE "TOWER"
#include "debug.h"


static void sendProposition(const uint32_t proposalNr);
static void sendPromise(const uint32_t proposalNr, const uint32_t previousProposalNr, const bool propositionAccepted, const TowerState* currentState);
static void sendStateUpdateRequest(const uint32_t proposalNr, const TowerState* newState);
static void sendStateUpdateAccept(const uint32_t proposalNr, const bool updateAccepted);
static void sendTowerMessage(const uint8_t* msg, const uint8_t size);

static uint8_t nodeId = 0;

void initTower() {
  DEBUG_PRINT("Tower initialization\n");

  uint64_t address = configblockGetRadioAddress();
  nodeId =(uint8_t)((address) & 0x00000000ff);
  DEBUG_PRINT("I have node id %d\n", nodeId);
}

void towerTimerCb(xTimerHandle timer) {
  static uint8_t i = 0;

  switch(i % 4) {
    case 0:
      sendProposition(i);
      break;
    case 1:
      {
        TowerState state;
        state.nodeId1 = 7;
        state.timeRemaining1 = 8;
        state.nodeId2 = 9;
        state.timeRemaining2 = 10;

        sendPromise(i, i - 1, false, &state);
      }
      break;
    case 2:
      {
        TowerState state;
        state.nodeId1 = 5;
        state.timeRemaining1 = 6;
        state.nodeId2 = 7;
        state.timeRemaining2 = 8;

        sendStateUpdateRequest(i, &state);
      }
      break;
    case 3:
      {
        sendStateUpdateAccept(i, true);
      }
      break;
  }

  i++;
}

static void sendProposition(const uint32_t proposalNr) {
  static Proposition msg = {
    .msgType = 1
  };

  msg.nodeId = nodeId;
  msg.proposalNr = proposalNr;

  sendTowerMessage((uint8_t*)&msg, sizeof(msg));
}

static void sendPromise(const uint32_t proposalNr, const uint32_t previousProposalNr, const bool propositionAccepted, const TowerState* currentState) {
  static Promise msg = {
    .msgType = 2
  };

  msg.nodeId = nodeId;
  msg.proposalNr = proposalNr;
  msg.previousProposalNr = previousProposalNr;
  msg.propositionAccepted = propositionAccepted;
  memcpy(&msg.currentState, currentState, sizeof(msg.currentState));

  sendTowerMessage((uint8_t*)&msg, sizeof(msg));
}

static void sendStateUpdateRequest(const uint32_t proposalNr, const TowerState* newState) {
  static StateUpdateRequest msg = {
    .msgType = 3
  };

  msg.nodeId = nodeId;
  msg.proposalNr = proposalNr;
  memcpy(&msg.newState, newState, sizeof(msg.newState));

  sendTowerMessage((uint8_t*)&msg, sizeof(msg));
}

static void sendStateUpdateAccept(const uint32_t proposalNr, const bool updateAccepted) {
  static StateUpdateAccept msg = {
    .msgType = 4
  };

  msg.nodeId = nodeId;
  msg.proposalNr = proposalNr;
  msg.updateAccepted = updateAccepted;

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
