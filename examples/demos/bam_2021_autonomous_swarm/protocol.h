#pragma once

typedef struct {
  uint8_t nodeId1;
  uint32_t timeRemaining1;
  uint8_t nodeId2;
  uint32_t timeRemaining2;
} __attribute__((packed)) TowerState;

typedef struct {
  uint8_t msgType; // = 1
  uint8_t nodeId;
  uint32_t proposalNr;
} __attribute__((packed)) Proposition;

typedef struct {
  uint8_t msgType; // = 2
  uint8_t nodeId;
  uint32_t proposalNr;
  uint32_t previousProposalNr;
  uint8_t propositionAccepted;
  TowerState currentState;
} __attribute__((packed)) Promise;

typedef struct {
  uint8_t msgType; // = 3
  uint8_t nodeId;
  uint8_t roundNr;
  TowerState newState;
} __attribute__((packed)) StateUpdate;

typedef struct {
  uint8_t msgType; // = 4
  uint8_t nodeId;
  uint8_t roundNr;
  uint8_t updateAccepted;
} __attribute__((packed)) Accepted;
