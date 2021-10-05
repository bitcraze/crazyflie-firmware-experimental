#pragma once


# define LOCK_COUNT 3

typedef struct {
  uint8_t nodeId;
  uint32_t timeRemaining;
} __attribute__((packed)) DeltaTimedLock;

typedef struct {
  DeltaTimedLock lock[LOCK_COUNT];
} __attribute__((packed)) DeltaState;

#define MSG_TYPE_PROPOSITION 1
typedef struct {
  uint8_t msgType; // = 1
  uint8_t nodeId;
  uint32_t proposalNr;
} __attribute__((packed)) Proposition;

#define MSG_TYPE_PROMISE 2
typedef struct {
  uint8_t msgType; // = 2
  uint8_t nodeId;
  uint32_t proposalNr;
  uint32_t previousProposalNr;
  uint8_t propositionAccepted;
  DeltaState currentState;
} __attribute__((packed)) Promise;

#define MSG_TYPE_STATE_UPDATE_REQUEST 3
typedef struct {
  uint8_t msgType; // = 3
  uint8_t nodeId;
  uint32_t proposalNr;
  DeltaState newState;
} __attribute__((packed)) StateUpdateRequest;

#define MSG_TYPE_STATE_UPDATE_ACCEPT 4
typedef struct {
  uint8_t msgType; // = 4
  uint8_t nodeId;
  uint32_t proposalNr;
  uint8_t updateAccepted;
  DeltaState newState;
} __attribute__((packed)) StateUpdateAccept;
