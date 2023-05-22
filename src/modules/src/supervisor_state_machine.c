/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <assert.h>
#include "supervisor_state_machine.h"
#include "test_support.h"

// #define DEBUG_ME

#ifdef DEBUG_ME
#define DEBUG_MODULE "SUPST"
#include "debug.h"
#include "cfassert.h"

static const char* const stateNames[] = {
  "Pre-flight checks not passed",
  "Pre-flight checks passed",
  "Ready to fly",
  "Flying",
  "Landed",
  "Reset",
  "Warning, level out",
  "Exception, free fall",
  "Locked",
};
static_assert(sizeof(stateNames) / sizeof(stateNames[0]) == supervisorState_NrOfStates);
#endif


// State transition definitions
static SupervisorStateTransition_t transitionsPreFlChecksNotPassed[] = {
  {
    .newState = supervisorStatePreFlChecksPassed,

    .triggerCombiner = supervisorAlways,

    .blockers = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP,
    .negatedBlockers = SUPERVISOR_CB_NONE,
    .blockerCombiner = supervisorAny,
  }
};

static SupervisorStateTransition_t transitionsPreFlChecksPassed[] = {
  {
    .newState = supervisorStatePreFlChecksNotPassed,

    .triggers = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAny,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStateReadyToFly,

    .triggers = SUPERVISOR_CB_ARMED,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAll,

    .blockers = SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP,
    .negatedBlockers = SUPERVISOR_CB_NONE,
    .blockerCombiner = supervisorAny,
  },
};

static SupervisorStateTransition_t transitionsReadyToFly[] = {
  {
    .newState = supervisorStatePreFlChecksNotPassed,

    .triggers = SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_CHARGER_CONNECTED | SUPERVISOR_CB_EMERGENCY_STOP,
    .negatedTriggers = SUPERVISOR_CB_ARMED,
    .triggerCombiner = supervisorAny,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStateFlying,

    .triggers = SUPERVISOR_CB_IS_FLYING,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAll,

    .blockerCombiner = supervisorNever,
  }
};

static SupervisorStateTransition_t transitionsFlying[] = {
  {
    .newState = supervisorStateExceptFreeFall,

    .triggers = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP,
    .negatedTriggers = SUPERVISOR_CB_ARMED,
    .triggerCombiner = supervisorAny,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStateWarningLevelOut,

    .triggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING,
    .negatedTriggers = SUPERVISOR_CB_NONE,
    .triggerCombiner = supervisorAll,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStateLanded,

    .triggers = SUPERVISOR_CB_NONE,
    .negatedTriggers = SUPERVISOR_CB_IS_FLYING,
    .triggerCombiner = supervisorAll,

    .blockerCombiner = supervisorNever,
  }
};

static SupervisorStateTransition_t transitionsLanded[] = {
  {
    .newState = supervisorStateReset,

    .triggerCombiner = supervisorAlways,

    .blockerCombiner = supervisorNever,
  },
};

static SupervisorStateTransition_t transitionsReset[] = {
  {
    .newState = supervisorStatePreFlChecksNotPassed,

    .triggerCombiner = supervisorAlways,

    .blockerCombiner = supervisorNever,
  },
};

static SupervisorStateTransition_t transitionsWarningLevelOut[] = {
  {
    .newState = supervisorStateExceptFreeFall,

    .triggers = SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT | SUPERVISOR_CB_IS_TUMBLED | SUPERVISOR_CB_EMERGENCY_STOP,
    .negatedTriggers = SUPERVISOR_CB_ARMED,
    .triggerCombiner = supervisorAny,

    .blockerCombiner = supervisorNever,
  },
  {
    .newState = supervisorStateFlying,

    .triggers = SUPERVISOR_CB_NONE,
    .negatedTriggers = SUPERVISOR_CB_COMMANDER_WDT_WARNING | SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT,
    .triggerCombiner = supervisorAll,

    .blockerCombiner = supervisorNever,
  },
};

static SupervisorStateTransition_t transitionsExceptFreeFall[] = {
  {
    .newState = supervisorStateLocked,

    .triggerCombiner = supervisorAlways,

    .blockerCombiner = supervisorNever,
  },
};

static SupervisorStateTransition_t transitionsLocked[] = {
  {
    .newState = supervisorStateLocked,

    .triggerCombiner = supervisorNever,

    .blockerCombiner = supervisorAlways,
  },
};

SupervisorStateTransitionList_t transitionLists[] = {
  {SUPERVISOR_TRANSITION_ENTRY(transitionsPreFlChecksNotPassed)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsPreFlChecksPassed)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsReadyToFly)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsFlying)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsLanded)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsReset)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsWarningLevelOut)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsExceptFreeFall)},
  {SUPERVISOR_TRANSITION_ENTRY(transitionsLocked)},
};
static_assert(sizeof(transitionLists) / sizeof(transitionLists[0]) == supervisorState_NrOfStates);


bool supervisorStateMachineInit() {
  if (sizeof(supervisorState_t) != sizeof(transitionLists)) {
    return false;
  }

  return true;
}

static bool areAllSet(const supervisorConditionBits_t conditions, const supervisorConditionBits_t requirements) {
    return (~conditions & requirements) == 0;
}

static bool isAnySet(const supervisorConditionBits_t conditions, const supervisorConditionBits_t requirements) {
    return (conditions & requirements) != 0;
}

static bool areConditionsMet(const supervisorConditionBits_t conditions, const supervisorConditionBits_t requirements, const supervisorConditionBits_t negRequirements, const SupervisorConditionCombiner_t combiner) {
  bool result = false;

  switch(combiner) {
    case supervisorAll:
      result = areAllSet(conditions, requirements) && !isAnySet(conditions, negRequirements);
      break;
    case supervisorAny:
      result = isAnySet(conditions, requirements) || !areAllSet(conditions, negRequirements);
      break;
    case supervisorAlways:
      result = true;
      break;
    case supervisorNever:
      result = false;
      break;
    default:
      break;
  }

  return result;
}

TESTABLE_STATIC supervisorState_t findTransition(const supervisorState_t currentState, const supervisorConditionBits_t conditions, const SupervisorStateTransitionList_t* transitions) {
  supervisorState_t newState = currentState;
  for (int i = 0; i < transitions->length; i++) {
    const SupervisorStateTransition_t* transitionDef = &transitions->transitionList[i];

    const bool isTriggerMatch = areConditionsMet(conditions, transitionDef->triggers, transitionDef->negatedTriggers, transitionDef->triggerCombiner);
    const bool isBlockerMatch = areConditionsMet(conditions, transitionDef->blockers, transitionDef->negatedBlockers, transitionDef->blockerCombiner);

    const bool isStateTransitionValid = isTriggerMatch && !isBlockerMatch;
    if (isStateTransitionValid) {
      newState = transitionDef->newState;
      break;
    }
  }
  return newState;
}

supervisorState_t supervisorStateUpdate(const supervisorState_t currentState, const supervisorConditionBits_t conditions) {
  #ifdef DEBUG_ME
  ASSERT(currentState < supervisorState_NrOfStates);
  #endif

  const SupervisorStateTransitionList_t* transitions = &transitionLists[currentState];
  const supervisorState_t newState = findTransition(currentState, conditions, transitions);

  #ifdef DEBUG_ME
  ASSERT(currentState < supervisorState_NrOfStates);
  ASSERT(newState < supervisorState_NrOfStates);

  if (newState != currentState) {
    DEBUG_PRINT("Enter %s -> %s\n", stateNames[currentState], stateNames[newState]);
  }
  #endif

  return newState;
}
