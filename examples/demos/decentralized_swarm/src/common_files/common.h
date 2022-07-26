#pragma once

// States of the state machine 
enum State {
    // Initialization
    STATE_IDLE = 0,
    STATE_WAIT_FOR_POSITION_LOCK,

    STATE_WAIT_FOR_TAKE_OFF,
    STATE_PREPARING_FOR_TAKE_OFF,
    STATE_TAKING_OFF,
    STATE_HOVERING,
    STATE_GOING_TO_RANDOM_POINT,
    STATE_PREPARING_FOR_LAND,
    STATE_GOING_TO_PAD,
    STATE_REGOING_TO_PAD,
    STATE_WAITING_AT_PAD,
    STATE_LANDING,
    STATE_CHECK_CHARGING,
    STATE_REPOSITION_ON_PAD,

    STATE_CRASHED,
};