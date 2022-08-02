/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 *
 *
 * common.h - Common State for the apps
 * 
 */
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
    // STATE_REGOING_TO_PAD,
    STATE_WAITING_AT_PAD,
    STATE_LANDING,
    STATE_CHECK_CHARGING,
    STATE_REPOSITION_ON_PAD,
    STATE_APP_TERMINATION,
    
    STATE_CRASHED,
    STATE_SNIFFING,
    STATE_UNKNOWN = 255,
};
