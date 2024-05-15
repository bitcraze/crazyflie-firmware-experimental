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
    STATE_WAIT_FOR_POSITION_LOCK = 1,

    STATE_WAIT_FOR_TAKE_OFF = 2,
    STATE_QUEUED_FOR_TAKE_OFF = 3,
    STATE_PREPARING_FOR_TAKE_OFF = 4,
    STATE_TAKING_OFF = 5,
    STATE_HOVERING = 6,
    STATE_GOING_TO_TRAJECTORY_START = 7,
    STATE_EXECUTING_TRAJECTORY = 8,
    STATE_GOING_TO_RANDOM_POINT = 9,
    STATE_PREPARING_FOR_LAND = 10,
    STATE_GOING_TO_PAD = 11,
    STATE_WAITING_AT_PAD = 12,
    STATE_LANDING = 13,
    STATE_CHECK_CHARGING = 14,
    STATE_REPOSITION_ON_PAD = 15,
    STATE_CRASHED = 16,
    STATE_SNIFFING = 17,
    STATE_UNKNOWN = 255,
};
