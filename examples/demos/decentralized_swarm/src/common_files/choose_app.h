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
 * choose_app.h - Choose app to build
 *
 */

// App selection can be controlled via compile-time flags:
// - Pass -DBUILD_PILOT_APP to build the pilot app
// - Pass -DBUILD_SNIFFER_APP to build the sniffer app
// If no flag is passed, default to pilot app

#if !defined(BUILD_PILOT_APP) && !defined(BUILD_SNIFFER_APP)
    // Default to pilot app if no flag is specified
    #define BUILD_PILOT_APP
#endif

// check if both apps are defined
#if defined(BUILD_PILOT_APP) && defined(BUILD_SNIFFER_APP)
    #error "Only one app can be defined to be built!"
#endif
