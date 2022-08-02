# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2022 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

ts = 0.05
N = 100
N_MAV = 4
(nu, nx) = (3, 3)  # control and state dimensions
# Costs Constants
(qX, qU, qN) = (500, 0.1, 89000)
(qObs, qBet) = (10000, 10000)

# Obstacles Constants
(cyl_x, cyl_y, cyl_z, cyl_r, cyl_h) = (0.5, 0.5, 0.5, 0.2, 1.0)
SWARM_COLLISION_DISTANCE = 0.6

(MIN_VELOCITY, MAX_VELOCITY) = (-1.0, 1.0)
# Distance between MAVs
# (dMAV2D, dMAVZ) = (0.5, 0.2) (not used for now)
