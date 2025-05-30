#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
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

import random
from typing import List
import time
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import statistics
import sys
import threading
import math

import numpy as np
from cflib.crazyflie.mem.trajectory_memory import TrajectoryMemory
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D

import zmq

from multi_mav_planning import main as multi_MAV
from colorama import Fore, Back, Style
from traffic_controller import TrafficController
from tower import Tower

import logging,sys
logging.basicConfig(stream=sys.stderr, level=logging.WARNING)
logging.getLogger("cflib.crazyflie").propagate = False


uris = [
    'radio://0/20/2M/E7E7E7E700',
    'radio://0/20/2M/E7E7E7E701',
    'radio://0/20/2M/E7E7E7E702',
    'radio://0/20/2M/E7E7E7E703',
    'radio://0/20/2M/E7E7E7E704',
    'radio://0/20/2M/E7E7E7E706',
    'radio://0/20/2M/E7E7E7E707',
    'radio://0/20/2M/E7E7E7E708',
    'radio://0/20/2M/E7E7E7E709'
]


count = 3
mode = 'normal'
if len(sys.argv) > 1:
    if sys.argv[1] == 'd':
        mode = 'dump'
    else:
        count = int(sys.argv[1])

if len(sys.argv) > 2:
    if sys.argv[2] == 's':
        mode = 'synch'

context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.bind("tcp://*:5555")

cflib.crtp.init_drivers(enable_debug_driver=False)

print('Starting tower with', count, 'Crazyflie(s)')
print('Flying with interleaved trajectories')
tower = Tower(uris, socket)
            
if not mode == 'dump':
    tower.fly(count)
else:
    tower.dump_state()
