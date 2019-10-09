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
#  Crazyflie Nano Quadcopter Client
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
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to a crazyflie and uses the high level commander to
go to a set of points.
"""
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from utils.util import Utils


uri = 'radio://0/30/2M'


#     x    y    z  YAW
sequence = [
    (1.0, 1.0, 1.0, 0),
    (-1.0, 1.0, 1.0, 0),
    (-1.0, -1.0, 1.0, 0),
    (1.0, -1.0, 1.0, 0),
    (0.0, 0.0, 1.0, 0),
]

def run_sequence(scf, sequence):
    cf = scf.cf

    commander = cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)

    for position in sequence:
        print('Setting position {}'.format(position))
        commander.go_to(position[0], position[1], position[2], position[3], 2.0)
        time.sleep(5.0)

    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        Utils().reset_estimator(scf)
        Utils().activate_high_level_commander(scf)
        Utils().activate_pid_controller()
        run_sequence(scf, sequence)
