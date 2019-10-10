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
Simple example that connects to a crazyflie and send a sequence of setpoints,
one every 5 seconds.

This example is intended to work with the Loco Positioning System. It aims at
documenting how to set the Crazyflie in position control mode
and how to send setpoints.
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
    (0.0, 0.0, 1.0, 0),
    (0.7, 0.7, 1.0, 0),
    (-0.7, 0.7, 1.0, 0),
    (-0.7, -0.7, 1.0, 0),
    (0.7, -0.7, 1.0, 0),
    (0.0, 0.0, 1.0, 0),
    (0.0, 0.0, 0.2, 0),
]

def run_sequence(scf, sequence):
    cf = scf.cf

    for position in sequence:
        print('Setting position {}'.format(position))
        next_point_time = time.time() + 5.0
        while time.time() < next_point_time:
            cf.commander.send_position_setpoint(position[0],
                                                position[1],
                                                position[2],
                                                position[3])
            time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        Utils().reset_estimator(scf)
        Utils().deactivate_high_level_commander(scf)
        Utils().activate_pid_controller(scf)
        run_sequence(scf, sequence)
