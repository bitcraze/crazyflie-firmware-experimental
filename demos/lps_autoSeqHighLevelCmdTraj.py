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
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D

from utils.util import Utils, Uploader


uri = 'radio://0/30/2M'


#  duration   x * 8    y * 8    z * 8    YAW * 8
sequence = [
  [2, 0.0, 0.0, 0.0, 0.0, 2.1875, -2.625, 1.09375, -0.15625, 0.0, 0.0, 0.0, 0.0, 2.1875, -2.625, 1.09375, -0.15625, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [2, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [2, 1.0, 0.0, 0.0, 7.28583859910259e-16, -4.374999999999999, 5.25, -2.1875, 0.3125, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [2, -1.0, 0.0, 0.0, -7.28583859910259e-16, -3.642919299551295e-16, -1.3660947373317356e-16, 1.7076184216646695e-17, -2.134523027080837e-18, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [2, -1.0, 0.0, 0.0, -7.28583859910259e-16, -3.642919299551295e-16, -1.3660947373317356e-16, 1.7076184216646695e-17, -2.134523027080837e-18, 1.0, 0.0, 0.0, 7.28583859910259e-16, -4.374999999999999, 5.25, -2.1875, 0.3125, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [2, -1.0, 0.0, 0.0, -7.28583859910259e-16, -3.642919299551295e-16, -1.3660947373317356e-16, 1.7076184216646695e-17, -2.134523027080837e-18, -1.0, 0.0, 0.0, -7.28583859910259e-16, -3.642919299551295e-16, -1.3660947373317356e-16, 1.7076184216646695e-17, -2.134523027080837e-18, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [2, -1.0, 0.0, 0.0, -7.28583859910259e-16, 4.374999999999999, -5.25, 2.1875, -0.3125, -1.0, 0.0, 0.0, -7.28583859910259e-16, -3.642919299551295e-16, -1.3660947373317356e-16, 1.7076184216646695e-17, -2.134523027080837e-18, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [2, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, -1.0, 0.0, 0.0, -7.28583859910259e-16, -3.642919299551295e-16, -1.3660947373317356e-16, 1.7076184216646695e-17, -2.134523027080837e-18, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
  [2, 1.0, 0.0, 0.0, 7.28583859910259e-16, -2.1874999999999996, 2.625, -1.09375, 0.15625, -1.0, 0.0, 0.0, -7.28583859910259e-16, 2.1874999999999996, -2.625, 1.09375, -0.15625, 1.0, 0.0, 0.0, 7.28583859910259e-16, 3.642919299551295e-16, 1.3660947373317356e-16, -1.7076184216646695e-17, 2.134523027080837e-18, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ],  # noqa
]


def upload_trajectory(scf, trajectory_id, trajectory):
    trajectory_mem = scf.cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.poly4Ds.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    Uploader().upload(trajectory_mem)
    scf.cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.poly4Ds))
    return total_duration


def run_sequence(scf, trajectory_id, duration):
    commander = scf.cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)
    relative = True
    commander.start_trajectory(trajectory_id, 1.0, relative)
    time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        trajectory_id = 1

        Utils().reset_estimator(scf)
        Utils().activate_high_level_commander(scf)
        Utils().activate_pid_controller()
        duration = upload_trajectory(scf, trajectory_id, sequence)
        run_sequence(scf, trajectory_id, duration)



