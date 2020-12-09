
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
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
Start some Crazyflies with fixed time separation
"""
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger

# Change uris and sequences according to your setup
URI0 = 'radio://0/10/2M/E7E7E7E708'
URI1 = 'radio://0/10/2M/E7E7E7E702'
URI2 = 'radio://0/10/2M/E7E7E7E703'
URI3 = 'radio://0/10/2M/E7E7E7E706'
URI4 = 'radio://0/10/2M/E7E7E7E701'

SEPARATION = 1.5

swarm_args = {
    URI0: [{'delay': SEPARATION * 1 +1 , 'landX': -0.5, 'landY': 0.0, 'id':1}],
    URI1: [{'delay': SEPARATION * 3, 'landX': 0.0, 'landY': 0.5, 'id':2}],
    URI2: [{'delay': SEPARATION * 4, 'landX': 0.0, 'landY': -0.5, 'id':3}],
    URI3: [{'delay': SEPARATION * 5, 'landX': 0.5, 'landY': -0.25, 'id':6}],
    URI4: [{'delay': SEPARATION * 6+0.5, 'landX': 0.5, 'landY': 0.25, 'id':8}],

    # URI2: [SEPARATION * 2],
}


def wait_for_position_estimator(scf,data):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                #print(data['id'])

                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(scf)


def setup(scf, data):
    cf = scf.cf

    x = data['landX']
    y = data['landY']

    cf.param.set_value('hyper.landX', x)
    cf.param.set_value('hyper.landY', y)
    print(scf.cf.link_uri, ': setting landing position to', x, y)


def run_trajectory(scf, data):
    cf = scf.cf

    delay =  data['delay']
    print(scf.cf.link_uri, ': delaying take off', delay, 'seconds')
    time.sleep(delay)
    print(scf.cf.link_uri, ': take off')
    for _ in range(5):
        cf.param.set_value('hyper.takeoff', 1)
        time.sleep(0.05)
    time.sleep(1)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(swarm_args.keys(), factory=factory) as swarm:
        swarm.parallel_safe(wait_for_position_estimator, args_dict=swarm_args)
        swarm.parallel_safe(setup, args_dict=swarm_args)
        swarm.parallel_safe(run_trajectory, args_dict=swarm_args)
