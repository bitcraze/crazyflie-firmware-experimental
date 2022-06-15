# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
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
"""
Simple example that connects to one crazyflie (check the address at the top
and update it to your crazyflie address) and uses the high level commander
to send setpoints and trajectory to fly a figure 8.

This example is intended to work with any positioning system (including LPS).
It aims at documenting how to set the Crazyflie in position control mode
and how to send setpoints using the high level commander.
"""
import sys
import time
from typing import List

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

from cflib.crazyflie.mem.trajectory_memory import TrajectoryMemory
import numpy as np
import threading

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# The trajectory to fly
# See https://github.com/whoenig/uav_trajectories for a tool to generate
# trajectories

# Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]

def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')


def upload_trajectory(cf:Crazyflie, trajectory_id, trajectory:np.array):
    """
        Uploads a trajectory to the Crazyflie.
        The trajectory is a list of (Duration ,x, y, z, yaw) polynomial coefficients.
    
        Duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
    """
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    upload_result = Uploader().upload(trajectory_mem)
    if not upload_result:
        print('Upload failed, aborting!')
        sys.exit(1)
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration


def run_sequence(cf:Crazyflie, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)
    relative = True
    commander.start_trajectory(trajectory_id, 1.0, relative)
    time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()


class TrafficController:
    CS_DISCONNECTED = 0
    CS_CONNECTING = 1
    CS_CONNECTED = 2

    STATE_UNKNOWN = -1
    STATE_IDLE = 0
    STATE_WAIT_FOR_POSITION_LOCK = 1
    STATE_WAIT_FOR_TAKE_OFF = 2  # Charging
    STATE_TAKING_OFF = 3
    STATE_HOVERING = 4
    STATE_WAITING_TO_GO_TO_INITIAL_POSITION = 5
    STATE_GOING_TO_INITIAL_POSITION = 6
    STATE_RUNNING_TRAJECTORY = 7
    STATE_GOING_TO_PAD = 8
    STATE_WAITING_AT_PAD = 9
    STATE_LANDING = 10
    STATE_CHECK_CHARGING = 11
    STATE_REPOSITION_ON_PAD = 12
    STATE_CRASHED = 13

    NO_PROGRESS = -1000.0

    PRE_STATE_TIMEOUT = 3

    def __init__(self, uri):
        self.uri = uri
        self.stay_alive = True
        self.reset_internal()
        self.connection_thread = threading.Thread(target=self.process)
        self.connection_thread.start()

    def upload_trajectory(self, trajectory:np.array,trajectory_id):
        trajectory_mem = self._cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
        trajectory_mem:TrajectoryMemory
        self.trajectory_mem=trajectory_mem

        self.latest_trajectory_id=trajectory_id

        total_duration = 0
        for row in trajectory:
            duration = row[0]
            x = Poly4D.Poly(row[1:9])
            y = Poly4D.Poly(row[9:17])
            z = Poly4D.Poly(row[17:25])
            yaw = Poly4D.Poly(row[25:33])
            trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
            total_duration += duration

        print('Uploading trajectory ...')
        trajectory_mem.write_data(self._upload_done,
                                  write_failed_cb=self._upload_failed)
    
    def _upload_done(self, mem, addr):
        print('Trajectory upload succesfull!')
        self._traj_upload_done = True
        self._traj_upload_success = True

        print("Defining trajectory ...")
        self._cf.high_level_commander.define_trajectory(self.latest_trajectory_id, 0, len(self.trajectory_mem.trajectory))


    def _upload_failed(self, mem, addr):
        print('Trajectory upload failed!')
        self._traj_upload_done = True
        self._traj_upload_success = False

    def reset_internal(self):
        self.connection_state = self.CS_DISCONNECTED
        self._cf = None
        self._log_conf = None
        self.copter_state = self.STATE_UNKNOWN
        self.vbat = -1.0
        self._time_for_next_connection_attempt = 0
        self.traj_cycles = None
        self.est_x = 0.0
        self.est_y = 0.0
        self.est_z = 0.0
        self.up_time_ms = 0
        self.flight_time_ms = 0

        # Pre states are used to prevent multiple calls to a copter
        # when waiting for the remote state to change
        self._pre_state_taking_off_end_time = 0
        self._pre_state_going_to_initial_position_end_time = 0

    def _pre_state_taking_off(self):
        return self._pre_state_taking_off_end_time > time.time()

    def _pre_state_going_to_initial_position(self):
        return self._pre_state_going_to_initial_position_end_time > time.time()

    def is_connected(self):
        return self.connection_state == self.CS_CONNECTED

    def has_found_position(self):
        return self.copter_state > self.STATE_WAIT_FOR_POSITION_LOCK

    def is_taking_off(self):
        return self.copter_state == self.STATE_TAKING_OFF or self._pre_state_taking_off()

    def is_ready_for_flight(self):
        return self.copter_state == self.STATE_HOVERING and not self._pre_state_going_to_initial_position()

    def is_flying(self):
        return self.copter_state == self.STATE_RUNNING_TRAJECTORY or \
               self.copter_state == self.STATE_WAITING_TO_GO_TO_INITIAL_POSITION or \
               self.copter_state == self.STATE_GOING_TO_INITIAL_POSITION or \
               self._pre_state_going_to_initial_position()

    def is_landing(self):
        return self.copter_state == self.STATE_GOING_TO_PAD or \
               self.copter_state == self.STATE_WAITING_AT_PAD or \
               self.copter_state == self.STATE_LANDING or \
               self.copter_state == self.STATE_CHECK_CHARGING or \
               self.copter_state == self.STATE_REPOSITION_ON_PAD

    def is_charging(self):
        return self.copter_state == self.STATE_WAIT_FOR_TAKE_OFF and not self._pre_state_taking_off()

    def is_crashed(self):
        return self.copter_state == self.STATE_CRASHED

    def take_off(self):
        if self.is_charging():
            if self._cf:
                self._pre_state_taking_off_end_time = time.time() + self.PRE_STATE_TIMEOUT
                self._cf.param.set_value('app.takeoff', 1)

    def start_trajectory(self, trajectory_delay, offset_x=0.0, offset_y=0.0, offset_z=0.0):
        if self.is_ready_for_flight():
            if self._cf:
                self._cf.param.set_value('app.offsx', offset_x)
                self._cf.param.set_value('app.offsy', offset_y)
                self._cf.param.set_value('app.offsz', offset_z)

                self._pre_state_going_to_initial_position_end_time = time.time() + self.PRE_STATE_TIMEOUT
                self._cf.param.set_value('app.start', trajectory_delay)

    def force_land(self):
        if self.connection_state == self.CS_CONNECTED:
            self._cf.param.set_value('app.stop', 1)

    def set_trajectory_count(self, count):
        if self.connection_state == self.CS_CONNECTED:
            self._cf.param.set_value('app.trajcount', count)

    def get_charge_level(self):
        return self.vbat

    def is_charged_for_flight(self):
        return self.vbat > 4.10

    def get_traj_cycles(self):
        return self.traj_cycles

    def process(self):
        while self.stay_alive:
            if self.connection_state == self.CS_DISCONNECTED:
                if time.time() > self._time_for_next_connection_attempt:
                    self._connect()

            time.sleep(1)
        self._cf.close_link()

    def _connected(self, link_uri):
        self.connection_state = self.CS_CONNECTED
        print('Connected to %s' % link_uri)

    def _all_updated(self):
        """Callback that is called when all parameters have been updated"""

        self.set_trajectory_count(2)
        self._setup_logging()

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self._set_disconnected(5)

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self._set_disconnected()

    def _set_disconnected(self, hold_back_time=5):
        self.reset_internal()
        self._time_for_next_connection_attempt = time.time() + hold_back_time

    def _connect(self):
        if self.connection_state != self.CS_DISCONNECTED:
            print("Can only connect when disconnected")
            return

        self.connection_state = self.CS_CONNECTING

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.param.all_updated.add_callback(self._all_updated)
        # self._cf.console.receivedChar.add_callback(self._console_incoming) #print debug messages from Crazyflie

        print("Connecting to " + self.uri)
        self._cf.open_link(self.uri)

    def _console_incoming(self, console_text):
        print("CF {} DEBUG:".format( self.uri[-2:] ),console_text, end='')

    def _setup_logging(self):
        # print("Setting up logging")
        self._log_conf = LogConfig(name='Tower', period_in_ms=100)
        self._log_conf.add_variable('app.state', 'uint8_t')
        self._log_conf.add_variable('app.prgr', 'float')
        self._log_conf.add_variable('app.uptime', 'uint32_t')
        self._log_conf.add_variable('app.flighttime', 'uint32_t')
        self._log_conf.add_variable('pm.vbat', 'float')
        self._log_conf.add_variable('stateEstimate.x', 'float')
        self._log_conf.add_variable('stateEstimate.y', 'float')

        self._cf.log.add_config(self._log_conf)
        self._log_conf.data_received_cb.add_callback(self._log_data)
        self._log_conf.start()

    def _log_data(self, timestamp, data, logconf):
        self.copter_state = data['app.state']

        if self.copter_state != self.STATE_WAIT_FOR_TAKE_OFF:
            self._pre_state_taking_off_end_time = 0

        if self.copter_state != self.STATE_HOVERING:
            self._pre_state_going_to_initial_position_end_time = 0

        self.vbat = data['pm.vbat']

        self.up_time_ms = data['app.uptime']
        self.flight_time_ms = data['app.flighttime']

        self.traj_cycles = data['app.prgr']
        if self.traj_cycles <= self.NO_PROGRESS:
            self.traj_cycles = None

        self.est_x = data['stateEstimate.x']
        self.est_y = data['stateEstimate.y']

    def dump(self):
        print("***", self.uri)
        print("  Connection state:", self.connection_state)
        print("  Copter state:", self.copter_state)
        print("  Bat:", self.vbat)
        print("  Up time:", self.up_time_ms / 1000)
        print("  Flight time:", self.flight_time_ms / 1000)
        print("  _pre_state_taking_off:", self._pre_state_taking_off())
        print("  _pre_state_going_to_initial_position:", self._pre_state_going_to_initial_position())

    def terminate(self):
        self.stay_alive = False


class Master:
    def __init__(self, uris):
        self.controllers: list[TrafficController] = []
        self._uris = uris
        print("URIs:", self._uris)
        for uri in uris:
            self.controllers.append(TrafficController(uri))
                
    def connected_count(self):
        count = 0
        for controller in self.controllers:
            if controller.is_connected():
                count += 1
        return count
    
    def land_all(self):
        for controller in self.controllers:
            controller:TrafficController

            controller.force_land()
    
    def all_connected(self):
        return self.connected_count() == len(self._uris)
    
    def takeoff_all(self):
        for controller in self.controllers:
            controller:TrafficController
            controller.take_off()
    
    def run(self):
        print("Waiting for all copters to connect... :")
        while not self.all_connected():
            time.sleep(1)

        # self.takeoff_all()

        self.upload_trajectory(figure8,1,cf_ids=[0,1])


    def upload_trajectory(self, trajectory:np.array,trajectory_id:int,cf_ids):
        if type(cf_ids) is int:
            cf_ids = [cf_ids]

        print("Uploading trajectory",trajectory_id,"to",cf_ids)
        
        for cf_id in cf_ids:
            if cf_id >= len(self.controllers):
                print("Invalid copter id:",cf_id)
                continue

            self.controllers[cf_id].upload_trajectory(trajectory,trajectory_id)





if __name__ == '__main__':
    cflib.crtp.init_drivers()

    uris = [
        uri_helper.uri_from_env(default='radio://0/40/2M/E7E7E7E704'),
        # uri_helper.uri_from_env(default='radio://0/40/2M/E7E7E7E704'),
        ]

    master = Master(uris)
    master.run()

