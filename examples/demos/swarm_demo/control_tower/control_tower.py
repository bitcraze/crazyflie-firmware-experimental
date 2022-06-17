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

#CONSTANTS
TRAJECTORY_SEGMENT_SIZE_BYTES = 132

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

generated = [
    # duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
[0.607070,0.023822,0.000000,0.000000,0.000000,5.677483,-12.861443,10.477816,-3.033793,0.015614,0.000000,0.000000,0.000000,3.697061,-8.348798,6.781650,-1.958805,1.003822,0.000000,0.000000,0.000000,0.919415,-2.091987,1.711222,-0.497150,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,-0.000000],
[0.607070,0.166753,0.467223,-0.127178,-1.150545,0.804131,1.823992,-2.414253,0.816415,0.109299,0.307552,-0.079086,-0.757424,0.506325,1.193214,-1.542268,0.514301,1.026755,0.074508,-0.021896,-0.183447,0.136266,0.293452,-0.401411,0.138293,0.000000,-0.000000,-0.000000,0.000000,0.000000,0.000000,0.000000,-0.000000],
[0.607070,0.309685,0.090744,0.131409,0.602434,-0.612568,-0.651324,1.055093,-0.371136,0.202985,0.054760,0.079094,0.413349,-0.370340,-0.444090,0.643248,-0.212801,1.049688,0.016202,0.023519,0.090183,-0.109121,-0.098380,0.186261,-0.070353,0.000000,-0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
[0.607286,0.452616,0.317774,-0.076234,-0.328604,0.336852,0.319465,-0.522046,0.180314,0.296670,0.218454,-0.038545,-0.255909,0.171252,0.251975,-0.261049,0.063164,1.072621,0.047497,-0.015980,-0.038358,0.071022,0.035585,-0.112703,0.048397,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
[0.592681,0.595394,0.185237,0.033668,0.187795,-0.167606,-0.186241,0.244469,-0.073027,0.390738,0.106084,0.015484,0.220733,-0.012622,-0.210025,0.007461,0.061352,1.095400,0.034613,0.005003,-0.009046,-0.065026,-0.000256,0.093031,-0.048555,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
[0.549856,0.730523,0.253680,-0.033088,-0.161220,0.036478,0.144411,-0.058505,-0.011896,0.489989,0.238850,0.052130,-0.250921,-0.148646,0.269080,0.261996,-0.230705,1.110531,0.002790,-0.051045,0.013569,0.070599,-0.027602,-0.108411,0.069238,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
[0.600705,0.842001,0.141378,-0.085560,0.123041,0.098957,-0.124137,-0.104294,0.086479,0.599040,0.127103,-0.068377,0.368680,0.436942,-0.331350,-0.625988,0.415640,1.102010,-0.026006,-0.022046,-0.053543,-0.094078,0.054341,0.158086,-0.099364,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
[0.828262,0.923440,0.156260,0.019688,-0.145150,-0.182312,0.155285,0.259346,-0.198118,0.743930,0.450113,0.283868,-0.596685,-0.793230,0.562075,1.121753,-0.812234,1.063450,-0.115136,-0.058880,0.148996,0.170966,-0.128837,-0.259732,0.189417,0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,0.000000,-0.000000],

]

class TrajectoryUploadConfig():
    def __init__(self) -> None:
        self.trajectory_id = None
        self.trajectory_memory_offset = None
        self.pol_segment_count = None
        self.defined=False
        self.should_upload=False

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
        
        self._traj_upload_done = False
        self._traj_upload_configs: list[TrajectoryUploadConfig] =[TrajectoryUploadConfig() for i in range(2)]

    def upload_trajectory(self, trajectory:np.array,trajectory_id):
        trajectory_mem = self._cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
        trajectory_mem:TrajectoryMemory

        self.trajectory_mem=trajectory_mem

        segments_in_memory=len(trajectory_mem.trajectory)
        
        id=trajectory_id-1
        self.latest_trajectory_id=trajectory_id

        self._traj_upload_configs[id].trajectory_id=id
        self._traj_upload_configs[id].pol_segment_count=len(trajectory)
        self._traj_upload_configs[id].defined=False
        self._traj_upload_configs[id].should_upload=True
        self._traj_upload_configs[id].trajectory_memory_offset= 0 if id == 1 else 15

        total_duration = 0
        for i,row in enumerate(trajectory):
            duration = row[0]
            x = Poly4D.Poly(row[1:9])
            y = Poly4D.Poly(row[9:17])
            z = Poly4D.Poly(row[17:25])
            yaw = Poly4D.Poly(row[25:33])

            pol=Poly4D(duration, x, y, z, yaw)
            
            index=self._traj_upload_configs[id].trajectory_memory_offset + i
            trajectory_mem.trajectory[index] = pol

            total_duration += duration
    
        print('Uploading trajectory with id:{} in {}...'.format(trajectory_id,self.uri))
        trajectory_mem.write_data(self._upload_done,
                                  write_failed_cb=self._upload_failed)
    
    def is_trajectory_uploaded(self):
        return self._traj_upload_done

    def _upload_done(self, mem, addr):
        print('Trajectory upload succesfull in address!',addr)
        self._traj_upload_done = True
        self._traj_upload_success = True

        # self.latest_offset refers to pol segments so multiplcation by 132 is needed to get the real offset
        # since each segment is 132 bytes long
        for i,conf in enumerate(self._traj_upload_configs):
            if conf.defined:
                continue
            

            id=conf.trajectory_id+1
            offset=conf.trajectory_memory_offset
            pol_segment_count=conf.pol_segment_count
            print("Defining trajectory with id: {}...".format(id))

            self._cf.high_level_commander.define_trajectory(id, offset*TRAJECTORY_SEGMENT_SIZE_BYTES, pol_segment_count)

            conf.defined=True

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
        return self.vbat > 3.6

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

        # append traj memory
        trajectory_mem = self._cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
        trajectory_mem:TrajectoryMemory
        row=np.zeros(33)
        for i in range(30):
            duration = row[0]
            x = Poly4D.Poly(row[1:9])
            y = Poly4D.Poly(row[9:17])
            z = Poly4D.Poly(row[17:25])
            yaw = Poly4D.Poly(row[25:33])

            pol=Poly4D(duration, x, y, z, yaw)
            
            trajectory_mem.trajectory.append(pol)

        
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


class TowerBase:
    def __init__(self, uris, report_socket=None):
        self.controllers:list[TrafficController] = []
        self._uris = uris
        for uri in uris:
            self.controllers.append(TrafficController(uri))
        self.report_socket = report_socket

    def connected_count(self):
        count = 0
        for controller in self.controllers:
            if controller.is_connected():
                count += 1
        return count

    def flying_count(self):
        count = 0
        for controller in self.controllers:
            if controller.is_flying():
                count += 1
        return count

    def find_best_controllers(self):
        too_low_battery = []

        charging_controllers = []
        for controller in self.controllers:
            if controller.is_charging():
                charge = controller.get_charge_level()
                if controller.is_charged_for_flight():
                    charging_controllers.append((controller, charge))
                else:
                    too_low_battery.append(
                        "{} ({:.2f}V)".format(controller.uri, charge))

        if len(too_low_battery) > 0:
            print("Ready but must charge:", too_low_battery)

        charging_controllers.sort(key=lambda d: d[1], reverse=True)

        return list(map(lambda d: d[0], charging_controllers))

    def land_all(self):
        for controller in self.controllers:
            controller.force_land()

    def dump_state(self):
        print('Waiting for connections...')
        end_time = time.time() + 40
        while time.time() < end_time and self.connected_count() < len(
                self._uris):
            time.sleep(1)

        print("Dumping state")
        print()
        for controller in self.controllers:
            controller.dump()
            controller.terminate()

    def send_report(self):
        if self.report_socket is None:
            return

        for i, controller in enumerate(self.controllers):
            state = "idle"
            if not controller.is_connected():
                state = "disconnected"
            elif controller.is_crashed():
                state = "crashed"
            elif controller.is_flying():
                state = "flying"
            elif controller.is_taking_off():
                state = "hovering"
            elif controller.is_landing():
                state = "landing"
            elif controller.is_charged_for_flight():
                state = "ready"
            elif controller.is_charging():
                state = "charging"

            try:
                report = {
                    'id': i,
                    'state': state,
                    'battery': controller.get_charge_level(),
                    'uptime': controller.up_time_ms,
                    'flighttime': controller.flight_time_ms,
                }
                self.report_socket.send_json(report, zmq.NOBLOCK)
            except Exception:
                pass
    
    def upload_trajectory_multiple(self, trajectory:np.array,trajectory_id:int,cf_uris=None)->list:
        """Uploads a trajectory to the crazyflies with the ids.
            @param trajectory: The trajectory to upload.
            @param trajectory_id: The id of the trajectory.
            @param cf_uris: The uris of the crazyflies to upload the trajectory to.
            
            @return: The uris of the crazyflies that failed to upload the trajectory.
        """
        
        if cf_uris ==-1:# if no crazyflies are specified, upload to all
            cf_uris = self._uris

        print("Uploading trajectory",trajectory_id,"to",cf_uris)
        
        for uri in cf_uris:
            id=self.get_controller_id(uri)
            if id is not None:
                self.controllers[id].upload_trajectory(trajectory,trajectory_id)
            else:
                print("Could not find controller for",uri)
                
    def get_controller_id(self, uri):
        for i,controller in enumerate(self.controllers):
            if controller.uri == uri:
                return i
        return None

    def trajectories_uploaded(self):
        for controller in self.controllers:
            if not controller.is_trajectory_uploaded():
                return False
        return True
class Tower(TowerBase):
    def __init__(self, uris, report_socket=None):
        TowerBase.__init__(self, uris, report_socket)

    def fly(self, wanted):
         # Wait for all CF to connect (to avoid race)
        time.sleep(10)

        self.upload_trajectory_multiple(generated,1,cf_uris=-1)
        self.upload_trajectory_multiple(figure8,2,cf_uris=-1)

        print("Waiting for trajectories to be uploaded..")
        #wait until cfs have received trajectory
        while not self.trajectories_uploaded():
            time.sleep( 0.5)

        print("All trajectories uploaded")

        while True:
            # print()
            if wanted:
                currently_flying = self.flying_count()
                missing = wanted - currently_flying
                if missing > 0:
                    print("Want", missing, "more copters")
                    self.prepare_copters(missing)
                    self.start_copters(missing, wanted)
            else:
                self.land_all()

            self.send_report()

            time.sleep(0.2)

    def prepare_copters(self, count):
        prepared_count = 0
        for controller in self.controllers:
            if controller.is_taking_off() or controller.is_ready_for_flight():
                prepared_count += 1

        missing = count - prepared_count
        new_prepared_count = 0
        if missing > 0:
            print("Trying to prepare", missing, "copter(s)")
            best_controllers = self.find_best_controllers()
            for best_controller in best_controllers[:missing]:
                if best_controller:
                    print("Preparing " + best_controller.uri)
                    new_prepared_count += 1
                    best_controller.take_off()
            print("Prepared", new_prepared_count, "copter(s)")

    def start_copters(self, count, total):
        unused_slot_times = self.find_unused_slot_times(total)
        # print("Unused slot times:", unused_slot_times)

        slot_index = 0
        for controller in self.controllers:
            if controller.is_ready_for_flight():
                if slot_index < count and slot_index < len(unused_slot_times):
                    trajectory_delay = 1.0 - unused_slot_times[slot_index]
                    if trajectory_delay == 1.0:
                        trajectory_delay = 0.0
                    print("Starting prepared copter", controller.uri,
                          'with a delay of', trajectory_delay)
                    controller.start_trajectory(trajectory_delay, offset_z=0.25)
                    slot_index += 1
                else:
                    return

    def find_unused_slot_times(self, total_slots):
        # Times are measured in trajectory cycles
        start_times = []
        for controller in self.controllers:
            if controller.is_flying():
                start_time = controller.get_traj_cycles()

                # If a is flying but has not updated the start time yes we do
                # not have enough information to calculate empty slots.
                # Return no unused slots for now
                if start_time is None:
                    # print("Start time is unknown, hold back")
                    return []

                start_times.append(start_time)

        # print("Used start times", start_times)
        return self.crunch_slot_times(start_times, total_slots)

    def crunch_slot_times(self, start_times, total_slots):
        # Start times may be multiple cycles ago, remove integer parts
        start_time_fractions = list(map(lambda t: t % 1.0, start_times))

        # Find the average offset
        offsets = list(
            map(lambda t: (t * total_slots) % 1.0, start_time_fractions))
        offset = 0.0
        if len(start_times) > 0:
            offset = statistics.mean(offsets) / total_slots

        adjusted_start_times = list(
            map(lambda t: t - offset, start_time_fractions))

        closest_slots = list(
            map(lambda t: round(t * total_slots), adjusted_start_times))
        unused_slots = list(
            filter(lambda s: s not in closest_slots, range(total_slots)))

        unsued_slot_times = list(
            map(lambda s: offset + s / total_slots, unused_slots))
        return unsued_slot_times


class SyncTower(TowerBase):
    def __init__(self, uris, report_socket=None):
        TowerBase.__init__(self, uris, report_socket)
        self.spacing = 0.40
        self.line_orientation = math.radians(40)

        master_offset = [0, 0]
        self._start_position = [
            [   0 + master_offset[0],    0 + master_offset[1], 0],
            [   0 + master_offset[0],  0.5 + master_offset[1], 0],
            [   0 + master_offset[0], -0.5 + master_offset[1], 0],
            [ 0.5 + master_offset[0],    0 + master_offset[1], 0],
            [-0.5 + master_offset[0],    0 + master_offset[1], 0],
            [ 0.5 + master_offset[0],  0.5 + master_offset[1], 0],
            [-0.5 + master_offset[0], -0.5 + master_offset[1], 0],
            [-0.5 + master_offset[0],  0.5 + master_offset[1], 0],
            [ 0.5 + master_offset[0], -0.5 + master_offset[1], 0]
        ]


    def fly(self, wanted):
        while True:
            if wanted:
                best = self.find_best_controllers()
                ready = list(
                    filter(lambda ctrlr: ctrlr.has_found_position(), best))
                found_count = len(ready)

                if found_count >= wanted:
                    self.start_line(wanted, ready)
                    print("Started, I'm done")
                    sys.exit(0)
                else:
                    print('Can only find ', found_count,
                          'copter(s) that are charged and ready')
            else:
                self.land_all()

            self.send_report()

            time.sleep(0.2)

    def start_line(self, wanted, best):
        self.prepare_copters(wanted, best)

        while not self.start_copters(wanted, best):
            time.sleep(1)

    def prepare_copters(self, count, best_controllers):
        prepared_count = 0
        for controller in self.controllers:
            if controller.is_taking_off() or controller.is_ready_for_flight():
                prepared_count += 1

        missing = count - prepared_count
        new_prepared_count = 0
        if missing > 0:
            print("Trying to prepare", missing, "copter(s)")
            for best_controller in best_controllers[:missing]:
                if best_controller:
                    print("Preparing " + best_controller.uri)
                    new_prepared_count += 1
                    best_controller.take_off()
            print("Prepared", new_prepared_count, "copter(s)")

    def start_copters(self, wanted, best):
        ready = []
        for controller in best:
            if controller.is_ready_for_flight():
                ready.append(controller)

        if len(ready) >= wanted:
            ready_positions = []
            for controller in ready:
                ready_positions.append([controller.est_x, controller.est_y, controller.est_z])

            offsets = self.get_start_offsets(ready_positions, self._start_position[:wanted])

            index = 0
            for controller in ready:
                offset_x = offsets[index][0]
                offset_y = offsets[index][1]
                offset_z = offsets[index][2]
                controller.start_trajectory(0.0, offset_x=offset_x,
                                            offset_y=offset_y,
                                            offset_z=offset_z)
                index += 1

            return True
        else:
            return False

    def calculate_distance(self, p1, p2):
        diff = [0,0,0]
        diff[0] = p1[0]-p2[0]
        diff[1] = p1[1]-p2[1]
        diff[2] = p1[2]-p2[2]
        return math.sqrt( (diff[0]*diff[0]) + (diff[1]*diff[1]) + (diff[2]*diff[2]) )

    def find_closest_target(self, start_position, targets_position):
        min_distance = None
        closest_index = 0
        for index, target in enumerate(targets_position):
            dist = self.calculate_distance(start_position, target)
            if min_distance is None or dist < min_distance:
                min_distance = dist
                closest_index = index
        
        return targets_position[closest_index]

    def get_start_offsets(self, start_positions, targets_positions):
        offsets = []
        target_used = [False,] * len(targets_positions)
        for start in start_positions:
            candidate_targets = []
            for i in range(len(targets_positions)):
                if not target_used[i]:
                    candidate_targets.append(targets_positions[i])
            
            closest_target = self.find_closest_target(start, candidate_targets)
            target_used[targets_positions.index(closest_target)] = True
            offsets.append(closest_target)
        
        return offsets


uris = [
    # 'radio://0/10/2M/E7E7E7E701',
    # 'radio://0/10/2M/E7E7E7E702',
    # 'radio://0/10/2M/E7E7E7E703',
    'radio://0/40/2M/E7E7E7E704',
    # 'radio://0/10/2M/E7E7E7E705',
    # 'radio://0/10/2M/E7E7E7E706',
    # 'radio://0/10/2M/E7E7E7E707',
    # 'radio://0/10/2M/E7E7E7E708',
    # 'radio://0/10/2M/E7E7E7E709'
]

count = 1
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
if mode == 'synch':
    print('Flying with synchronized trajectories')
    tower = SyncTower(uris, socket)
else:
    print('Flying with interleaved trajectories')
    tower = Tower(uris, socket)

if not mode == 'dump':
    tower.fly(count)
else:
    tower.dump_state()
