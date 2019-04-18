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
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

import time
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig


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
    STATE_GOING_TO_INITIAL_POSITION = 5
    STATE_RUNNING_TRAJECTORY = 6
    STATE_GOING_TO_PAD = 7
    STATE_WAITING_AT_PAD = 8
    STATE_LANDING = 9
    STATE_CHECK_CHARGING = 10

    def __init__(self, uri):
        self.uri = uri
        self.reset_internal()

    def reset_internal(self):
        self.connection_state = self.CS_DISCONNECTED
        self._cf = None
        self._log_conf = None
        self.copter_state = self.STATE_UNKNOWN
        self.vbat = -1.0
        self._time_for_next_connection_attempt = 0

        # Pre states are used to prevent multiple calls to a copter
        # when waiting for the remote state to change
        # Reset by timer to be roubust
        self._pre_state_taking_off = False
        self._pre_state_going_to_initial_position = False
        self._pre_state_reset_time = 0

    def is_starting(self):
        return self.copter_state == self.STATE_TAKING_OFF or self._pre_state_taking_off

    def is_ready_for_flight(self):
        return self.copter_state == self.STATE_HOVERING and not self._pre_state_going_to_initial_position

    def is_flying(self):
        return self.copter_state == self.STATE_RUNNING_TRAJECTORY or \
               self.copter_state == self.STATE_GOING_TO_INITIAL_POSITION or \
               self._pre_state_going_to_initial_position

    def is_landing(self):
        if self.connection_state != self.CS_CONNECTED:
            return False
        return self.copter_state == self.STATE_GOING_TO_PAD or \
               self.copter_state == self.STATE_WAITING_AT_PAD or \
               self.copter_state == self.STATE_LANDING or \
               self.copter_state == self.STATE_CHECK_CHARGING

    def is_charging(self):
        return self.copter_state == self.STATE_WAIT_FOR_TAKE_OFF and not self._pre_state_taking_off

    def take_off(self):
        if self.is_charging():
            if self._cf:
                self._pre_state_taking_off = True
                self._pre_state_reset_time = time.time() + 3.0
                self._cf.param.set_value('app.takeoff', 1)

    def start_trajectory(self):
        if self.is_ready_for_flight():
            if self._cf:
                self._pre_state_going_to_initial_position = True
                self._pre_state_reset_time = time.time() + 3.0
                self._cf.param.set_value('app.start', 1)

    def get_charge_level(self):
        return self.vbat

    def process(self):
        if self.connection_state == self.CS_DISCONNECTED and time.time() > self._time_for_next_connection_attempt:
            self._connect()

        if self._pre_state_reset_time < time.time():
            self._pre_state_taking_off = False
            self._pre_state_going_to_initial_position = False

    def _connected(self, link_uri):
        self.connection_state = self.CS_CONNECTED
        print('Connected to %s' % link_uri)

        self._setup_logging()

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self._set_disconnected(5)

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self._set_disconnected()

    def _set_disconnected(self, hold_back_time=0):
        self.reset_internal()

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

        print("Connecting to " + self.uri)
        self._cf.open_link(self.uri)

    def _setup_logging(self):
        print("Setting up logging")
        self._log_conf = LogConfig(name='Tower', period_in_ms=500)
        self._log_conf.add_variable('app.state', 'uint8_t')
        self._log_conf.add_variable('pm.vbat', 'float')

        self._cf.log.add_config(self._log_conf)
        self._log_conf.data_received_cb.add_callback(self._log_data)
        self._log_conf.start()

    def _log_data(self, timestamp, data, logconf):
        self.copter_state = data['app.state']
        self.vbat = data['pm.vbat']


class Tower:
    def __init__(self):
        self.controllers = []
        for uri in [
            'radio://0/67/2M',
            'radio://0/90/2M',
        ]:
            self.controllers.append(TrafficController(uri))

    def fly_one(self):
        while True:
            if self.flying_count() < 1:
                # print("Want more copters")
                self.prepare_copters(1)
                self.fly_copters(1)
            time.sleep(0.1)

            self.process_controllers()

    def flying_count(self):
        count = 0
        for controller in self.controllers:
            if controller.is_flying():
                count += 1
        return count

    def prepare_copters(self, count):
        prepared_count = 0
        for controller in self.controllers:
            # print(controller.copter_state)
            if controller.is_starting() or controller.is_ready_for_flight():
                prepared_count += 1

        if prepared_count < count:
            best_controller = self.find_best_controller()
            if best_controller:
                print("Preparing " + best_controller.uri)
                best_controller.take_off()

    def fly_copters(self, count):
        for controller in self.controllers:
            if controller.is_ready_for_flight():
                print("Starting prepared copter " + controller.uri)
                controller.start_trajectory()
                return

    def find_best_controller(self):
        best_controller = None
        best_charge = 0.0
        for controller in self.controllers:
            if controller.is_charging():
                charge = controller.get_charge_level()
                if charge > best_charge:
                    best_charge = charge
                    best_controller = controller

        return best_controller

    def process_controllers(self):
        for controller in self.controllers:
            controller.process()


cflib.crtp.init_drivers(enable_debug_driver=False)

tower = Tower()
tower.fly_one()
