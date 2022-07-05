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
import simpleaudio
from colorama import Fore, Back, Style
from traffic_controller import TrafficController
class TowerBase:
    def __init__(self, uris, report_socket=None):
        self.controllers:List[TrafficController] = []
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
    
    def get_flying_controllers(self)->List[TrafficController]:
        flying_controllers = []
        for controller in self.controllers:
            if controller.is_flying():
                flying_controllers.append(controller)
        
        return flying_controllers

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
    
    def upload_trajectory_multiple(self, trajectory:np.array,cf_uris=None)->list:
        """Uploads a trajectory to the crazyflies with the ids.
            @param trajectory: The trajectory to upload.
            @param cf_uris: The uris of the crazyflies to upload the trajectory to.
            
            @return: The uris of the crazyflies that failed to upload the trajectory.
        """
        
        if cf_uris ==-1:# if no crazyflies are specified, upload to all
            cf_uris = self._uris

        print("Uploading trajectory to",cf_uris)
        
        for uri in cf_uris:
            id=self.get_controller_id(uri)
            if id is not None:
                self.controllers[id].upload_trajectory(trajectory)
            else:
                print("Could not find controller for",uri)
                
    def get_controller_id(self, uri):
        for i,controller in enumerate(self.controllers):
            if controller.uri == uri:
                return i
        return None

    def trajectories_uploaded(self):
        for controller in self.get_flying_controllers():
            if not controller.is_trajectory_uploaded():
                return False
        return True
