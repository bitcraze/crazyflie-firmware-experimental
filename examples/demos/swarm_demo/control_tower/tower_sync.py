
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
from tower_base import TowerBase

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
