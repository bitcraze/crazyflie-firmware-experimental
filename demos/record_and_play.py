#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
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
Records and playback

This script records a trajectory and plays it back.
To use it:
 - Set CRAZYFLIE_URI
 - Launch the script, it will print "WAITING"
 - Take the Crazyflie and **move it straight up**, when you mode it 30cm up
   the trajectory recording is starting
 - Move the Crazyflie around and place it back at its initial position
 - After 2 seconds, the Crazyflie takes-off, replay the trajectory and lands
"""
import logging
import time
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander

from utils.util import Utils

CRAZYFLIE_URI = "radio://0/30/2M"

# Configuration constants
START_THRESHOLD = 0.3
STOP_RECORDING_THRESHOLD = 0.2
RECORDING_TICK_PERIOD_MS = 100
STOP_RECORDING_DELAY_TICK = 20

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Different possible states
IDLE = "IDLE"
WAITING = "WAITING"
RECORDING = "RECORDING"
PLAYING = "PLAYING"

# Calculate the distance between two points
def distance(p1, p2):
    diff = [p1[0]-p2[0], p1[1]-p2[1], p1[2]-p2[2]]
    return math.sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2] )

class RecordAndPlay():
    def __init__(self, uri):
        self._uri = uri

        # State maching state
        self._current_state = IDLE
        self._initial_z = 0
        self._start_position = [0,0,0]
        self._stop_recording_timer = 0

        # Recorded trajectory
        self._trajectory = []

        # Playback step
        self._current_step = 0

    def run(self):
        lg_stab = LogConfig(name='stateEstimate', period_in_ms=RECORDING_TICK_PERIOD_MS)
        lg_stab.add_variable('stateEstimate.x', 'float')
        lg_stab.add_variable('stateEstimate.y', 'float')
        lg_stab.add_variable('stateEstimate.z', 'float')

        cf = Crazyflie(rw_cache='./cache')
        with SyncCrazyflie(self._uri, cf=cf) as scf:
            Utils().activate_pid_controller(scf)
            Utils().deactivate_high_level_commander(scf)
            with SyncLogger(scf, lg_stab) as logger:
                for log_entry in logger:
                    data = log_entry[1]
                    position = [data['stateEstimate.x'],
                                data['stateEstimate.y'],
                                data['stateEstimate.z']]

                    print(self._current_state)

                    # Run action for this state
                    if self._current_state == RECORDING:
                        self.record(position)
                    elif self._current_state == PLAYING:
                        self.playback(cf)


                    # Update state for next time
                    self._current_state = self.get_next_state(position[0],
                                                              position[1],
                                                              position[2])

    # Calculate next step in the state machine
    # This function is responsible for the script sequencing
    def get_next_state(self, x, y, z):
        state = self._current_state

        if state == IDLE:
            self._initial_z = z
            state = WAITING
        elif state == WAITING:
            if z - self._initial_z > START_THRESHOLD:
                self._start_position = [x, y, z - START_THRESHOLD]
                state = RECORDING
        elif state == RECORDING:
            self._distance_to_start = distance(self._start_position, [x, y, z])
            if self._distance_to_start < STOP_RECORDING_THRESHOLD:
                self._stop_recording_timer += 1
                if self._stop_recording_timer > STOP_RECORDING_DELAY_TICK:
                    state = PLAYING
            else:
                self._stop_recording_timer = 0

        return state

    # Recorder
    def record(self, position):
        self._trajectory.append(position)

    # Playback
    def playback(self, cf):
        if self._current_step < len(self._trajectory):
            setpoint = self._trajectory[self._current_step]
            cf.commander.send_position_setpoint(setpoint[0], setpoint[1], setpoint[2], 0)
            self._current_step += 1
        else:
            cf.commander.send_stop_setpoint()


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    rap = RecordAndPlay(CRAZYFLIE_URI)
    rap.run()


