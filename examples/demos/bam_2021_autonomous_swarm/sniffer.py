# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2021 Bitcraze AB
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
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.

This example utilizes the SyncCrazyflie and SyncLogger classes.
"""
import logging
from os import times
import time
from datetime import datetime
from datetime import timedelta
import struct

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper




uri = uri_helper.uri_from_env(default='usb://0')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class State:
    LOCK_COUNT = 3

    def __init__(self) -> None:
        self.lock = []
        for _ in range(self.LOCK_COUNT):
            self.lock.append({"nodeId": 0, "endTime": None })


class Sniffer:
    def __init__(self) -> None:
        # Latest accepted concensus state (that the sniffer knows about)
        self.concensusStateProposalNr = 0
        self.concensusState = State()
        self.concensusStateAcceptCount = 0

        self.latestSeqNr = [0] * 9

        self.MAJORITY = 5

    def handle_packet(self, data):
        # print(len(data), data)
        msg_type = data[0]

        if msg_type < 5:
            nodeId = data[1]
            seqNr = data[2]

            print(datetime.now().strftime("%H:%M:%S.%f: "), end='')

            if msg_type == 1:
                vals = struct.unpack('<BBL', data[1:])
                print("From {}, seq {}, Proposal,           proposalNr: {:3d}".format(*vals))
            elif msg_type == 2:
                vals = struct.unpack('<BBLL?', data[1:12])
                print("From {}, seq {},  Promise,           proposalNr: {:3d}, previousProposalId: {:3d}, propositionAccepted: {}, currentState: ".format(*vals))
                # self.print_state(self.unpack_state(data[12:27]))
            elif msg_type == 3:
                vals = struct.unpack('<BBL', data[1:7])
                print("From {}, seq {}, StateUpdateRequest, proposalNr: {:3d}, newState: ".format(*vals), end='')
                self.print_state(self.unpack_state(data[7:22]))
            elif msg_type == 4:
                vals = struct.unpack('<BBL?', data[1:8])
                print("From {}, seq {},  StateUpdateAccept, proposalNr: {:3d}, updateAccepted: {}, newState: ".format(*vals), end='')
                state = self.unpack_state(data[8:23])
                self.print_state(state)

                if seqNr != self.latestSeqNr[nodeId]:
                    proposalNr = vals[2]
                    self.handle_state_update_accept(proposalNr, state)

            self.latestSeqNr[nodeId] = seqNr

        elif msg_type == 5:
            vals = struct.unpack('<B', data[1:])
            print("ActivationUpdate,  isActive: {}".format(*vals))
        else:
            print("Warning! unknown message type:", msg_type)
            print(data)


    def handle_state_update_accept(self, proposalNr, state):
        if proposalNr == self.concensusStateProposalNr:
            self.concensusStateAcceptCount += 1
            if self.concensusStateAcceptCount == self.MAJORITY:
                self.concensusState = state
                print("New concensus!", end='')
                self.print_state(self.concensusState)
        elif proposalNr > self.concensusStateProposalNr:
            self.concensusStateProposalNr = proposalNr;
            self.concensusStateAcceptCount = 1

    def unpack_state(self, data):
        vals = struct.unpack('<BLBLBL', data)

        result = State()
        for i in range(State.LOCK_COUNT):
            result.lock[i]["nodeId"] = vals[i * 2]
            delta = vals[i * 2 + 1]
            if delta != 0:
                result.lock[i]["endTime"] = datetime.now() + timedelta(seconds=delta / 1000.0)
            else:
                result.lock[i]["endTime"] = None

        return result

    def print_state(self, state):
        result = "["
        for i in range(State.LOCK_COUNT):
            timeStr = 'free'
            endTime = state.lock[i]["endTime"]
            if endTime is not None:
                clock = endTime.strftime("%H:%M:%S.%f")
                seconds_remaining = (endTime - datetime.now()).seconds
                timeStr = "{} ({})".format(clock, seconds_remaining)
            result += "node{}: {} - {}, ".format(i, state.lock[i]["nodeId"], timeStr)

        result += ']'

        print(result)


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(uri, cf=cf) as scf:
        sniffer = Sniffer()
        cf.appchannel.packet_received.add_callback(sniffer.handle_packet)
        print("Sniffer started. Waiting for packets...")
        while True:
            time.sleep(5)
