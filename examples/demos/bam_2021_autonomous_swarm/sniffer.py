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
import time
from datetime import datetime
import struct

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper


uri = uri_helper.uri_from_env(default='usb://0')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def print_state(data):
    vals = struct.unpack('<BLBL', data)
    print("[nodeId1: {}, timeRemaining1: {}, nodeId2: {}, timeRemaining2: {}]".format(*vals))

def print_packet(data):
#    print("Length:", len(data))
#    print(data)
    msg_type = data[0]

    print(datetime.now().strftime("%H:%M:%S.%f: "), end='')

    if msg_type == 1:
        vals = struct.unpack('<BL', data[1:])
        print("From {}, Proposal,           proposalNr: {:3d}".format(*vals))
    elif msg_type == 2:
        vals = struct.unpack('<BLL?', data[1:11])
        print("From {}, Promise,            proposalNr: {:3d}, previousProposalId: {:3d}, propositionAccepted: {}, currentState: ".format(*vals), end='')
        print_state(data[11:21])
    elif msg_type == 3:
        vals = struct.unpack('<BB', data[1:3])
        print("From {}, StateUpdateRequest, proposalNr: {:3d}, newState: ".format(*vals), end='')
        print_state(data[3:13])
    elif msg_type == 4:
        vals = struct.unpack('<BB?', data[1:4])
        print("From {}, StateUpdateAccept,  proposalNr: {:3d}, updateAccepted: {}".format(*vals))
    else:
        print("Warning! unknown message type:", msg_type)
        print(data)


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(uri, cf=cf) as scf:
        cf.appchannel.packet_received.add_callback(print_packet)
        print("Sniffer started. Waiting for packets...")
        while True:
            time.sleep(5)
