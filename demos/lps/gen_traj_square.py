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
"""
Generate a square trajectory
"""
import math

import numpy as np

from traj_util import Node, Segment


segment_time = 2
z = 1.0
yaw = 0.0

segments = []

# Nodes with one control point has no velocity, this is similar to calling
# goto in the High-level commander

nodes = [
    Node((0.0, 0.0, z, yaw)),
    Node((1.0, 1.0, z, yaw)),
    Node((1.0, 1.0, z, yaw)),  # Delay
    Node((-1.0, 1.0, z, yaw)),
    Node((-1.0, 1.0, z, yaw)),  # Delay
    Node((-1.0, -1.0, z, yaw)),
    Node((-1.0, -1.0, z, yaw)),  # Delay
    Node((1.0, -1.0, z, yaw)),
    Node((1.0, -1.0, z, yaw)),  # Delay
    Node((0.0, 0.0, z, yaw)),
]

prev_node = None
for node in nodes:
    if prev_node is not None:
        segments.append(Segment(prev_node, node, segment_time))
    prev_node = node

print('Paste this code into a script to use with the high level commander')
for s in segments:
    s.print_poly_python()
