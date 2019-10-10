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
Example of how to generate trajectories for the High Level commander using
Bezier curves. The output from this script is intended to be pasted into the
autonomous_sequence_high_level.py example.

This code uses Bezier curves of degree 7, that is with 8 control points.
See https://en.wikipedia.org/wiki/B%C3%A9zier_curve

All coordinates are (x, y, z, yaw)
"""
import math

import numpy as np
from utils.traj_util import Node, Segment


class Visualizer:
    def __init__(self):
        self.canvas = scene.SceneCanvas(keys='interactive', size=(800, 600),
                                        show=True)
        view = self.canvas.central_widget.add_view()
        view.bgcolor = '#ffffff'
        view.camera = TurntableCamera(fov=10.0, distance=40.0, up='+z',
                                      center=(0.0, 0.0, 1.0))
        XYZAxis(parent=view.scene)
        self.scene = view.scene

    def marker(self, pos, color='black', size=8):
        Markers(pos=np.array(pos, ndmin=2), face_color=color,
                parent=self.scene, size=size)

    def lines(self, points, color='black'):
        LinePlot(points, color=color, parent=self.scene)

    def line(self, a, b, color='black'):
        self.lines([a, b], color)

    def run(self):
        self.canvas.app.run()


segment_time = 2
z = 1
yaw = 0

segments = []

# Nodes with one control point has not velocity, this is similar to calling
# goto in the High-level commander

n0 = Node((0, 0, z, yaw))
n1 = Node((1, 0, z, yaw))
n2 = Node((1, 1, z, yaw))
n3 = Node((0, 1, z, yaw))

segments.append(Segment(n0, n1, segment_time))
segments.append(Segment(n1, n2, segment_time))
segments.append(Segment(n2, n3, segment_time))
segments.append(Segment(n3, n0, segment_time))


# By setting the q1 control point we get velocity through the nodes
# Increase d to 0.7 to get some more action
d = 0.1

n5 = Node((1, 0, z, yaw), q1=(1 + d, 0 + d, z, yaw))
n6 = Node((1, 1, z, yaw), q1=(1 - d, 1 + d, z, yaw))
n7 = Node((0, 1, z, yaw), q1=(0 - d, 1 - d, z, yaw))

segments.append(Segment(n0, n5, segment_time))
segments.append(Segment(n5, n6, segment_time))
segments.append(Segment(n6, n7, segment_time))
segments.append(Segment(n7, n0, segment_time))


# When setting q2 we can also control acceleration and get more action.
# Yaw also adds to the fun.

d2 = 0.2
dyaw = 2
f = -0.3

n8 = Node(
    (1, 0, z, yaw),
    q1=(1 + d2, 0 + d2, z, yaw),
    q2=(1 + 2 * d2, 0 + 2 * d2 + 0*f * d2, 1, yaw))
n9 = Node(
    (1, 1, z, yaw + dyaw),
    q1=(1 - d2, 1 + d2, z, yaw + dyaw),
    q2=(1 - 2 * d2 + f * d2, 1 + 2 * d2 + f * d2, 1, yaw + dyaw))
n10 = Node(
    (0, 1, z, yaw - dyaw),
    q1=(0 - d2, 1 - d2, z, yaw - dyaw),
    q2=(0 - 2 * d2,  1 - 2 * d2,  1, yaw - dyaw))

segments.append(Segment(n0, n8, segment_time))
segments.append(Segment(n8, n9, segment_time))
segments.append(Segment(n9, n10, segment_time))
segments.append(Segment(n10, n0, segment_time))


print('Paste this code into the autonomous_sequence_high_level.py example to '
      'see it fly')
for s in segments:
    s.print_poly_python()


# Enable this if you have Vispy installed and want a visualization of the
# trajectory
if False:
    # Import here to avoid problems for users that do not have Vispy
    from vispy import scene
    from vispy.scene import XYZAxis, LinePlot, TurntableCamera, Markers

    visualizer = Visualizer()
    for s in segments:
        s.draw_trajectory(visualizer)
        # s.draw_vel(visualizer)
        # s.draw_control_points(visualizer)

    for n in [n0, n1, n2, n3, n5, n6, n7, n8, n9, n10]:
        n.draw_unscaled_controlpoints(visualizer)

    visualizer.run()
