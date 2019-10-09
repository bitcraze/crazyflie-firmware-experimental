# -*- coding: utf-8 -*-

# Visualization of a LPS system using vispy

import math
import numpy as np
import threading
import logging

from vispy import scene
from vispy.scene import XYZAxis, LinePlot, TurntableCamera, Markers
from vispy import app

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import LighthouseBsGeometry


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

    def marker(self, pos, color='black', size=8, prev=None):
        if prev is None:
            return Markers(pos=np.array(pos, ndmin=2), face_color=color,
                           size=size, parent=self.scene)

        prev.set_data(pos=np.array(pos, ndmin=2), face_color=color,
                      size=size)
        prev.parent = self.scene
        return prev

    def lines(self, points, color='black', prev=None):
        if prev is None:
            return LinePlot(points, color=color, marker_size=0.0, parent=self.scene)

        prev.set_data(points, color=color, marker_size=0.0)
        prev.parent=self.scene
        return prev

    def line(self, a, b, color='black', prev=None):
        return self.lines([a, b], color, prev=prev)

    def body_orientation(self, center, rot_mat, length=0.3, prev=None):
        xv = np.dot(rot_mat, np.array([length, 0.0, 0.0]))
        yv = np.dot(rot_mat, np.array([0.0, length, 0.0]))
        zv = np.dot(rot_mat, np.array([0.0, 0.0, length]))

        prev_l = prev
        if prev is None:
            prev_l = [None, None, None]

        prev_l[0] = self.line(center, center + xv, color="red", prev=prev_l[0])
        prev_l[1] = self.line(center, center + yv, color="green", prev=prev_l[1])
        prev_l[2] = self.line(center, center + zv, color="blue", prev=prev_l[2])

        return prev_l

    def run(self):
        self.canvas.app.run()

    def run_timer(self, interval, callback):
        timer = app.Timer(interval=interval, connect=callback, iterations=-1, start=True, app=self.canvas.app)
        timer.start()
        self.run()

    def remove(self, visual):
        visual.parent = None


class Marker:
    def __init__(self, color='black', size=8):
        self.color = color
        self.marker = None
        self.size = size

    def visualize(self, pos, visualizer):
        if pos is None:
            if self.marker is not None:
                self.marker.parent = None
        else:
            if self.marker is None:
                self.marker = visualizer.marker(pos, color=self.color, size=self.size)
            else:
                self.marker = visualizer.marker(pos, color=self.color, size=self.size, prev=self.marker)


class Line:
    def __init__(self, color='black'):
        self.color = color
        self.line = None

    def visualize(self, a, b, visualizer):
        if a is None or b is None:
            if self.line is not None:
                self.line.parent = None
        else:
            if self.line is None:
                self.line = visualizer.line(a, b, color=self.color)
            else:
                self.line = visualizer.line(a, b, color=self.color, prev=self.line)

class Deck:
    def __init__(self):
        self.pos = np.array((0.0, 0.0, 0.0))
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.r = self._rotation_matrix(self.roll, self.pitch, self.yaw)

        self.vis_body_or = None
        self.vis_marker = Marker(color='red')

    def update_data(self, coms):
        self.roll = coms.roll
        self.pitch = coms.pitch
        self.yaw = coms.yaw
        self.pos[0] = coms.x
        self.pos[1] = coms.y
        self.pos[2] = coms.z

        self.r = self._rotation_matrix(self.roll, self.pitch, self.yaw)

    def visualize(self, visualizer):
        self.vis_marker.visualize(self.pos, visualizer)
        self.vis_body_or = visualizer.body_orientation(self.pos, self.r, prev=self.vis_body_or)

    def _rotation_matrix(self, roll, pitch, yaw):
        # http://planning.cs.uiuc.edu/node102.html
        # Pitch reversed compared to page above
        cg = math.cos(roll)
        cb = math.cos(-pitch)
        ca = math.cos(yaw)
        sg = math.sin(roll)
        sb = math.sin(-pitch)
        sa = math.sin(yaw)

        r = [
            [ca * cb, ca * sb * sg - sa * cg, ca * sb * cg + sa * sg],
            [sa * cb, sa * sb * sg + ca * cg, sa * sb * cg - ca * sg],
            [-sb, cb * sg, cb * cg],
        ]

        return np.array(r)

    def rotate(self, vec):
        return np.dot(self.r, vec)


class Basestation:
    def __init__(self, index):
        self.index = index
        self.pos = None
        self.r = None
        self.is_geo_set = False

        self.vis_body_or = None
        self.vis_marker = Marker(color='green')

        # Rotation matrix from open cv to crayzlife coordinate systems
        self.opencv_to_cf = np.array([
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ])

        self.cf_to_opencv = np.array([
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
        ])

    def update_data(self, coms):
        if coms.got_geo_data and not self.is_geo_set:
            # Transform data to cf coordinate system
            self.pos = np.dot(self.opencv_to_cf, coms.geo_data[self.index].origin)
            self.r = np.dot(self.opencv_to_cf, np.dot(coms.geo_data[self.index].rotation_matrix, self.cf_to_opencv))

            self.is_geo_set = True

    def visualize(self, visualizer):
        self.vis_marker.visualize(self.pos, visualizer)

        if self.r is not None:
            self.vis_body_or = visualizer.body_orientation(self.pos, self.r, prev=self.vis_body_or)

    def rotate(self, mat):
        if self.r is not None:
            return np.dot(self.r, mat)

        return mat


class Setup:
    def __init__(self, deck, basestations):
        self.deck = deck
        self.bss = basestations

        self.cage_added = False

    def update_data(self, coms):
        self.deck.update_data(coms)
        for bs in self.bss:
            bs.update_data(coms)

    def visualize(self, visualizer):
        self.deck.visualize(visualizer)
        for bs in self.bss:
            bs.visualize(visualizer)

        if not self.cage_added:
            w = 4.0 / 2
            l = 4.0 / 2
            h = 3.0

            visualizer.lines([
                [l, w, 0.0],
                [l, -w, 0.0],
                [-l, -w, 0.0],
                [-l, w, 0.0],
                [l, w, 0.0],
            ])

            visualizer.lines([
                [l, w, h],
                [l, -w, h],
                [-l, -w, h],
                [-l, w, h],
                [l, w, h],
            ])

            visualizer.line([l, w, 0.0], [l, w, h])
            visualizer.line([-l, w, 0.0], [-l, w, h])
            visualizer.line([-l, -w, 0.0], [-l, -w, h])
            visualizer.line([l, -w, 0.0], [l, -w, h])

            self.cage_added = True

class Communicator:
    def __init__(self, uri):
        logging.basicConfig(level=logging.ERROR)
        cflib.crtp.init_drivers(enable_debug_driver=False)

        self.uri = uri

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.got_geo_data = False
        self.geo_data = None

        self._cf = Crazyflie(rw_cache='./cache')
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % self.uri)

        self._cf.open_link(self.uri)
        self.is_connected = True

        # self.set_static_geo_data()

    def set_static_geo_data(self):
        # Rotation angle in degrees
        angle = math.radians(-45 + 180.0)

        raw = [
            [[0.021855, 2.318295, 1.666880, ], [[0.999998, -0.001797, 0.000000, ], [0.001451, 0.807746, 0.589529, ], [-0.001059, -0.589528, 0.807747, ], ]],
            [[-0.176309, 2.290894, -1.768068, ], [[-0.995269, 0.091770, -0.031898, ], [0.053428, 0.791198, 0.609222, ], [0.081146, 0.604635, -0.792358, ], ]],
        ]

        # Rotation matrix around Y-axis
        c = math.cos(angle)
        s = math.sin(angle)
        r = np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c],
        ])

        def convert(raw, index):
            bs = LighthouseBsGeometry()
            bs.origin = np.dot(r, raw[index][0])
            bs.rotation_matrix = np.dot(r, raw[index][1])
            return bs

        self.got_geo_data = True

        self.geo_data = [convert(raw, 0), convert(raw, 1)]


    def _connected(self, link_uri):
        print('Connected to %s' % link_uri)

        lg_pose = LogConfig(name='test', period_in_ms=50)
        lg_pose.add_variable('stabilizer.roll', 'float')
        lg_pose.add_variable('stabilizer.pitch', 'float')
        lg_pose.add_variable('stabilizer.yaw', 'float')
        lg_pose.add_variable('stateEstimate.x', 'float')
        lg_pose.add_variable('stateEstimate.y', 'float')
        lg_pose.add_variable('stateEstimate.z', 'float')

        self._cf.log.add_config(lg_pose)
        lg_pose.data_received_cb.add_callback(self.data_receivedPose)

        if not self.got_geo_data:
            lg_pose.start()

        self.read_geo_data()

    def data_receivedPose(self, timestamp, data, logconf):
        self.roll = math.radians(data['stabilizer.roll'])
        self.pitch = math.radians(data['stabilizer.pitch'])
        self.yaw = math.radians(data['stabilizer.yaw'])
        self.x = data['stateEstimate.x']
        self.y = data['stateEstimate.y']
        self.z = data['stateEstimate.z']

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def read_geo_data(self):
        mems = self._cf.mem.get_mems(MemoryElement.TYPE_LH)
        self.got_geo_data = False

        print('Rquesting geo data')
        mems[0].update(self.mem_updated)

    def mem_updated(self, mem):
        self.geo_data = mem.geometry_data
        self.got_geo_data = True
        mem.dump()


################################################################

class Main:
    def run(self):
        self.coms = Communicator('radio://0/30/2M/E7E7E7E7E7')

        bss = [Basestation(0), Basestation(1)]
        self.setup = Setup(Deck(), bss)

        self.visualizer = Visualizer()
        self.visualizer.run_timer(0.03, self.update)

    def update(self, ev):
        self.setup.update_data(self.coms)
        self.setup.visualize(self.visualizer)



Main().run()
