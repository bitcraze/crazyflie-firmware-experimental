from vispy import scene
from vispy.scene import XYZAxis, LinePlot, Node, Mesh, TurntableCamera, Markers
import numpy as np
from vispy.visuals.transforms import MatrixTransform
import math
import threading
import time
import logging
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.mem import MemoryElement

logging.basicConfig(level=logging.ERROR)


class AnimCanvas(scene.SceneCanvas):
    def __init__(self):
        scene.SceneCanvas.__init__(self, keys='interactive', size=(800, 600),
                                   show=True)


class Visulizer:
    def __init__(self):
        self.terminate = False

        self.canvas = AnimCanvas()

        self.view = self.canvas.central_widget.add_view()
        self.view.bgcolor = '#ffffff'
        self.view.camera = TurntableCamera(fov=10.0, distance=40.0, up='+y',
                                           center=(0.0, 1.0, 0.0))

        self.scene = self.view.scene

        XYZAxis(parent=self.scene)

        self.base_stations = [
            {
                'origin': [-0.115942, 2.315495, -1.296867, ],
                'mat': [
                    [-0.992832, -0.031243, 0.115362],
                    [0.065489, 0.665217, 0.743772],
                    [-0.099979, 0.745996, -0.658403],
                ],
            },
            {
                'origin': [-0.124054, 2.293810, 1.354691],
                'mat': [
                    [0.998887, -0.047174, -0.000000],
                    [0.038335, 0.811734, 0.582768],
                    [-0.027491, -0.582120, 0.812638],
                ],
            },
        ]

        self.angles = [
            [0.19170784950256348, 0.01734159141778946],
            [-0.12198804318904877, -0.3505938947200775],
        ]

        self.objs = [{}, {}]
        self.generate_scene()

    def generate_scene(self):
        p = [0, 0, 0]

        for i in range(2):
            objs = self.objs[i]

            if i == 0:
                objs['bs'] = self.marker(p, color='green')
            else:
                objs['bs'] = self.marker(p, color='red')

            objs['tripod'] = self.line(p, p, color='blue')
            objs['center_line'] = self.line(p, p, color="green")
            objs['beam'] = self.line(p, p, color="red")

    def update_scene(self):
        normal = [0, 0, -4]
        for i in range(2):
            objs = self.objs[i]

            bs = self.base_stations[i]
            origin = bs['origin']
            self.update_marker(objs['bs'], origin)
            self.update_line(objs['tripod'], [origin[0], 0, origin[2]], origin)

            rot_bs = np.array(bs['mat'])
            center_line = np.dot(rot_bs, normal)
            self.update_line(objs['center_line'], origin,
                             np.add(center_line, origin))

            ################

            a = self.angles[i]

            a_x = a[0]
            a_y = a[1]

            beam_line = np.array([math.sin(a_x) * math.cos(a_y), -math.cos(a_x) * math.sin(a_y), math.cos(a_x) * math.cos(a_y)]) * -4
            beam_line = np.dot(rot_bs, beam_line)

            self.update_line(objs['beam'], origin, np.add(beam_line, origin))

    def marker(self, pos, color='black'):
        return Markers(pos=np.array(pos, ndmin=2), face_color=color,
                       parent=self.scene)

    def update_marker(self, marker, pos):
        marker.set_data(np.array(pos, ndmin=2))

    def line(self, p1, p2, color='black'):
        return LinePlot([p1, p2], color=color, parent=self.scene,
                        marker_size=0)

    def update_line(self, line, p1, p2):
        line.set_data(data=[p1, p2], marker_size=0)

    def run(self):
        print('starting CF thread')
        self.t = threading.Thread(target=self.cf_handler)
        self.t.start()

        self.canvas.app.run()
        self.terminate = True

    def cf_handler(self):
        uri = "radio://0/10/2M/E7E7E7E701"

        lg_block = LogConfig(name='LH', period_in_ms=50)
        lg_block.add_variable('lighthouse.angle0x', 'float')
        lg_block.add_variable('lighthouse.angle0y', 'float')
        lg_block.add_variable('lighthouse.angle1x', 'float')
        lg_block.add_variable('lighthouse.angle1y', 'float')

        cf = Crazyflie(rw_cache='./cache')
        with SyncCrazyflie(uri, cf=cf) as scf:
            print("Getting LH geometry")
            mems = scf.cf.mem.get_mems(MemoryElement.TYPE_LH)
            mems[0].update(self._update_geometry)

            with SyncLogger(scf, lg_block) as logger:
                for log_entry in logger:
                    data = log_entry[1]

                    self.angles[0][0] = data['lighthouse.angle0x']
                    self.angles[0][1] = data['lighthouse.angle0y']
                    self.angles[1][0] = data['lighthouse.angle1x']
                    self.angles[1][1] = data['lighthouse.angle1y']

                    if self.terminate:
                        break

                    self.update_scene()

    def _update_geometry(self, mem):
        print("Received LH geometry")
        for i in range(2):
            self.base_stations[i]['origin'] = mem.geometry_data[i].origin
            self.base_stations[i]['mat'] = mem.geometry_data[i].rotation_matrix

        self.update_scene()


# Initialize the low-level drivers (don't list the debug drivers)
cflib.crtp.init_drivers(enable_debug_driver=False)

viz = Visulizer()
viz.run()
