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
import copy

figure8 = [
    # duration, x^0,       x^1,     x^2,        x^3,     x^4,       x^5,        x^6,        x^7,    y^0,        y^1,        y^2,    y^3,        y^4,       y^5,     y^6,        y^7,    z^0,        z^1,      z^2,      z^3,      z^4,      z^5,      z^6,     z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7

    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.110000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.110000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],  # noqa
]

generated = [
    # duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
[0.607070,0.023822,0.000000,0.000000,0.000000,5.677483,-12.861443,10.477816,-3.033793,0.015614,0.000000,0.000000,0.000000,3.697061,-8.348798,6.781650,-1.958805,1.003822,0.000000,0.000000,0.000000,0.919415,-2.091987,1.711222,-0.497150,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,-0.000000],
[0.607070,0.166753,0.467223,-0.127178,-1.150545,0.804131,1.823992,-2.414253,0.816415,0.109299,0.307552,-0.079086,-0.757424,0.506325,1.193214,-1.542268,0.514301,1.026755,0.074508,-0.021896,-0.183447,0.136266,0.293452,-0.401411,0.138293,0.000000,-0.000000,-0.000000,0.000000,0.000000,0.000000,0.000000,-0.000000],
[0.607070,0.309685,0.090744,0.131409,0.602434,-0.612568,-0.651324,1.055093,-0.371136,0.202985,0.054760,0.079094,0.413349,-0.370340,-0.444090,0.643248,-0.212801,1.049688,0.016202,0.023519,0.090183,-0.109121,-0.098380,0.186261,-0.070353,0.000000,-0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
[0.607286,0.452616,0.317774,-0.076234,-0.328604,0.336852,0.319465,-0.522046,0.180314,0.296670,0.218454,-0.038545,-0.255909,0.171252,0.251975,-0.261049,0.063164,1.072621,0.047497,-0.015980,-0.038358,0.071022,0.035585,-0.112703,0.048397,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
[0.592681,0.595394,0.185237,0.033668,0.187795,-0.167606,-0.186241,0.244469,-0.073027,0.390738,0.106084,0.015484,0.220733,-0.012622,-0.210025,0.007461,0.061352,1.095400,0.034613,0.005003,-0.009046,-0.065026,-0.000256,0.093031,-0.048555,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
[0.549856,0.730523,0.253680,-0.033088,-0.161220,0.036478,0.144411,-0.058505,-0.011896,0.489989,0.238850,0.052130,-0.250921,-0.148646,0.269080,0.261996,-0.230705,1.110531,0.002790,-0.051045,0.013569,0.070599,-0.027602,-0.108411,0.069238,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
[0.600705,0.842001,0.141378,-0.085560,0.123041,0.098957,-0.124137,-0.104294,0.086479,0.599040,0.127103,-0.068377,0.368680,0.436942,-0.331350,-0.625988,0.415640,1.102010,-0.026006,-0.022046,-0.053543,-0.094078,0.054341,0.158086,-0.099364,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
[0.828262,0.923440,0.156260,0.019688,-0.145150,-0.182312,0.155285,0.259346,-0.198118,0.743930,0.450113,0.283868,-0.596685,-0.793230,0.562075,1.121753,-0.812234,1.063450,-0.115136,-0.058880,0.148996,0.170966,-0.128837,-0.259732,0.189417,0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,0.000000,-0.000000],

]

class Tower(TowerBase):
    def __init__(self, uris, report_socket=None):
        TowerBase.__init__(self, uris, report_socket)
        
        self.predefined_xrefs=[
            [-1,-1,1],
            [+1,-1,1],
            [-1,+1,1],
            [+1,+1,1],            
        ]

        self.wanted :int = None

    def fly(self, wanted):
        self.wanted=wanted
         # Wait for all CF to connect (to avoid race)
        time.sleep(10)

        # self.upload_trajectory_multiple(figure8 ,cf_uris=['radio://0/20/2M/E7E7E7E702'])
        # time.sleep(6)
        # self.upload_trajectory_multiple(generated ,cf_uris=['radio://0/20/2M/E7E7E7E702'])
        # time.sleep(6)
        # self.upload_trajectory_multiple(figure8 ,cf_uris=['radio://0/20/2M/E7E7E7E702'])
        # time.sleep(6)
        # self.upload_trajectory_multiple(generated ,cf_uris=['radio://0/20/2M/E7E7E7E702'])
        # time.sleep(6)
        
        # self.upload_trajectory_multiple(figure8   ,cf_uris=-1)
        # time.sleep(10)
        # self.upload_trajectory_multiple(figure8   ,cf_uris=-1)

        # self.upload_trajectory_multiple(trajs[1],1,cf_uris=['radio://0/40/2M/E7E7E7E704'])
        # self.upload_trajectory_multiple(trajs[1],2,cf_uris=['radio://0/40/2M/E7E7E7E701'])


        while True:
            
            self.safety_check()

            if wanted:
                currently_flying = self.flying_count()
                missing = wanted - currently_flying
                if missing > 0:
                    print("Want", missing, "more copters")
                    self.prepare_copters(missing)
                    self.start_copters(missing, wanted)
            else:
                self.land_all()


            
            # Check if new trajectories need to be calculated
            # time.sleep(3)
            if self.all_flying_copters_waiting_for_trajectories():
                print("All copters are waiting for trajectories")
                self.solve_multiple_MAV()
            
            if self.all_flying_copters_waiting_to_start_trajectories():
                print("All flying copters are waiting to start trajectories")
                self.all_flying_copters_start_trajectory()
            
            self.send_report()

            time.sleep(0.2)

    def safety_check(self):
        for controller in self.controllers:
            if controller.pos_out_of_bounds():
                print("Out of bounds: " + controller.uri)
                controller.safety_land()

    def all_flying_copters_start_trajectory(self):         
        for controller in self.get_flying_controllers():
            controller.set_start_trajectory_param()
        
    def all_flying_copters_waiting_to_start_trajectories(self):
        flying_controllers = self.get_flying_controllers()
        if len(flying_controllers) == 0 :
            return False

        for controller in flying_controllers:
            if not controller.waiting_to_start_trajectory() :
                return False

        return True

    def copters_landing_after_traj(self,flying_controllers=None)->List[int]:
        """Returns a list of copter ids that need to go on the charger after the trajectory is done"""
        if flying_controllers is None:
            flying_controllers = self.get_flying_controllers()
        
        copters_ids_to_go_on_chargers=[]
        for i,controller in enumerate(flying_controllers):
            traj_count=controller.get_trajectory_count()
            print("controller",controller.uri[-2:], "traj count:",traj_count, type(traj_count))
            if int(traj_count)==1:
                copters_ids_to_go_on_chargers.append(i)
        
        return copters_ids_to_go_on_chargers
                

    def solve_multiple_MAV(self):
        flying_controllers = self.get_flying_controllers()
        
        x0s=[]
        for cf in flying_controllers:
            if type(cf.final_position)==type(None):#if final position is not set yet
                pos=[cf.est_x,cf.est_y,cf.est_z]
            else:
                pos=cf.final_position
            
            x0s.append(pos)

        #xrefs
        xrefs=[]
        copters_ids_to_go_on_chargers=self.copters_landing_after_traj(flying_controllers)
        print("copters_ids_to_go_on_chargers: ",[flying_controllers[i].uri[-2:] for i in copters_ids_to_go_on_chargers])

        predef_xrefs=copy.deepcopy(self.predefined_xrefs)
        random.shuffle(predef_xrefs)# shuffle so that we don't always go to the same one

        for i,controller in enumerate(flying_controllers):
            if i in copters_ids_to_go_on_chargers:
                # if id is in copters_ids_to_go_on_chargers, we want to go on charger
                height_above_charger=0.4
                pos=[controller.charging_pad_position[0],controller.charging_pad_position[1],height_above_charger]
                xrefs.append(pos)

            else:
                # otherwise, it needs to go on one of thr predefined xref that is not 
                # the same as the one it is currently on
                
                for j in range(len(predef_xrefs)):
                    dx=np.linalg.norm( np.array(predef_xrefs[j]) - np.array(x0s[i]) )
                    if dx >0.9:
                        pos=predef_xrefs[j]
                        xrefs.append(pos)

                        #delete the used xref
                        del predef_xrefs[j]
                        break
                    elif j==len(predef_xrefs)-1:
                        print("final xref is the same as the final x0 left")
                        xrefs.append(xrefs[0])
                        xrefs[0]=predef_xrefs[j] #TODO: this is a "hack" to make sure we don't use the same xref twice 


        points_to_pad=multi_MAV.N_MAV-len(x0s)
        for i in range(points_to_pad):
            x0s.append([0,0,0])
            xrefs.append([0,0,0])
        
        print("Trying to solve problem with:")
        print("x0s:",x0s)
        print("xrefs:",xrefs)
        print("")
        # input("Press enter to continue")
        trajs=multi_MAV.solve_problem(x0s,xrefs)
        
        for i,cf in enumerate(flying_controllers):
            cf.upload_trajectory(trajs[i])


    def all_flying_copters_waiting_for_trajectories(self):
        flying_controllers:List[TrafficController] = self.get_flying_controllers()
        # print("All flying copters:", [cf.copter_state for cf in flying_controllers])
        # print("All flying copters waiting for trajectories:", [cf.uri for cf in flying_controllers if cf.is_waiting_for_trajectory()])

        if len(flying_controllers) == 0:
            return False
        
        if len(flying_controllers) != self.wanted:
            print("Some flying copters are flying but not as many as wanted to start trajectories")
            return False

        for controller in flying_controllers:
            if not controller.is_waiting_for_trajectory() :
                return False

        return True

    def prepare_copters(self, count):
        prepared_count = 0
        for controller in self.controllers:
            if controller.is_taking_off() or controller.is_ready_for_flight():
                prepared_count += 1

        missing = count - prepared_count
        new_prepared_count = 0
        if missing > 0:
            print("Trying to prepare", missing, "copter(s)")
            best_controllers = self.find_best_controllers()
            print("Best controllers:", [controller.uri for controller in best_controllers])

            for best_controller in best_controllers[:missing]:
                if best_controller:
                    print("Preparing " + best_controller.uri)
                    new_prepared_count += 1
                    best_controller.take_off()
            print("Prepared", new_prepared_count, "copter(s)")

    def start_copters(self, count, total):
        for controller in self.controllers:
            if controller.is_ready_for_flight():
                print("Starting " + controller.uri)
                controller.start_trajectory(0.15, offset_z=0.25)
                   
    def find_unused_slot_times(self, total_slots):
        # Times are measured in trajectory cycles
        start_times = []
        for controller in self.controllers:
            if controller.is_flying():
                start_time = controller.get_traj_cycles()

                # If a is flying but has not updated the start time yes we do
                # not have enough information to calculate empty slots.
                # Return no unused slots for now
                if start_time is None:
                    # print("Start time is unknown, hold back")
                    return []

                start_times.append(start_time)

        # print("Used start times", start_times)
        return self.crunch_slot_times(start_times, total_slots)

    def crunch_slot_times(self, start_times, total_slots):
        # Start times may be multiple cycles ago, remove integer parts
        start_time_fractions = list(map(lambda t: t % 1.0, start_times))

        # Find the average offset
        offsets = list(
            map(lambda t: (t * total_slots) % 1.0, start_time_fractions))
        offset = 0.0
        if len(start_times) > 0:
            offset = statistics.mean(offsets) / total_slots

        adjusted_start_times = list(
            map(lambda t: t - offset, start_time_fractions))

        closest_slots = list(
            map(lambda t: round(t * total_slots), adjusted_start_times))
        unused_slots = list(
            filter(lambda s: s not in closest_slots, range(total_slots)))

        unsued_slot_times = list(
            map(lambda s: offset + s / total_slots, unused_slots))
        return unsued_slot_times
