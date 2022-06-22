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

        
        # self.upload_trajectory_multiple(generated ,cf_uris=-1)
        # time.sleep(10)
        # self.upload_trajectory_multiple(figure8   ,cf_uris=-1)
        # time.sleep(10)
        # self.upload_trajectory_multiple(figure8   ,cf_uris=-1)

        # self.upload_trajectory_multiple(trajs[1],1,cf_uris=['radio://0/40/2M/E7E7E7E704'])
        # self.upload_trajectory_multiple(trajs[1],2,cf_uris=['radio://0/40/2M/E7E7E7E701'])


        while True:
            

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
        
        x0s=[ [cf.est_x,cf.est_y,cf.est_z] for cf in flying_controllers]

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
            else:
                # otherwise, it needs to go on one of thr predefined xref that is not 
                # the same as the one it is currently on
                
                for j in range(len(predef_xrefs)):
                    dx=np.linalg.norm( np.array(predef_xrefs[j]) - np.array(x0s[i]) )
                    if dx >0.5:
                        pos=predef_xrefs[j]
                        #delete the used xref
                        del predef_xrefs[j]
                        break
                
            xrefs.append(pos)

        points_to_pad=2-len(x0s) # 2 is hard coded for now
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
