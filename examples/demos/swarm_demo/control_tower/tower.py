# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2022 Bitcraze AB
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
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

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
from predefined_trajs import spiral_traj

SPIRAL_TRAJ_ID=5

class Tower(TowerBase):
    TRAJ_RECEIVE_CONFLICT_TIMEOUT = 4

    def __init__(self, uris, report_socket=None):
        TowerBase.__init__(self, uris, report_socket)
        
        self.predefined_xrefs=[
            [-1,-1,1],
            [+1,-1,1],
            [-1,+1,1],
            [+1,+1,1],            
        ]

        self.wanted :int = None
        self.wanted_original :int = None

        self.traj_receive_conflict=False

    def fly(self, wanted):
        self.wanted_original=wanted
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
            
            self.safety_check()#checks if any copter is out of the safety zone and if so, it lands it

            if wanted:
                currently_flying = self.flying_count()
                missing = wanted - currently_flying
                if missing > 0:
                    # print("Want", missing, "more copters")
                    self.prepare_copters(missing)
                    self.start_copters(missing, wanted)
            else:
                self.land_all()


            
            # Check if new trajectories need to be calculated
            if self.all_flying_copters_waiting_for_trajectories():
                print("All copters are waiting for trajectories")
                self.solve_multiple_MAV()
            
            if self.all_flying_copters_waiting_to_start_trajectories():
                print("All flying copters are waiting to start trajectories")
                self.all_flying_copters_start_trajectory()
            
            #Check if traj_receive_conflict and resolve the conflict by resolving the problem for the current positions
            if self.check_waiting_to_start_traj_conflict():
                print(Fore.RED,"Trajectory receive conflict detected !!!",Fore.RESET)
                #use current positions as start since copters haven't executed the previously uploaded trajectory
                self.solve_multiple_MAV(use_current_positions=True)
            
            self.check_if_too_many_crashed()
            
            self.send_report()

            time.sleep(0.2)

    def check_if_too_many_crashed(self):
        """
            Check if there are flying copers waiting for trajectories and the \
            rest of the copters are crashed. If so, land all the waiting copters.
        """

        copters_crashed = self.get_crashed_copters()
        crashed_count = len(copters_crashed)
        # able_to_fly_count = len(self.get_connected_copters()) #didnt work,there was only 1 connected copter 
        # able_to_fly_count = len(self.controllers)
        able_to_fly=self.get_controllers_able_to_fly()
        able_to_fly_count = len(able_to_fly)

        if able_to_fly_count == 0 and crashed_count == 0:
            print("No copters connected")
            return

        if self.wanted > able_to_fly_count-crashed_count:
            print("Too many crashed copters , landing all waiting copters")
        
        # print("Wanted copters:",self.wanted,"Connected copters:",able_to_fly_count,"Crashed copters:",crashed_count)
        prev_wanted = self.wanted
        self.wanted = min([self.wanted_original , max(able_to_fly_count-crashed_count,1)])
        if prev_wanted != self.wanted:
            print("Wanted copters changed:",self.wanted)
            print("able to fly:",[ c.short_uri for c in able_to_fly] , "crashed:",[ c.short_uri for c in copters_crashed])


    def get_copters_waiting_for_trajectories(self):
        return [controller for controller in self.get_flying_controllers() if controller.copter_state==TrafficController.STATE_WAITING_TO_RECEIVE_TRAJECTORY] 

    def get_crashed_copters(self):
        return [controller for controller in self.controllers if controller.copter_state==TrafficController.STATE_CRASHED]

    def check_waiting_to_start_traj_conflict(self):
        """
        Check if some copters are on state WAITING_FOR_TRAJECTORY while 
        others are on state WAITING_TO_START_TRAJECTORY (usually happens when
        a copter crashes when it has already received the next trajectory)
        
        Returns True if there is at least one copter on state WAITING_TO_RECEIVE_TRAJECTORY
        and one on state WAITING_TO_START_TRAJECTORY and this situation persists for more 
        than a time threshold.
        """

        flying_controllers = self.get_flying_controllers()

        waiting_to_receive_traj_controllers = [c for c in flying_controllers if c.copter_state == TrafficController.STATE_WAITING_TO_RECEIVE_TRAJECTORY]        
        waiting_to_start_traj_controllers   = [c for c in flying_controllers if c.copter_state == TrafficController.STATE_WAITING_TO_START_TRAJECTORY]
        
        #count
        waiting_to_receive_traj_count = len(waiting_to_receive_traj_controllers)
        waiting_to_start_traj_count   = len(waiting_to_start_traj_controllers)

        conflict_exists = waiting_to_receive_traj_count > 0 and waiting_to_start_traj_count > 0

        if  not self.traj_receive_conflict:
            self.traj_receive_conflict=conflict_exists
            self.traj_receive_conflict_start_time=time.time()   

            return False
        else:
            if conflict_exists:
                if time.time() - self.traj_receive_conflict_start_time > Tower.TRAJ_RECEIVE_CONFLICT_TIMEOUT:
                    print("Conflict detected:")
                    print("Waiting to receive traj:", waiting_to_receive_traj_count)
                    print("Waiting to start traj:", waiting_to_start_traj_count)
                    self.traj_receive_conflict=False
                    self.traj_receive_conflict_start_time=time.time()
                    return True
            else:
                self.traj_receive_conflict=False
                return False

    def get_controllers_able_to_fly(self):#TODO: make this check event based whenever a copter changes state
        """
            Returns the number of copters that are able to fly in general,not necessarily now.(not never connected)
        """
        # return [controller for controller in self.controllers if controller.copter_state==TrafficController.CS_CONNECTED]# not working
        return [controller for controller in self.controllers if controller.able_to_fly()]

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
            
            if controller.needs_charging():
                print("controller",controller.uri[-2:],"needs charging ,planning trajectory to go on charger")
                traj_id_to_land_after = 2 if int(controller.latest_trajectory_id)==1 else 1
                #sending the id of the trajectory to land after
                controller.force_land(traj_id_to_land_after)
                print("Forced Landing Signal sent")

            if int(traj_count)==1 or int(traj_count)==0 or controller.needs_charging():
                copters_ids_to_go_on_chargers.append(i)
        
        return copters_ids_to_go_on_chargers
                

    def solve_multiple_MAV(self,use_current_positions=False):
        flying_controllers = self.get_flying_controllers()
        
        # if only flying 1 drone ,use the spiral trajectory
        # no need to solve trajectory for landing since it is only one
        if self.wanted==1:
            controller=flying_controllers[0]
            #not uploading the spiral trajectory since it is over 15 segments and it will mess up 
            #the uploading mechanism,but seeting the specific traj id to define the spiral trajectory
            #the copter itself
            controller._cf.param.set_value('app.curr_traj_id', SPIRAL_TRAJ_ID)
            return

        x0s=[]
        for cf in flying_controllers:
            #if final position is not set yet or if we want to use current positions
            if type(cf.final_position)==type(None) or use_current_positions:
                print("Using current position for:",cf.get_short_uri())
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
                        xrefs[0]=predef_xrefs[j] #this is a way to make sure we don't use the same xref twice 


        points_to_pad=multi_MAV.N_MAV-len(x0s)
        for i in range(points_to_pad):
            x0s.append(  [2,2,2] )  #padding with zeros seemed to crash the solver     
            xrefs.append([2,2,2] )  #padding with zeros seemed to crash the solver
        
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
            uris=[cf.short_uri for cf in flying_controllers]
            print("{} flying copters are flying ({}) but not as many as wanted ({}) to receive trajectories".format(len(flying_controllers),uris,self.wanted))
            print("crashed count :",len(self.get_crashed_copters()))
            return False
        else:
            uris=[cf.short_uri for cf in flying_controllers]
            print("Flying copters ({}) as many as wanted ({}) to receive trajectories".format(uris,self.wanted))
        
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
            # print("Trying to prepare", missing, "copter(s)")
            best_controllers = self.find_best_controllers()
            # print("Best controllers:", [controller.uri for controller in best_controllers])

            for best_controller in best_controllers[:missing]:
                if best_controller:
                    print("Preparing " + best_controller.uri)
                    new_prepared_count += 1
                    best_controller.take_off()
            # print("Prepared", new_prepared_count, "copter(s)")

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

    def reset_crash_states(self):
        print(Fore.RED,"Waiting 5 seconds",Fore.RESET)
        print("Resetting crash states")
        for controller in self.controllers:
            controller.reset_crash_state()

    def safety_land_all_flying(self):
        for controller in self.get_flying_controllers():
            controller.safety_land()
            controller.reset_internal()
