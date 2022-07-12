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

from typing import List
import time
import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import threading

import numpy as np
from cflib.crazyflie.mem.trajectory_memory import TrajectoryMemory
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D


from multi_mav_planning import main as multi_MAV
from colorama import Fore, Back, Style

#CONSTANTS
TRAJECTORY_SEGMENT_SIZE_BYTES = 132

TRAJECTORY_COUNT=3

class TrajectoryUploadConfig():
    def __init__(self) -> None:
        self.trajectory_id = None
        self.pol_segment_count = None
        self.defined=False
        self.should_upload=False

class TrafficController:
    CS_DISCONNECTED = 0
    CS_CONNECTING = 1
    CS_CONNECTED = 2

    STATE_UNKNOWN = -1
    STATE_IDLE = 0
    STATE_WAIT_FOR_POSITION_LOCK = 1
    STATE_WAIT_FOR_TAKE_OFF = 2  # Charging
    STATE_TAKING_OFF = 3
    STATE_HOVERING = 4
    STATE_WAITING_TO_RECEIVE_TRAJECTORY = 5
    STATE_WAITING_TO_START_TRAJECTORY = 6
    STATE_RUNNING_TRAJECTORY = 7
    STATE_GOING_TO_PAD = 8
    STATE_WAITING_AT_PAD = 9
    STATE_LANDING = 10
    STATE_CHECK_CHARGING = 11
    STATE_REPOSITION_ON_PAD = 12
    STATE_CRASHED = 13

    NO_PROGRESS = -1000.0

    PRE_STATE_TIMEOUT = 3

    OUT_OF_BOUNDS_CUBE =(1.6,1.6,1.8)
    
    CHARGED_FOR_FLIGHT_THRESHOLD = 4.0

    LOW_BATTERY_THRESHOLD = 3.2

    def __init__(self, uri):
        self.uri = uri
        self.short_uri= self.get_short_uri()
        self.stay_alive = True
        self.reset_internal()
        self.connection_thread = threading.Thread(target=self.process)
        self.connection_thread.start()
        
        self._traj_upload_done = False
        self._traj_upload_success = False

        self._traj_upload_configs: list[TrajectoryUploadConfig] =[TrajectoryUploadConfig() for i in range(2)]

        self.can_upload_trajectory=False # can upload trajectory if copter is in state STATE_WAITING_TO_RECEIVE_TRAJECTORY (5) or while executing trajectory (7)
        self.latest_trajectory_id=None

        self.trajs_uploaded=0
        self.trajs_uploaded_success=0
        
        self._waiting_to_start_traj=False
        
        self._console_buffer = ""

        self.charging_pad_position = None

        self._trajcount = 255
        self.final_position=None
        self._fully_connected = False

    def pos_out_of_bounds(self):
        return self.est_x>self.OUT_OF_BOUNDS_CUBE[0]\
            or self.est_y<-self.OUT_OF_BOUNDS_CUBE[1]\
            or self.est_z>self.OUT_OF_BOUNDS_CUBE[2]\

    def safety_land(self):
        self._cf.param.set_value('app.safety_land', 1)
    
    def waiting_to_start_trajectory(self):
        return self._waiting_to_start_traj
    
    def set_start_trajectory_param(self):
        self._cf.param.set_value("app.start_traj",1)

    def is_waiting_for_trajectory(self):
        pending_trajs=self.trajs_uploaded-self.trajs_uploaded_success
        expected_states=[self.STATE_WAITING_TO_RECEIVE_TRAJECTORY,self.STATE_RUNNING_TRAJECTORY]
        is_in_valid_state= self.copter_state == expected_states[0] or self.copter_state == expected_states[1]

        stage1 = is_in_valid_state and pending_trajs==0 and self.can_upload_trajectory
        
        if stage1:
            trajcount=self.get_trajectory_count()
            # trajectory_going_to_pad = int(trajcount) == 0 or int(trajcount) == 255
            trajectory_going_to_pad = int(trajcount) == 255
            
            if not trajectory_going_to_pad:
                return True
            else:
                return False
        
        return False
    
    def upload_trajectory(self, trajectory:np.array):
        trajectory_mem = self._cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
        trajectory_mem:TrajectoryMemory

        if len(trajectory) > 15:
            print('Trajectory is too long!')
            raise Exception('Trajectory is too long ({} segments)!'.format(len(trajectory)))

        self.trajectory_mem=trajectory_mem

        #fill trajectory memory with new trajectory
        trajectory_mem.trajectory=[]
        total_duration = 0
        for i,row in enumerate(trajectory):
            duration = row[0]
            x = Poly4D.Poly(row[1:9])
            y = Poly4D.Poly(row[9:17])
            z = Poly4D.Poly(row[17:25])
            yaw = Poly4D.Poly(row[25:33])

            pol=Poly4D(duration, x, y, z, yaw)
            
            trajectory_mem.trajectory.append(pol)

            total_duration += duration
        
        #setting upload config and mechanisms
        print(self.short_uri + "Previous latest_trajectory_id:",self.latest_trajectory_id)

        self.latest_trajectory_id = 2 if int(self.latest_trajectory_id)==1 else 1
        id = self.latest_trajectory_id-1 #0-based indexing


        self._traj_upload_configs[id].trajectory_id=id
        self._traj_upload_configs[id].defined=False
        self._traj_upload_configs[id].should_upload=True
        self._traj_upload_configs[id].pol_segment_count=len(trajectory)
        
        print('Uploading trajectory with id:{} in {}...'.format(self.latest_trajectory_id,self.uri))
        self.trajs_uploaded+=1
        
        #get the final position of the trajectory
        self.final_position=self.get_final_position(trajectory).pos
        print(self.short_uri+"Final position:",self.final_position)
        self.traj_start_time = time.time()
        
        self.upload_trajectory_to_memory(trajectory_mem,self._upload_done,self._upload_failed,self.latest_trajectory_id)

        self._traj_upload_done = False
        self._traj_upload_success = False

        self.can_upload_trajectory=False

    def upload_trajectory_to_memory(self,traj_mem:TrajectoryMemory,upload_done_callback,upload_failed_callback,tr_id):
        """
        Uploads a trajectory to the memory of the copter.A simple mechanism is used to ensure that the trajectory is uploaded in the correct order.
        The whole trajectory memory can handle 31 trajectory segments.So it is splitted in half and the maximum number of segments that can be uploaded at once is 15.
        The trajectory ids used are 1 and 2 and each one have the following trajectory memory offsets:
            1:  0x00
            2:  15* TRAJECTORY_SEGMENT_SIZE_BYTES since the maximum number of segments is 15
                where TRAJECTORY_SEGMENT_SIZE_BYTES is the size of a trajectory segment in bytes
        
        So the trajectory id 1 is uploaded in the first half of the memory and the trajectory id 2 is uploaded in the second half of the memory.
        The first time it is demanded to upload a trajectory it is uploaded with the id 1 
        and the second time it is demanded to upload a trajectory it is uploaded with the id 2 (while the previous one is being executed).

        The copter will execute a trajectory as soon as it is in the state STATE_WAITING_TO_START_TRAJECTORY (6) and the trajectory_id parameter is changed.
        """
        
        if tr_id==1:
            start_addr=0x00
        else:
            start_addr=15*TRAJECTORY_SEGMENT_SIZE_BYTES
            
        traj_mem.write_data(upload_done_callback,write_failed_cb=upload_failed_callback,start_addr=start_addr)

    def get_final_position(self,trajectory:List[List[float]]):
        trajectory=np.array(trajectory)

        matrix=trajectory[-1,:] # last row
        matrix=[matrix]

        tr=multi_MAV.Trajectory()
        tr.load_from_matrix(matrix)
        pos=tr.eval(tr.duration)
        
        return pos

    def is_trajectory_uploaded(self):
        return self._traj_upload_done and self._traj_upload_success

    def _upload_done(self, mem, addr):
        dt=time.time()-self.traj_start_time
        print(Fore.GREEN+'Trajectory upload succesfull to {} after {:0.3f} sec!'.format(self.uri[-2:],dt)+Style.RESET_ALL)

        # self.latest_offset refers to pol segments so multiplication by 132 is needed to get the real offset
        # since each segment is 132 bytes long
        for i,conf in enumerate(self._traj_upload_configs):
            if conf.defined or conf.trajectory_id==None:
                continue
            
            id=conf.trajectory_id+1
            pol_segment_count=conf.pol_segment_count
            print("Defining trajectory with id: {}...".format(id))
            
            offset=0 if id==1 else 15
            self._cf.high_level_commander.define_trajectory(id, offset*TRAJECTORY_SEGMENT_SIZE_BYTES, pol_segment_count)

            conf.defined=True
            
            self._cf.param.set_value('app.curr_traj_id', id)
            print("CF:{} Current trajectory id: {}".format(self.uri[-2:],id))
        
        self._traj_upload_done = True
        self._traj_upload_success = True
        
        self.trajs_uploaded_success+=1
        if(self.trajs_uploaded_success!=self.trajs_uploaded):
            print(Fore.RED+"Trajectories difference: {}".format(self.trajs_uploaded-self.trajs_uploaded_success))
            print(Style.RESET_ALL)

    def _upload_failed(self, mem, addr):
        print(Fore.RED+'Trajectory upload FAILED!')
        print(Style.RESET_ALL)
        
        self._traj_upload_done = True
        self._traj_upload_success = False

    def reset_internal(self):
        self.connection_state = self.CS_DISCONNECTED
        self._cf = None
        self._log_conf = None
        self.copter_state = self.STATE_UNKNOWN
        self.vbat = -1.0
        self._time_for_next_connection_attempt = 0
        self.traj_cycles = None
        self.est_x = 0.0
        self.est_y = 0.0
        self.est_z = 0.0
        self.up_time_ms = 0
        self.flight_time_ms = 0

        # Pre states are used to prevent multiple calls to a copter
        # when waiting for the remote state to change
        self._pre_state_taking_off_end_time = 0
        self._pre_state_going_to_initial_position_end_time = 0

    def _pre_state_taking_off(self):
        return self._pre_state_taking_off_end_time > time.time()

    def _pre_state_going_to_initial_position(self):
        return self._pre_state_going_to_initial_position_end_time > time.time()

    def is_connected(self):
        return self.connection_state == self.CS_CONNECTED

    def has_found_position(self):
        return self.copter_state > self.STATE_WAIT_FOR_POSITION_LOCK

    def is_taking_off(self):
        return self.copter_state == self.STATE_TAKING_OFF or self._pre_state_taking_off()

    def is_ready_for_flight(self):
        # print(self.short_uri + "is_ready_for_flight:",self.copter_state, (not self._pre_state_going_to_initial_position()) )
        return self.copter_state == self.STATE_HOVERING and not self._pre_state_going_to_initial_position()

    def is_flying(self):
        return self.copter_state == self.STATE_RUNNING_TRAJECTORY or \
               self.copter_state == self.STATE_WAITING_TO_RECEIVE_TRAJECTORY or \
               self.copter_state == self.STATE_WAITING_TO_START_TRAJECTORY or \
               self._pre_state_going_to_initial_position()

    def is_landing(self):
        return self.copter_state == self.STATE_GOING_TO_PAD or \
               self.copter_state == self.STATE_WAITING_AT_PAD or \
               self.copter_state == self.STATE_LANDING or \
               self.copter_state == self.STATE_CHECK_CHARGING or \
               self.copter_state == self.STATE_REPOSITION_ON_PAD

    def is_charging(self):
        return self.copter_state == self.STATE_WAIT_FOR_TAKE_OFF and not self._pre_state_taking_off()

    def is_crashed(self):
        return self.copter_state == self.STATE_CRASHED

    def take_off(self):
        if self.is_charging():
            if self._cf:
                self._pre_state_taking_off_end_time = time.time() + self.PRE_STATE_TIMEOUT
                self._cf.param.set_value('app.takeoff', 1)

    def start_trajectory(self, trajectory_delay, offset_x=0.0, offset_y=0.0, offset_z=0.0):
        if self.is_ready_for_flight():
            if self._cf:
                self._cf.param.set_value('app.offsx', offset_x)
                self._cf.param.set_value('app.offsy', offset_y)
                self._cf.param.set_value('app.offsz', offset_z)

                self._pre_state_going_to_initial_position_end_time = time.time() + self.PRE_STATE_TIMEOUT
                self._cf.param.set_value('app.start', trajectory_delay)

    def force_land(self,value=1):
        # if self.connection_state == self.CS_CONNECTED:
        self._cf.param.set_value('app.stop', value)

    def set_trajectory_count(self, count):
        if self.connection_state == self.CS_CONNECTED:
            self._cf.param.set_value('app.trajcount', count)
    
    def get_trajectory_count(self):
        if self.connection_state == self.CS_CONNECTED:
            self._cf.param.request_param_update('app.trajcount')
            return self._cf.param.get_value('app.trajcount')
            
    def get_charge_level(self):
        return self.vbat

    def is_charged_for_flight(self):
        return self.vbat > TrafficController.CHARGED_FOR_FLIGHT_THRESHOLD

    def needs_charging(self):
        return self.vbat < TrafficController.LOW_BATTERY_THRESHOLD

    def get_traj_cycles(self):
        return self.traj_cycles

    def process(self):
        while self.stay_alive:
            if self.connection_state == self.CS_DISCONNECTED:
                if time.time() > self._time_for_next_connection_attempt:
                    self._connect()

            time.sleep(1)
        self._cf.close_link()

    def _connected(self, link_uri):
        self.connection_state = self.CS_CONNECTED
        print('Connected to %s' % link_uri)

    def get_short_uri(self):
        return "CF:{} ".format(self.uri[-2:])

    def _all_updated(self,link_uri):
        """Callback that is called when all parameters have been updated"""
        self._fully_connected = True

        self.set_trajectory_count(TRAJECTORY_COUNT)
        self._setup_logging()
        
        self.latest_trajectory_id = self._cf.param.get_value('app.curr_traj_id')
        print(self.short_uri+'Latest trajectory id: {}'.format(self.latest_trajectory_id))

        # append traj memory
        trajectory_mem = self._cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
        trajectory_mem:TrajectoryMemory
        row=np.zeros(33)
        for i in range(30):
            duration = row[0]
            x = Poly4D.Poly(row[1:9])
            y = Poly4D.Poly(row[9:17])
            z = Poly4D.Poly(row[17:25])
            yaw = Poly4D.Poly(row[25:33])

            pol=Poly4D(duration, x, y, z, yaw)
            
            trajectory_mem.trajectory.append(pol)

    def able_to_fly(self):#TODO: make this check event based whenever a copter changes state
        """
            Returns true if the copter is able to fly in general(is connected).
        """
        is_airborne = self.copter_state > self.STATE_WAIT_FOR_TAKE_OFF and self.copter_state <= self.STATE_CRASHED
        charging_and_ready_for_flight = self.is_charging() and self.is_charged_for_flight()

        return is_airborne or charging_and_ready_for_flight 
        
    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self._set_disconnected(5)

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self._set_disconnected()

    def _set_disconnected(self, hold_back_time=5):
        self.reset_internal()
        self._time_for_next_connection_attempt = time.time() + hold_back_time

    def _connect(self):
        if self.connection_state != self.CS_DISCONNECTED:
            print("Can only connect when disconnected")
            return

        self.connection_state = self.CS_CONNECTING

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.fully_connected.add_callback(self._all_updated)
        self._cf.console.receivedChar.add_callback(self._console_incoming) #print debug messages from Crazyflie
        self._cf.param.add_update_callback(group='app', name='stop', cb=self._force_land_cb)


        print("Connecting to " + self.uri)
        self._cf.open_link(self.uri)

    def _console_incoming(self, console_text):
        # print each message in one line 
        if console_text[-1] != '\n':
            self._console_buffer += console_text
        else:
            self._console_buffer += console_text

            print(Fore.YELLOW+"CF {} DEBUG:".format( self.uri[-2:] ),self._console_buffer, end='')
            print(Style.RESET_ALL)

            self._console_buffer = ""

    def _setup_logging(self):
        # print("Setting up logging")
        self._log_conf = LogConfig(name='Tower', period_in_ms=100)
        self._log_conf.add_variable('app.state', 'uint8_t')
        # self._log_conf.add_variable('app.prgr', 'float')
        self._log_conf.add_variable('app.uptime', 'uint32_t')
        self._log_conf.add_variable('app.flighttime', 'uint32_t')
        self._log_conf.add_variable('pm.vbat', 'float')
        self._log_conf.add_variable('stateEstimate.x', 'float')
        self._log_conf.add_variable('stateEstimate.y', 'float')
        self._log_conf.add_variable('stateEstimate.z', 'float')

        self._cf.log.add_config(self._log_conf)
        self._log_conf.data_received_cb.add_callback(self._log_data)
        self._log_conf.start()

    def _log_data(self, timestamp, data, logconf):
        if(data['app.state'] !=self.copter_state): 
            #copter state has changed            
            print(Fore.BLUE+"Copter {} state changed to {} {} ".format(self.uri[-2:],data['app.state'] , type(data['app.state'])))
            print(Style.RESET_ALL,end="")
            
            if data['app.state']==self.STATE_LANDING:
                self.set_trajectory_count(TRAJECTORY_COUNT)
                self.final_position = None # reset final position so after taking of use the current position

            if data['app.state']==self.STATE_WAITING_TO_RECEIVE_TRAJECTORY or data['app.state']==self.STATE_RUNNING_TRAJECTORY :
                # can_upload_trajectory is set only when copter enters for first time the state 
                print("Copter {} can upload trajectory".format(self.uri[-2:]))
                self.can_upload_trajectory = True
            
            if  data['app.state']==self.STATE_WAITING_TO_START_TRAJECTORY:
                self._waiting_to_start_traj = True
            else:
                self._waiting_to_start_traj = False

            if data['app.state']==self.STATE_CRASHED:
                self._cf.param.set_value("led.bitmask",255)
            elif self.copter_state== self.STATE_CRASHED or self.copter_state==self.STATE_UNKNOWN :# if previous state was crashed or unknown
                self._cf.param.set_value("led.bitmask",0)

        self.copter_state = data['app.state']

        if self.copter_state != self.STATE_WAIT_FOR_TAKE_OFF:
            self._pre_state_taking_off_end_time = 0

        if self.copter_state != self.STATE_HOVERING:
            self._pre_state_going_to_initial_position_end_time = 0
        
        #charging pad position set
        if self.charging_pad_position== None:
            self.charging_pad_position = (data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z'])
            print("Charging pad position set to {}".format(self.charging_pad_position))

        self.vbat = data['pm.vbat']

        self.up_time_ms = data['app.uptime']
        self.flight_time_ms = data['app.flighttime']

        # self.traj_cycles = data['app.prgr']
        # if self.traj_cycles <= self.NO_PROGRESS:
        #     self.traj_cycles = None
        self.traj_cycles=None

        self.est_x = data['stateEstimate.x']
        self.est_y = data['stateEstimate.y']
        self.est_z = data['stateEstimate.z']

    def reset_crash_state(self):
        self._cf.param.add_update_callback("app","reset_crash_state",self._reset_crash_state_callback)
        self._cf.param.set_value('app.reset_crash_state', 1)

    def _reset_crash_state_callback(self, name, value):
        print(self.get_short_uri(),"Reset crash state callback ->new value: {}".format(value))

    def dump(self):
        print("***", self.uri)
        print("  Connection state:", self.connection_state)
        print("  Copter state:", self.copter_state)
        print("  Bat:", self.vbat)
        print("  Up time:", self.up_time_ms / 1000)
        print("  Flight time:", self.flight_time_ms / 1000)
        print("  _pre_state_taking_off:", self._pre_state_taking_off())
        print("  _pre_state_going_to_initial_position:", self._pre_state_going_to_initial_position())

    def terminate(self):
        self.stay_alive = False

    def _force_land_cb(self, name, value):
        print(Fore.CYAN,self.get_short_uri(),"Force land callback ->new value: {}".format(value),Fore.RESET )