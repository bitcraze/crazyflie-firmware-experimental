import threading
import time
from collections import namedtuple
from queue import Queue
from typing import List
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm


uris = [
    'radio://0/20/2M/E7E7E7E700',
    # 'radio://0/20/2M/E7E7E7E701', nope
    'radio://0/20/2M/E7E7E7E702',
    'radio://0/20/2M/E7E7E7E704',
    'radio://0/20/2M/E7E7E7E706',
    'radio://0/20/2M/E7E7E7E707',
    'radio://0/20/2M/E7E7E7E709'
]

class SimpleInterface:
    def __init__(self, uri,param_value):
        self.param_value = param_value
        self.stay_alive=True
        self.is_connected = None
        self.uri = uri
        self.connection_thread = threading.Thread(target=self.process)
        self.connection_thread.start()
        
        self.conn_timeout = time.time() + 10

    def process(self):        
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.fully_connected.add_callback(self._fully_connected)
        print("Connecting to " + self.uri)
        self.cf.open_link(self.uri)

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False
        self.stay_alive = False
        
    def terminate_thread(self):
        self.stay_alive = False
        self.connection_thread.join()
        self.cf.close_link()
        print("Connection thread terminated")

    def _fully_connected(self, link_uri):
        print('Connected to %s' % link_uri)
        param,value=self.param_value
        self.cf.param.set_value(param,value)
        
        self.is_connected = True
        # self.cf.param.set_value('app.safety_land', '1')
        self.stay_alive = False
    
class Handler:
    def __init__(self,uris,param_value_pair) -> None:
        self.cfs :List[SimpleInterface]= []
        self.uris = uris
        for uri in uris:
            self.cfs.append(SimpleInterface(uri,param_value_pair))
    