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
from colorama import Fore
import numpy as np 
import matplotlib.pyplot as plt
try:
    from trajectory_gen import uav_trajectory
    from optim_problem import plotting
    from MAV_utils import MAVPlannerDebugger
except:
    from .trajectory_gen import uav_trajectory
    from .optim_problem import plotting
    from .MAV_utils import MAVPlannerDebugger

def analyse_trajs_from_file(filename="traj_matrices_0.npy"):
    path="/home/oem/MARIOS/crazyflie-firmware-experimental/examples/demos/swarm_demo/control_tower/multi_mav_planning/logged_trajs/"
    trajs_mat=np.load(path+filename, allow_pickle = True)
    MAVPlannerDebugger.analyse_trajs_from_matrix(trajs_mat)

if __name__=="__main__":
    # traj_numbers=range(6)
    traj_numbers=[5]
    for i in traj_numbers:
        filename="traj_matrices_{}.npy".format(i)
        analyse_trajs_from_file(filename)
        
        plt.show()