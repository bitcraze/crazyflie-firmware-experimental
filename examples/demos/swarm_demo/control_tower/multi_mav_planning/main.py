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

try:
    from trajectory_gen.uav_trajectory import Trajectory
    from trajectory_gen.min_snap_traj_gen import TrajectoryGenerator
    from trajectory_gen import traj_utils 
    from optim_problem.test_multiple import * 
    from optim_problem.test_multiple import * 
    from MAV_utils import MAVPlannerDebugger,MAVUtils
except:
    from .trajectory_gen.uav_trajectory import Trajectory
    from .trajectory_gen.min_snap_traj_gen import TrajectoryGenerator
    from .trajectory_gen import traj_utils 
    from .optim_problem.test_multiple import *
    from .MAV_utils import MAVPlannerDebugger,MAVUtils

import copy
from colorama import Fore
import matplotlib.pyplot as plt
import numpy as np
from typing import List

x0s= [[
    [1.00001493, 1.00008743, 1.00000786   ], 
    [-0.99999097, -0.99999993,  1.        ],
    [ 0.99996825, -1.00008681,  0.9999922 ],
    [-0.99999097,  1.00000002,  1.        ],
    ]]

xrefs=[[
    [-0.9824921488761902, 1.1573436260223389, 0.4],
    [-1.0102819204330444, -0.5790976285934448, 0.4], 
    [0.8624248504638672, -0.6220494508743286, 0.4], 
    [0.894738495349884, 1.3935884237289429, 0.4]
    ]]

def generate_trajectories(MAV_sequences:List[List[List[float]]],total_time:float,downsample_step=6)->List[Trajectory]:
    """Generates the trajectories for the MAVs.
    Args: 
        MAV_sequences: List of waypoints of each MAV dimensions (N_MAV,HORIZON_N,XYZ).
        total_time: Total time of the trajectory execution.
        downsample_step: Step of the downsampling.
    Returns:
        trajectories: List of trajectories for each drone.
    """
    trajs=[]
    
    #check shortest distance between MAVs
    dists=calculate_distances(MAV_sequences)

    waypoints_used_for_traj_gen=[]
    for i in range(len(MAV_sequences)):
        waypoints=MAV_sequences[i]

        extra_times_to_add=MAVUtils.find_times_of_closest_distances(dists,MAV_index=i)
        #sort the times to add so that they are in order
        extra_times_to_add.sort()

        #downsample the trajectory but always include the first and last waypoint
        final=waypoints[-1,:]
        #perform the downsampling
        new_waypoints=MAVUtils.downsample_waypoints(waypoints,downsample_step,extra_times_to_add)
        
        waypoints = np.array(new_waypoints)

        if not np.equal(waypoints[-1],final).all() and len(waypoints)<15:
            # waypoints=np.append(waypoints,final,axis=1)
            waypoints=np.vstack((waypoints,final))

        print(Fore.RED,waypoints.shape,Fore.RESET)

        #insert column of zeros at the end(yaw)
        waypoints=np.insert(waypoints,3,0,axis=1)

        # Check if generated trajectory is stable,otherwise, reduce total time
        # dt=0
        # while True:
        #     tr=min_snap_tg.min_snap_traj_generation(waypoints,total_time=total_time-dt)
        #     tr_unstable = tr.is_unstable(should_plot=0)
        #     print("Trajectory {} unstable: {}".format(i,tr_unstable))
             
        #     if( not tr_unstable):
        #         break
        #         # 
        #     dt+=0.25
        #     print("Increasing dt to {}".format(dt))
        
        tr=TrajectoryGenerator.min_snap_traj_generation(waypoints,total_time=total_time)
        trajs.append( tr )
        waypoints_used_for_traj_gen.append(waypoints)
        
    return trajs , np.array(waypoints_used_for_traj_gen)



def log_trajectories(trajs:List[Trajectory],x0s:List[List[float]],xrefs:List[List[float]]):
    """Logs the trajectories to a file."""
    traj_matrices = [trajs[i].get_matrix() for i in range(len(trajs))]

    #deep copy the trajectories
    traj_matrices_copy=copy.deepcopy(traj_matrices)
    #insert first xrefs and then x0s so that at the end it is going to be like this:
    #[x0s,xrefs,traj_matrices_copy]
    traj_matrices_copy.insert(0,xrefs) 
    traj_matrices_copy.insert(0,x0s)   
    np.save('examples/demos/swarm_demo/control_tower/multi_mav_planning/logged_trajs/traj_matrices_{}.npy'.format(log_trajectories.counter),traj_matrices_copy)
    log_trajectories.counter += 1

log_trajectories.counter=0


def solve_problem(x0s:List[List[float]] , xrefs:List[List[float]] ) ->List[np.array] :
    """Solves the multiple MAV problem.
    Args:
        x0s: Initial positions of the MAVs.
        xrefs: Reference positions of the MAVs.
    Returns:
        trajectories: List of trajectory matrices for each drone.
    """

    us = solve_multiple_MAV_problem(x0s, xrefs)

    MAV_sequences = getMAVPaths(us,x0s)
    
    sequences_to_ignore=[]
    for i in range(len(x0s)):
        if np.linalg.norm(np.array(x0s[i])-np.array(xrefs[i]))==0:
            sequences_to_ignore.append(i)

    if (len(sequences_to_ignore)>0):
        print("Sequences to ignore:",sequences_to_ignore)
        #TODO: do not generate trajectories for the sequences to ignore
        MAV_sequences = [MAV_sequences[i] for i in range(len(MAV_sequences)) if i not in sequences_to_ignore]
    
    

    downsample_step=9
    total_time=4.5
    trajs ,waypoints_used_for_gen= generate_trajectories(MAV_sequences,total_time=total_time,downsample_step=downsample_step)

    log_trajectories(trajs,x0s,xrefs)

    if __name__ == '__main__':
        MAVUtils.compare_planned_with_generated(MAV_sequences,trajs,waypoints_used_for_gen,MAVs_to_plot=[1,2])
        pass

    traj_matrices = [trajs[i].get_matrix() for i in range(len(trajs))]
    
    return traj_matrices


if __name__ == "__main__":
    traj_matrices=solve_problem(x0s[-1], xrefs[-1])
    MAVPlannerDebugger.analyse_trajs_from_matrix(traj_matrices)
    plt.show()
