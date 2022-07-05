# try:
try:
    from trajectory_gen.uav_trajectory import Trajectory
    from trajectory_gen import min_snap_traj_gen as min_snap_tg
    from trajectory_gen import traj_utils 
    from optim_problem.test_multiple import * 
    from optim_problem.test_multiple import * 
    from logged_trajs_visualizer import analyse_trajs_from_matrix
    from MAV_utils import find_times_of_closest_distances

except:
    from .trajectory_gen.uav_trajectory import Trajectory
    from .trajectory_gen import min_snap_traj_gen as min_snap_tg
    from .trajectory_gen import traj_utils 
    from .optim_problem.test_multiple import *
    from .logged_trajs_visualizer import analyse_trajs_from_matrix
    from .MAV_utils import find_times_of_closest_distances

# except:
#     from .trajectory_gen.uav_trajectory import Trajectory
#     from .optim_problem.test_multiple import *
#     from .trajectory_gen import min_snap_traj_gen as min_snap_tg
#     from .logged_trajs_visualizer import analyse_trajs_from_matrix

import copy
from turtle import down
from colorama import Fore
import matplotlib.pyplot as plt
import numpy as np
from typing import List

def downsample_waypoints(waypoints:List[List[float]],downsample_step,extra_times_to_add:List[int]):
    new_waypoints=[]
    downsampled_times= range(0,len(waypoints),downsample_step)
    times_added=[]
    for ii in range(len(downsampled_times) ):
        # print("ii: {}".format(downsampled_times[ii]))
        new_waypoints.append(waypoints[downsampled_times[ii],:])
        times_added.append(downsampled_times[ii])
        
        while len(extra_times_to_add)>0  and (downsampled_times[ii]<extra_times_to_add[0] and extra_times_to_add[0]<downsampled_times[ii+1] )  :
            index=extra_times_to_add[0]
            
            # print("Adding extra waypoint at time {}".format(index))
            new_waypoints.append(waypoints[index,:])
            times_added.append(index)
            extra_times_to_add.pop(0)

    # print("Times added: {}".format(times_added))
    return new_waypoints

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
    for i in range(N_MAV):
        waypoints=MAV_sequences[i]

        extra_times_to_add=find_times_of_closest_distances(dists,MAV_index=i)
        #sort the times to add so that they are in order
        extra_times_to_add.sort()

        #downsample the trajectory but always include the first and last waypoint
        final=waypoints[-1,:]
        #perform the downsampling
        new_waypoints=downsample_waypoints(waypoints,downsample_step,extra_times_to_add)
        
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
        
        tr=min_snap_tg.min_snap_traj_generation(waypoints,total_time=total_time)
        trajs.append( tr )
        waypoints_used_for_traj_gen.append(waypoints)
        
    return trajs , np.array(waypoints_used_for_traj_gen)

x0s = [
       [ [0, 0, 1],[0, 1, 1] ],
       [ [0.5, 0, 1],[0.5, 1, 1] ], 
       [ [0.5, 0.5, 0.5],[0.5, 0.5, 1.5] ],
       [ [0, 0, 1],[1, 0, 1] ],
       [ [-1, -1 , 1] , [ 1, 1, 1] ],

   ]

xrefs = [
    [ [1, 1, 1],[1, 0, 1] ],
    [ [0.5, 1, 1.5],[0.5, 0, 1.5] ],
    [ [0.5, 0.5, 1.5],[0.5, 0.5, 0.5] ],
    [ [1, 1, 1.5],[0, 1, 1.5] ],
    [ [1, 1, 1],[-1, -1, 1] ],
]

x0s = [[
    [0.96, 0.97, 1.00],
    [-1.00, -0.96, 1.00],
    [1.00, -0.96, 1.00],
    [-0.96, 0.95, 1.00],
]]

xrefs = [[
    [0.86, -0.96, 0.40],
    [0.98, 1.21,  0.40],
    [-0.78, 1.17,  0.40],
    [-1.03, -0.99, 0.40],
]]

x0s = [[
[-1.00 , -1.00  ,1.00],
[-1.00 , 1.00   ,1.00],
[1.00  , 1.00   ,1.00],
[1.00  ,-1.00   ,1.00],
]]

xrefs = [[
[ -1.00,  1.20,     0.40],
[ 0.82 , -0.73,     0.40],
[ -0.97,  -0.70,    0.40],
[ 0.89 , 1.39,      0.40],
]]

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


def main(case=2)->List[Trajectory]:
    # Run simulations
    us = solve_multiple_MAV_problem(x0s[case], xrefs[case])

    MAV_sequences = getMAVPaths(us,x0s[case])
    
    trajs=generate_trajectories(MAV_sequences,total_time=4)
    
    #PLotting stuff
    # plotting(MAV_sequences)
    # plotGridSpec(MAV_sequences)
    # animate3D(MAV_sequences)
    # plt.show()

    return trajs

def log_trajectories(trajs:List[Trajectory],x0s:List[List[float]],xrefs:List[List[float]]):
    """Logs the trajectories to a file."""
    traj_matrices = [trajs[i].get_matrix() for i in range(len(trajs))]

    #deep copy the trajectories
    traj_matrices_copy=copy.deepcopy(traj_matrices)
    #insert first xrefs and then x0s so that at the end it is going to be like this:
    #[x0s,xrefs,traj_matrices_copy]
    traj_matrices_copy.insert(0,xrefs) 
    traj_matrices_copy.insert(0,x0s)   
    np.save('/home/oem/MARIOS/crazyflie-firmware-experimental/examples/demos/swarm_demo/control_tower/multi_mav_planning/logged_trajs/traj_matrices_{}.npy'.format(log_trajectories.counter),traj_matrices_copy)
    log_trajectories.counter += 1

log_trajectories.counter=0

def compare_planned_with_generated(MAV_sequences:List[List[List[float]]], trajs:List[Trajectory],waypoints_used_for_gen:List[List[List[float]]],MAVs_to_plot:List[int]=[0,1]):
    """Plots the generated trajectories and the waypoints of the planning in order to compare and debug."""
    
    for i,tr in enumerate(trajs):
        if i in MAVs_to_plot:
            waypoints=MAV_sequences[i]
            min_snap_tg.debug_traj_generation(waypoints,tr,waypoints_used_for_gen[i],plt_title='Drone {}'.format(i))


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
    
    downsample_step=9
    total_time=4.5
    trajs ,waypoints_used_for_gen= generate_trajectories(MAV_sequences,total_time=total_time,downsample_step=downsample_step)

    log_trajectories(trajs,x0s,xrefs)

    if __name__ == '__main__':
        compare_planned_with_generated(MAV_sequences,trajs,waypoints_used_for_gen,MAVs_to_plot=[1,2])
        pass

    traj_matrices = [trajs[i].get_matrix() for i in range(len(trajs))]
    
    return traj_matrices


if __name__ == "__main__":
    # trajs=main(1)

    traj_matrices=solve_problem(x0s[-1], xrefs[-1])
    analyse_trajs_from_matrix(traj_matrices)
    plt.show()
