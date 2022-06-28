try:
    from trajectory_gen.uav_trajectory import Trajectory
    from optim_problem.test_multiple import * 
    from trajectory_gen import marios_gen as min_snap_tg
except:
    from .trajectory_gen.uav_trajectory import Trajectory
    from .optim_problem.test_multiple import *
    from .trajectory_gen import marios_gen as min_snap_tg

import copy
import matplotlib.pyplot as plt
import numpy as np
from typing import List

def generate_trajectories(MAV_sequences,total_time):
    """Generates the trajectories for the MAVs."""
    trajs=[]
    
    for i in range(N_MAV):
        waypoints=MAV_sequences[i]

        #downsample the trajectory
        downsample_step=6
        waypoints=waypoints[::downsample_step,:]

        #insert column of zeros at the end
        waypoints=np.insert(waypoints,3,0,axis=1)

        tr=min_snap_tg.min_snap_traj_generation(waypoints,total_time=total_time)
        trajs.append( tr )

    # fig=plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # for i in range(N_MAV):
    #     trajs[i].plot(timestep=0.1,ax=ax,label=str(i))
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # ax.set_title('Generated Trajectories')
    # ax.legend()
    
    return trajs

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

def log_trajectories(traj_matrices:list,x0s,xrefs):
    """Logs the trajectories to a file."""
    #deep copy the trajectories
    traj_matrices_copy=copy.deepcopy(traj_matrices)
    #insert first xrefs and then x0s so that at the endit going to be like this:
    #[x0s,xrefs,traj_matrices_copy]
    traj_matrices_copy.insert(0,xrefs) 
    traj_matrices_copy.insert(0,x0s)   
    np.save('/home/oem/MARIOS/crazyflie-firmware-experimental/examples/demos/swarm_demo/control_tower/multi_mav_planning/logged_trajs/traj_matrices_{}.npy'.format(log_trajectories.counter),traj_matrices_copy)
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
    
    # plotGridSpec(MAV_sequences)


    trajs:List[Trajectory]=generate_trajectories(MAV_sequences,total_time=4)

    # for i in range(N_MAV):
    #     trajs[i].plot(timestep=0.1,ax=ax)
    # plt.show()

    traj_matrices=[]
    for i in range(len(trajs)):
        traj_matrices.append(trajs[i].get_matrix())
    
    log_trajectories(traj_matrices,x0s,xrefs)


    return traj_matrices
# #get date for saving the files
# import datetime
# now = datetime.datetime.now()
# date = now.strftime("%Y-%m-%d-%H-%M-%S")
# print(date)


if __name__ == "__main__":
    # trajs=main(1)

    ms=solve_problem(x0s[-1], xrefs[-1])
    print(ms)