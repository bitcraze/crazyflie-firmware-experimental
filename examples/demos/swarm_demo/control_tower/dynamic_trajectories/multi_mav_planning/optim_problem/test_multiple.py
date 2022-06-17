import casadi.casadi as cs
import opengen as og
import matplotlib.pyplot as plt
import numpy as np
try:
    from constants import *
    from plotting import *
except:
    from .constants import *
    from .plotting import *

import trajectory_gen.marios_gen as min_snap_tg

def dynamics_ct(_x, _u):
    """
    Returns the dx dynamics of the system given the current state x and the
    control input u.
    """
    return [_u[0],
            _u[1],
            _u[2]]


def dynamics_dt(x, u):
    dx = dynamics_ct(x, u)
    return [x[i] + ts * dx[i] for i in range(nx)]


def concatenate_xs(x0s, xrefs):
    z = []
    for i in range(N_MAV):
        z.append(x0s[i])
    for i in range(N_MAV):
        z.append(xrefs[i])

    return np.concatenate(z)


def getMAVPaths(us,x0s):
    MAV_sequences = []
    for i in range(N_MAV):  # for each mav
        state_sequence = []
        MAV_offset = i*nx*N
        x = x0s[i]
        for k in range(N):  # for each time step in the horizon
            # print("===========================================================")
            u = us[MAV_offset+3*k:MAV_offset+3*k+3]
            # print("x_prev:", x)
            # print("u:", u)

            x_next = dynamics_dt(x, u)
            # print("x_next:", x_next)

            state_sequence.append(x_next)
            x = x_next

        MAV_sequences.append(state_sequence)

    MAV_sequences = np.array(MAV_sequences)

    return MAV_sequences


def solve_multiple_MAV_problem(x0s, xrefs):
    """Solves the multiple MAV path planning problem and returns the velocity control inputs each MAV."""
    # Create a TCP connection manager
    try:
        mng = og.tcp.OptimizerTcpManager("my_optimizers/navigation_multiple")
    except:
        print("Could not create a TCP connection manager with short path")
        path="/home/oem/MARIOS/crazyflie-firmware-experimental/examples/demos/swarm_demo/dynamic_trajectories/multi_mav_planning/my_optimizers/navigation_multiple"
        mng = og.tcp.OptimizerTcpManager(path)

    # Start the TCP server
    mng.start()

    z = concatenate_xs(x0s, xrefs)
    # print("z:", z)

    # call the solver with the initial and reference states
    solver_status = mng.call(z)
    
    print("solver_status:", solver_status["exit_status"])
    # print("solver_status:", solver_status["max_constraint_violation"])
    us = solver_status['solution']

    # print("us: ", us)
    # print("us length:", len(us))

    # Thanks TCP server; we won't be needing you any more
    mng.kill()

    return us

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

    fig=plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for i in range(N_MAV):
        trajs[i].plot(timestep=0.1,ax=ax)
    
    return trajs


def main_solver(case=2):
    # Run simulations
    us = solve_multiple_MAV_problem(x0s[case], xrefs[case])

    MAV_sequences = getMAVPaths(us,x0s[case])
    
    # generate_trajectories(MAV_sequences)
    # plotting(MAV_sequences)

    plotGridSpec(MAV_sequences)

    # animate3D(MAV_sequences)

    plt.show()

x0s = [
       [ [0, 0, 1],[0, 1, 1] ],
       [ [0.5, 0, 1],[0.5, 1, 1] ], 
       [ [0.5, 0.5, 0.5],[0.5, 0.5, 1.5] ],
       [ [0 , 0, 1],[1, 0, 1] ],
       [ [-1, -1 , 1] , [ 1, 1, 1] ],
   ]

xrefs = [
    [ [1, 1, 1],[1, 0, 1] ],
    [ [0.5, 1, 1.5],[0.5, 0, 1.5] ],
    [ [0.5, 0.5, 1.5],[0.5, 0.5, 0.5] ],
    [ [1, 1, 1.5],[0, 1, 1.5] ],
    [ [1, 1, 1],[-1, -1, 1] ],
]

if __name__ == "__main__":
    # for i in range(len(x0s)):
        # main_solver(i)
    main_solver(-1)