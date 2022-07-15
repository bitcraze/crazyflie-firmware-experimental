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

import logging
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


def getMAVPaths(us, x0s):
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
        path = "my_optimizers/navigation_multiple"
        mng = og.tcp.OptimizerTcpManager(path)
        
    except:
        try:
            logging.debug("Could not create a TCP connection manager with short path,trying long path")
            path = "control_tower/multi_mav_planning/my_optimizers/navigation_multiple"
            mng = og.tcp.OptimizerTcpManager(path)
        except:
            logging.debug("Could not create a TCP connection manager with short path,trying longer path")
            path = "examples/demos/swarm_demo/control_tower/multi_mav_planning/my_optimizers/navigation_multiple"
            mng = og.tcp.OptimizerTcpManager(path)

    # Start the TCP server
    mng.start()

    z = concatenate_xs(x0s, xrefs)
    # print("z:", z)

    # call the solver with the initial and reference states
    solver_status = mng.call(z)

    logging.debug("solver_status: "+ solver_status["exit_status"] +" in {:.2f} msec".format(solver_status["solve_time_ms"]))
    # logging.debug("solver_status:", solver_status["max_constraint_violation"])
    us = solver_status['solution']

    # print("us: ", us)
    # print("us length:", len(us))

    # Thanks TCP server; we won't be needing you any more
    mng.kill()

    return us


def calculate_distances(MAV_sequences):
    """Finds the shortest distance between MAVs
        MAV_sequences: list of MAV paths 
           dimensions: [N_MAV, N_timesteps, 3]    
        
        returns: array of distances between MAVs 
              dimensions: [N_MAV, N_MAV,N_timesteps]
        
        e.g: (i,j,k) = distance between MAV i and MAV j at "time" k
    """
    if len(MAV_sequences)==1:
        return []

    if type(MAV_sequences) is not np.ndarray:
        MAV_sequences = np.array(MAV_sequences)

    logging.debug("MAV_sequences.shape: {}".format(MAV_sequences.shape) )
    try:
        ticks=MAV_sequences.shape[1]
    except:
        ticks=len(MAV_sequences[0])

    dist=np.ones((len(MAV_sequences),len(MAV_sequences),ticks))
    
    for i in range(len(MAV_sequences)):
        for j in range(len(MAV_sequences)):
            if i==j:
                continue
            for k in range(ticks):
                if k>=len(MAV_sequences[i]) or k>=len(MAV_sequences[j]):
                    continue
                dist[i,j,k]=np.linalg.norm(MAV_sequences[i][k]-MAV_sequences[j][k])
    
    logging.debug("min distance: {}".format(min(dist.flatten())) )

    return dist

def plot_distance_in_MAV_pairs(dists,MAV_pairs,ax=None,downsample_step=1,label=None):
    should_show_plot = True if ax==None else False
    print("should_show_plot:", should_show_plot)
    if ax==None:
        fig=plt.figure()
        ax=fig.add_subplot(111)
        ax.set_title("Distance between MAVs {} and {}".format(MAV_pairs[0],MAV_pairs[1]))
    
    ax.set_xlabel("ticks")
    ax.set_ylabel("distance")
    ticks=np.arange(len(dists[0][0]))*  N/len(dists[0][0])

    ax.plot(ticks,dists[MAV_pairs[0],MAV_pairs[1],:],label=label)

    ax.grid()

    if should_show_plot:
        plt.show()

def main_solver(case=-1):
    # Run simulations
    us = solve_multiple_MAV_problem(x0s[case], xrefs[case])
    print("x0s:", x0s[case])
    print("xrefs:", xrefs[case])

    MAV_sequences = getMAVPaths(us, x0s[case])

    dists =calculate_distances(MAV_sequences)
    print("Shortest distance:", min(dists.flatten()))
    
    MAV_dist_pairs=[1,3]
    plot_distance_in_MAV_pairs(dists,MAV_dist_pairs)

    # generate_trajectories(MAV_sequences)
    # plotting(MAV_sequences)

    plotGridSpec(MAV_sequences)

    # animate3D(MAV_sequences)

    plt.show()


x0s = [
    [[0, 0, 1], [0, 1, 1]],
    [[0.5, 0, 1], [0.5, 1, 1]],
    [[0.5, 0.5, 0.5], [0.5, 0.5, 1.5]],
    [[0, 0, 1], [1, 0, 1]],
    [[-1, -1, 1], [-1, 1, 1], [1, -1, 1], [1, 1, 1]],
    [[1, -1, 1], [-1, -1, 1], [1, 1, 1], [-1, 1, 1]],
]

xrefs = [
    [[1, 1, 1], [1, 0, 1]],
    [[0.5, 1, 1.5], [0.5, 0, 1.5]],
    [[0.5, 0.5, 1.5], [0.5, 0.5, 0.5]],
    [[1, 1, 1.5], [0, 1, 1.5]],
    [[1, 1, 1], [1, -1, 1], [-1, 1, 1], [-1, -1, 1]],
    [[-1, 1, 1], [1, -1, 1], [-1, -1, 1], [1, 1, 1]],
]

x0s = [
    [
        [0.91, -0.95, 1.00],
        [-0.96, -0.97, 1.00],
        [0.97, 0.92, 1.00],
        [-0.92, 1.00, 1.00], ]

]

xrefs = [
    [
        [-0.99, 0.96, 1.00],
        [0.96, -1.01, 1.00],
        [-0.95, -0.95, 1.00],
        [0.96, 1.00, 1.00],
    ]

]

x0s = [[
    [0.96, 0.97, 1.00],
    [-1.00, -0.96, 1.00],
    [-0.96, 0.95, 1.00],
    [1.00, -0.96, 1.00],

]]

xrefs = [[
    [0.86, -0.96, 0.40],
    [0.98, 1.21,  0.40],
    [-1.03, -0.99, 0.40],
    [-0.78, 1.17,  0.40],
]]

x0s= [[
    [-0.75389974,  1.39870884,  0.40000269], 
    [-1.22628488, -0.51857905,  0.40000269], 
    [0, 0, 10], 
    [0, 0, 10],
    ]]

xrefs = [[
    [1, 1, 1], 
    [-1, 1, 1], 
    [0, 0, 10], 
    [0, 0, 10],
    ]]

x0s= [[
    [0.7863444089889526, -0.4456811547279358, 0.26416492462158203], 
    [0.9756213426589966, -1.0080533027648926, 1.0074353218078613], 
    [-0.9979766607284546, -1.005533218383789, 1.0134520530700684], 
    [2,2,2]
    ]]

xrefs=[[
        [1, 1, 1],
        [0.26890477538108826, 1.4520509243011475, 0.4],
        [-1.0447402000427246, 0.5452059507369995, 0.4], 
        [2,2,2]
    ]]
    
if __name__ == "__main__":
    # for i in range(len(x0s)):
    # main_solver(i)
    main_solver(-1)
