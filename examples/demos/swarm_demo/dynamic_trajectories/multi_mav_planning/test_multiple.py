import casadi.casadi as cs
import opengen as og
import matplotlib.pyplot as plt
import numpy as np
from constants import *
from plotting import *


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


def getMAVPaths(us):
    MAV_sequences = []
    for i in range(N_MAV):  # for each mav
        state_sequence = []
        MAV_offset = i*nx*N
        x = x0s[i]
        for k in range(N):  # for each time step in the horizon
            print("===========================================================")
            u = us[MAV_offset+3*k:MAV_offset+3*k+3]
            print("x_prev:", x)
            print("u:", u)

            x_next = dynamics_dt(x, u)
            print("x_next:", x_next)

            state_sequence.append(x_next)
            x = x_next

        MAV_sequences.append(state_sequence)

    MAV_sequences = np.array(MAV_sequences)

    return MAV_sequences


def solve_multiple_MAV_problem(x0s, xrefs):
    """Solves the multiple MAV path planning problem and returns the velocity control inputs each MAV."""
    # Create a TCP connection manager
    mng = og.tcp.OptimizerTcpManager("my_optimizers/navigation_multiple")

    # Start the TCP server
    mng.start()

    z = concatenate_xs(x0s, xrefs)
    print("z:", z)
    # input("Press Enter to continue...")

    # call the solver with the initial and reference states
    solver_status = mng.call(z)

    us = solver_status['solution']

    print("us: ", us)
    print("us length:", len(us))

    # Thanks TCP server; we won't be needing you any more
    mng.kill()

    return us


if __name__ == "__main__":
    # Run simulations
    x0s = [
        [0, 0, 1],
        [1, 0, 1]
    ]

    xrefs = [
        [1, 1, 1.5],
        [0, 1, 1.5]
    ]

    us = solve_multiple_MAV_problem(x0s, xrefs)

    MAV_sequences = getMAVPaths(us)

    # plotting(MAV_sequences)

    # plotGridSpec(MAV_sequences)

    animate3D(MAV_sequences)