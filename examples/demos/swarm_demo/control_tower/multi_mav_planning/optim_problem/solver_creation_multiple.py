import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
from constants import *
# Build parametric optimizer
# ------------------------------------

# u contains the control inputs (each of size nu) of the MAVS [u0, u1, ..., uN]
u = cs.SX.sym('u', nu*N*N_MAV)

# z0 contains the initial and final states(each of size nx) of the MAVs [x0,x1,x2,...,xN,x_ref0,x_ref1,x_ref2,...,x_refN]
z0 = cs.SX.sym('z0', 2*nx*N_MAV)

x0s = []
xrefs = []
xref_start = nx*N_MAV  # index of the first x_ref
for i in range(N_MAV):
    offset = i*nx  # offset of the current mav

    x0s.append([z0[offset],
                z0[offset+1],
                z0[offset+2]])

    xrefs.append([z0[xref_start+offset],
                  z0[xref_start+offset+1],
                  z0[xref_start+offset+2]])

cost = 0
c = 0
for t in range(0, N):  # for each time step in the horizon

    for i in range(N_MAV):  # for each mav

        # # Intermediate state cost
        # cost += qX*((x0s[i][0] - xrefs[i][0])**2 +
        #             (x0s[i][1] - xrefs[i][1])**2 +
        #             (x0s[i][2] - xrefs[i][2])**2)

        MAV_offset = i*nu*N
        print(MAV_offset)
        curr_t = MAV_offset + nu*t

        u_t = u[curr_t:curr_t+nu]
        # Input Cost
        cost += qU * cs.dot(u_t, u_t)

        # Dynamics
        x0s[i][0] += ts * u[curr_t+0]
        x0s[i][1] += ts * u[curr_t+1]
        x0s[i][2] += ts * u[curr_t+2]

    # Swarm Collision cost
    # h_dist = dMAV - cs.sqrt((x0s[0][0]-x0s[1][0])**2 +
        #  (x0s[0][1]-x0s[1][1])**2 +
        #  (x0s[0][2]-x0s[1][2])**2)

   
    epsilon=0.0000001
    h = 0.3 - cs.sqrt((x0s[0][2]-x0s[1][2]+epsilon)**2 + (x0s[0][0]-x0s[1][0]+epsilon)**2 + (x0s[0][1]-x0s[1][1]+epsilon)**2)

    cost += 10000*cs.fmax(0, h)
    # print("cost:", cost)

for i in range(N_MAV):
    # Termination cost
    cost += qN*((x0s[i][0] - xrefs[i][0])**2 +
                (x0s[i][1] - xrefs[i][1])**2 +
                (x0s[i][2] - xrefs[i][2])**2)


# [umin0_t0, umin0_t1, ..., umin0_tN, umin1_t0, ..., umin1_tN ,..., uminN_t0, ..., uminN_tN]
umin = [-1.0] * (nu*N*N_MAV)
# [umax0_t0, umax0_t1, ..., umax0_tN, umax1_t0, ..., umax1_tN ,..., umaxN_t0, ..., umaxN_tN]
umax = [+1.0] * (nu*N*N_MAV)

# umin[nu*N:]=[0.8]
# umax[nu*N:]=[+2.0]
bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(u, z0, cost).with_constraints(bounds)
build_config = og.config.BuildConfiguration()\
    .with_build_directory("my_optimizers")\
    .with_build_mode("debug")\
    .with_tcp_interface_config()


meta = og.config.OptimizerMeta()\
    .with_optimizer_name("navigation_multiple")

solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-5)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)
builder.build()

from test_multiple import main 
main()