import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

# Build parametric optimizer
# ------------------------------------

ts = 0.1
N = 50
(nu, nx) = (3, 3)

u = cs.SX.sym('u', nu*N)
z0 = cs.SX.sym('z0', nx*2)


(qX, qU, qN, qObs) = (10, 0.1, 400, 200)

(x, y, z) = (z0[0], z0[1], z0[2])
(xref, yref, zref) = (z0[3], z0[4], z0[5])

cost = 0
c = 0
for t in range(0, nu*N, nu):
    cost += qX*((x-xref)**2 + (y-yref)**2 + (z-zref)**2)

    u_t = u[t:t+3]

    cost += qU * cs.dot(u_t, u_t)
    # Dynamics
    x += ts * u_t[0]
    y += ts * u_t[1]
    z += ts * u_t[2]

    # Obstacle cost
    cylinder_radius = 0.2
    c += cs.fmax(0, cylinder_radius**2 - (0.5 - x)**2 - (0.5 - y)**2)

cost += qObs*c
cost += qN*((x-xref)**2 + (y-yref)**2 + (z-zref)**2)

umin = [-1.0] * (nu*N)
umax = [1.0] * (nu*N)
bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(u, z0, cost).with_constraints(bounds)
build_config = og.config.BuildConfiguration()\
    .with_build_directory("my_optimizers")\
    .with_build_mode("debug")\
    .with_tcp_interface_config()


meta = og.config.OptimizerMeta()\
    .with_optimizer_name("navigation")

solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-5)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)
builder.build()
