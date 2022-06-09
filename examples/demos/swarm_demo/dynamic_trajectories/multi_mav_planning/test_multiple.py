import stat
import casadi.casadi as cs
import opengen as og
import matplotlib.pyplot as plt
import numpy as np
from constants import *
from plotting import *


def dynamics_ct(_x, _u):
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


# Create a TCP connection manager
mng = og.tcp.OptimizerTcpManager("my_optimizers/navigation_multiple")

# Start the TCP server
mng.start()

# Run simulations
x0s = [
    [0, 0, 1],
    [1, 0, 1]
]
xrefs = [
    [1, 1, 1.5],
    [0, 1, 1.5]
]

z = concatenate_xs(x0s, xrefs)
print("z:", z)
input("Press Enter to continue...")
simulation_steps = 1000

state_sequence = []
solver_status = mng.call(z)

us = solver_status['solution']
print("us: ", us)
print("us length:", len(us))

MAV_sequences = []
# state_sequence.append(x0)
for i in range(N_MAV):
    state_sequence = []
    MAV_offset = i*nx*N
    x = x0s[i]
    for k in range(N):
        print("===========================================================")
        u = us[MAV_offset+3*k:MAV_offset+3*k+3]
        print("x_prev:", x)
        print("u:", u)

        x_next = dynamics_dt(x, u)
        print("x_next:", x_next)

        state_sequence.append(x_next)
        x = x_next

    MAV_sequences.append(state_sequence)

# Thanks TCP server; we won't be needing you any more
mng.kill()

MAV_sequences = np.array(MAV_sequences)


# ================= PLOTTING STUFF ======================
time = np.arange(0, ts*N, ts)

plt.figure()

for i in range(N_MAV):
    plt.plot(MAV_sequences[i, :, 0], MAV_sequences[i, :, 1])

plt.grid()
plt.xlim(-2, 2)
plt.ylim(-2, 2)

# 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i in range(N_MAV):
    ax.plot(MAV_sequences[i, :, 0],
            MAV_sequences[i, :, 1],
            MAV_sequences[i, :, 2])

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(0, 2)

# cyl_center = [0.5, 0.5]
# cyl_radius = 0.15
# cyl_height = 4.0
# Xc, Yc, Zc = data_for_cylinder_along_z(
# cyl_center[0], cyl_center[1], cyl_radius, cyl_height)
# ax.plot_surface(Xc, Yc, Zc, alpha=0.5)

plt.show()
