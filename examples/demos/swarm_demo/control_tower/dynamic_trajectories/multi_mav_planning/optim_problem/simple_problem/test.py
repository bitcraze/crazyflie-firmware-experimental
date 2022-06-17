import stat
import casadi.casadi as cs
import opengen as og
import matplotlib.pyplot as plt
import numpy as np

sampling_time = 0.1
N = 50  # The MPC horizon length
NX = 3  # The number of elements in the state vector=
NU = 3  # The number of elements in the control vector
N_MAV = 1  # The number of MAVs


def dynamics_ct(_x, _u):
    return [_u[0],
            _u[1],
            _u[2]]


def dynamics_dt(x, u):
    dx = dynamics_ct(x, u)
    return [x[i] + sampling_time * dx[i] for i in range(NX)]


# Create a TCP connection manager
mng = og.tcp.OptimizerTcpManager(
    "my_optimizers/navigation")

# Start the TCP server
mng.start()

# Run simulations
x0 = [0, 0, 1]
xref = [1, 0.8, 1.5]
# xref = [-1, 0.5, -0.4]
simulation_steps = 1000

state_sequence = []

solver_status = mng.call([x0[0], x0[1], x0[2], xref[0], xref[1], xref[2]])

print("solver_status: ", dir(solver_status))
us = solver_status['solution']
print("us: ", us)
print("us length:", len(us))

# state_sequence.append(x0)
x = x0
for k in range(N):
    print("===========================================================")
    u = us[3*k:3*k+3]
    print("x_prev:", x)
    print("u:", u)

    x_next = dynamics_dt(x, u)
    print("x_next:", x_next)

    state_sequence.append(x_next)
    x = x_next

state_sequence = np.array(state_sequence)

# Thanks TCP server; we won't be needing you any more
mng.kill()


# ================= PLOTTING STUFF ======================

def data_for_cylinder_along_z(center_x, center_y, radius, height_z):
    z = np.linspace(0, height_z, 20)
    theta = np.linspace(0, 2*np.pi, 20)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid, y_grid, z_grid


time = np.arange(0, sampling_time*N, sampling_time)
plt.subplot(311)
plt.plot(time, state_sequence[:, 0], '-', label="x")
plt.grid()
plt.ylim(-1, 1)
plt.subplot(312)
plt.plot(time, state_sequence[:, 1], '-', label="y")
plt.grid()
plt.ylim(-1, 1)
plt.subplot(313)
plt.plot(time, state_sequence[:, 2], '-', label="z")
plt.grid()
plt.ylim(-1, 1)

# plt.legend(bbox_to_anchor=(0.7, 0.85), loc='upper left', borderaxespad=0.)

plt.figure()
# plot circle
theta = np.linspace(0, 2*np.pi, 300)
x = 0.5 + 0.15*np.cos(theta)
y = 0.5 + 0.15*np.sin(theta)
plt.plot(x, y, 'r-')

plt.plot(state_sequence[:, 0], state_sequence[:, 1])
plt.grid()
plt.xlim(-2, 2)
plt.ylim(-2, 2)

# 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(state_sequence[:, 0], state_sequence[:, 1], state_sequence[:, 2])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(0, 2)

cyl_center = [0.5, 0.5]
cyl_radius = 0.15
cyl_height = 4.0
Xc, Yc, Zc = data_for_cylinder_along_z(
    cyl_center[0], cyl_center[1], cyl_radius, cyl_height)
ax.plot_surface(Xc, Yc, Zc, alpha=0.5)

plt.show()
