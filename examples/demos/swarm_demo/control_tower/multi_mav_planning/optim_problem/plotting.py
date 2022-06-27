import matplotlib.pyplot as plt
import numpy as np
try:
    from constants import *
except:
    from .constants import *
    
from matplotlib.gridspec import GridSpec
from matplotlib import animation

def plot_circle2D(center, radius, n_points=100):
    theta = np.linspace(0, 2*np.pi, n_points)
    x = center[0] + radius*np.cos(theta)
    y = center[1] + radius*np.sin(theta)
    return x, y


def data_for_cylinder_along_z(center_x, center_y, radius, height_z):
    z = np.linspace(0, height_z, 20)
    theta = np.linspace(0, 2*np.pi, 20)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid, y_grid, z_grid


def plot2D(MAV_sequences, ax2D=None, axZ=None):
    print("ax2D:", ax2D)
    print("axZ:", axZ)

    # 2D plot
    for i in range(N_MAV):
        ax2D.plot(MAV_sequences[i, :, 0],
                  MAV_sequences[i, :, 1], label="MAV "+str(i+1))

    plot_start_and_goal(MAV_sequences, ax2D)

    ax2D.set_xlabel('X')
    ax2D.set_ylabel('Y')

    # add legend
    ax2D.legend()
    ax2D.grid()
    ax2D.set_xlim(-2, 2)
    ax2D.set_ylim(-2, 2)

    # z plot
    for i in range(N_MAV):
        axZ.plot(MAV_sequences[i, :, 2], label="MAV "+str(i+1))

    axZ.grid()
    axZ.set_ylabel('Z')
    axZ.set_xlabel('Horizon steps')
    axZ.legend()


def plot3D(MAV_sequences, ax3D=None):

    # 3D plot
    for i in range(N_MAV):
        ax3D.plot(MAV_sequences[i, :, 0],
                  MAV_sequences[i, :, 1],
                  MAV_sequences[i, :, 2], label='MAV '+str(i+1))

    plot_start_and_goal(MAV_sequences, ax3D)

    ax3D.set_xlabel('X')
    ax3D.set_ylabel('Y')
    ax3D.set_zlabel('Z')
    ax3D.set_xlim(-1, 1)
    ax3D.set_ylim(-1, 1)
    ax3D.set_zlim(0, 2)
    ax3D.legend()


def plotting(MAV_sequences):
    time = np.arange(0, ts*N, ts)
    fig1 = plt.figure(constrained_layout=True)
    ax2D = fig1.add_subplot(2, 1, 1)
    axZ = fig1.add_subplot(2, 1, 2)

    fig2 = plt.figure(constrained_layout=True)
    ax3D = fig2.add_subplot(1, 1, 1, projection='3d')

    plot2D(MAV_sequences, ax2D, axZ)
    plot3D(MAV_sequences, ax3D)


def plot_start_and_goal(MAV_sequences, ax: plt.Axes = None):
    if ax.name != "3d":
        for seq in MAV_sequences:
            ax.plot(seq[0, 0], seq[0, 1], 'o', color='green')
            ax.plot(seq[-1, 0], seq[-1, 1], 'o', color='red')
    else:
        for seq in MAV_sequences:
            ax.plot(seq[0, 0], seq[0, 1], seq[0, 2], 'o', color='green')
            ax.plot(seq[-1, 0], seq[-1, 1], seq[-1, 2], 'o', color='red')


def plotGridSpec(MAV_sequences):
    fig = plt.figure(constrained_layout=True)

    gs = GridSpec(2, 2)

    ax2D = fig.add_subplot(gs[0, 0])
    axZ = fig.add_subplot(gs[1, 0])
    ax3D = fig.add_subplot(gs[0, 1], projection='3d')

    ax3D_anim=fig.add_subplot(gs[1, 1], projection='3d')

    # 2D plot
    plot2D(MAV_sequences, ax2D, axZ)

    # 3D plot
    plot3D(MAV_sequences, ax3D)

    #Animation 3D
    animate3D(MAV_sequences, ax3D_anim,fig)
    # plt.show()

def animate3D(MAV_sequences,ax=None,fig=None):
    if ax is None and fig is None:
        fig = plt.figure(constrained_layout=True)
        ax = fig.add_subplot(1, 1, 1, projection='3d')
    
    lines=[]
    for i in range(len(MAV_sequences)):
        line, = ax.plot(MAV_sequences[i,0:1,0], MAV_sequences[i,0:1,1], MAV_sequences[i,0:1,2])
        lines.append(line)

    plot_start_and_goal(MAV_sequences, ax)

    ax.set_xlim(-1.1,1.1)
    ax.set_ylim(-1.1,1.1)
    ax.set_zlim(0.5,1.5)
    
    def update(num, MAV_sequences, lines):
        if num<2:
            num=2
        
        for i in range(len(MAV_sequences)):

            lines[i].set_data(MAV_sequences[i,:num,0], MAV_sequences[i,:num,1])
            lines[i].set_3d_properties(MAV_sequences[i,:num,2])
    
    frames=N
    fps=60
    interval=1000/fps
    ani = animation.FuncAnimation(fig, update, frames, fargs=(MAV_sequences, lines), interval=interval, blit=False)
    plt.show()

if __name__ == "__main__":
    plotGridSpec([])
