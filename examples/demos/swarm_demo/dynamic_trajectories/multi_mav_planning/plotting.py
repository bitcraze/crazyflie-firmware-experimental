import matplotlib.pyplot as plt
import numpy as np
from constants import *

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

def plot2D(MAV_sequences):
        plt.figure()
        plt.subplot(211)
        for i in range(N_MAV):
            plt.plot(MAV_sequences[i, :, 0],
                     MAV_sequences[i, :, 1], label="MAV "+str(i+1))
        
        plot_start_and_goal(MAV_sequences)

        plt.xlabel('X')
        plt.ylabel('Y')

        # add legend
        plt.legend()
        plt.grid()
        plt.xlim(-2, 2)
        plt.ylim(-2, 2)

        plt.subplot(212)
        for i in range(N_MAV):
            plt.plot(MAV_sequences[i, :, 2], label="MAV "+str(i+1))

        plt.grid()
        plt.ylabel('Z')
        plt.xlabel('Horizon steps')
        plt.legend()

def plot3D(MAV_sequences): 
    # 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(N_MAV):
        ax.plot(MAV_sequences[i, :, 0],
                MAV_sequences[i, :, 1],
                MAV_sequences[i, :, 2], label='MAV '+str(i+1))
    
    plot_start_and_goal(MAV_sequences,ax)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(0, 2)
    plt.legend()

def plotting(MAV_sequences):
    time = np.arange(0, ts*N, ts)

    plot2D(MAV_sequences)
    plot3D(MAV_sequences)
    plt.show()

def plot_start_and_goal(MAV_sequences,ax=None):
    if ax is None:
        for seq in MAV_sequences:
            plt.plot(seq[0,0],seq[0,1],'o',color='green')
            plt.plot(seq[-1,0],seq[-1,1],'o',color='red')
    else:
        for seq in MAV_sequences:
            ax.plot(seq[0,0],seq[0,1],seq[0,2],'o',color='green')
            ax.plot(seq[-1,0],seq[-1,1],seq[-1,2],'o',color='red')
    