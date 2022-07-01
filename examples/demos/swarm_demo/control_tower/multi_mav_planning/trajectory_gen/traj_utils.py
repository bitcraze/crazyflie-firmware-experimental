import time
from typing import List
import numpy as np
from numpy.core.function_base import linspace
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib import animation

try:
    from uav_trajectory import *
except:
    from .uav_trajectory import *


def debug_traj_generation(waypoints:np.ndarray, tr:Trajectory,downsample_step,plt_title=None):
    """
    This function is used to debug the trajectory generation.
    It plots the generated trajectory and the waypoints.
    
    @param waypoints: list of waypoints in shape (waypoints_number, 3)
    @param tr: trajectory to be debugged
    """

    if type(waypoints)!=type(np.array):
        waypoints = np.array(waypoints)

    waypoints_original = waypoints.copy()
    waypoints = waypoints[::downsample_step,:]

    timestep=0.01
    pos,traj_time=tr.get_path(timestep=timestep)
    x,y,z = pos[0],pos[1],pos[2]
    t_traj=traj_time

    x_wps,y_wps,z_wps= waypoints[:,0],waypoints[:,1],waypoints[:,2]

    dur_wps=[ pol.duration for pol in tr.polynomials]

    t_wps=[0]
    for i in range(len(dur_wps)):
        t_wps.append(t_wps[-1]+dur_wps[i])
    

    if len(t_traj)!=len(x):
        delta=len(t_traj)-len(x)
        t_traj=t_traj[:-delta]
    
    if len(t_wps)!=len(x_wps):
        t_wps=t_wps[:-1]

    #plotting 
    t_wps_original = interpolate_time_setpoints(waypoints, waypoints_original, t_wps)    
    
    fig = plt.figure(constrained_layout=True)
    if plt_title is not None:
        fig.suptitle(plt_title)

    USE_ONE_FIGURE=1
    if USE_ONE_FIGURE:
        grid_spec_plot(waypoints_original, x, y, z, t_traj, x_wps, y_wps, z_wps, t_wps,t_wps_original, fig)
    else:
        separate_plots(waypoints_original, x, y, z, t_traj, x_wps, y_wps, z_wps, t_wps, t_wps_original, fig)

def interpolate_time_setpoints(waypoints, waypoints_original, t_wps):
    """
    This function interpolates the time setpoints that correspond to each waypoint that was used to generate teh trajectory.
    As a result,new time setpoints are generated that correspond to the original waypoints.
    
    @param waypoints: list of downsampled waypoints used for trajectory generation in shape (waypoints_number, 3)
    @param waypoints_original: list of original waypoints in shape (original_waypoints_number, 3)
    @param t_wps: list of time setpoints in shape (waypoints_number,)
    
    @return: list of time setpoints in shape (original_waypoints_number,)
    """

    original_shape = waypoints_original.shape
    t_wps_original=[None] * original_shape[0]
    for i in range(original_shape[0]):
        for j in range(len(waypoints)):
            if (waypoints_original[i,:]==waypoints[j,:]).all():
                t_wps_original[i]=t_wps[j]
    
    min_i = 0
    min_v = 0
    for i, v in enumerate(t_wps_original):
       if v is not None:
           t_wps_original[min_i: i + 1] = list(np.linspace(min_v, v, i - min_i + 1))
           min_i = i
           min_v = v
    
    print(t_wps_original)

    return t_wps_original

def grid_spec_plot(waypoints_original, x, y, z, t_traj, x_wps, y_wps, z_wps, t_wps,t_wps_original, fig):
    """
    Plot the generated trajectory and the waypoints in each axis and  3D in one figure.
    """
    
    WAYPOINTS_USED_FOR_GEN_MARKER_SIZE=100
    
    gs = GridSpec(2, 3)

    axX = fig.add_subplot(gs[0, 0])
    axY = fig.add_subplot(gs[0, 1])
    axZ = fig.add_subplot(gs[0, 2])

    ax3D = fig.add_subplot(gs[1,:], projection='3d')
  
    axX.plot(t_traj,x,color="orange",label="generated")
    for ii in range(len(t_wps)):
        axX.annotate("wp_{}".format(ii)  ,(t_wps[ii],x_wps[ii]))
    
    axX.scatter(t_wps,x_wps,color="r",label="waypoints",s=WAYPOINTS_USED_FOR_GEN_MARKER_SIZE)
    axX.scatter(t_wps_original,waypoints_original[:,0],color="g",label="waypoints_original",alpha=0.5)

    axX.set_title('x')
    axX.grid()
    axX.legend()
    
    axY.plot(t_traj,y,color="orange",label="generated")
    for ii in range(len(t_wps)):
        axY.annotate("wp_{}".format(ii)  ,(t_wps[ii],y_wps[ii]))

    axY.scatter(t_wps,y_wps,color="r",label="waypoints",s=WAYPOINTS_USED_FOR_GEN_MARKER_SIZE)
    axY.scatter(t_wps_original,waypoints_original[:,1],color="g",label="waypoints_original",alpha=0.5)

    axY.set_title('y')
    axY.grid()
    axY.legend()

    axZ.plot(t_traj,z,color="orange",label="generated")
    for ii in range(len(t_wps)):
        axZ.annotate("wp_{}".format(ii)  ,(t_wps[ii],z_wps[ii]))
    
    axZ.scatter(t_wps,z_wps,color="r",label="waypoints",s=WAYPOINTS_USED_FOR_GEN_MARKER_SIZE)
    axZ.scatter(t_wps_original,waypoints_original[:,2],color="g",label="waypoints_original",alpha=0.5)

    axZ.set_title('z')
    axZ.grid()
    axZ.legend()
    
    
    ax3D.plot(x,y,z)
    ax3D.scatter(x_wps,y_wps,z_wps,color='r')
    ax3D.scatter(waypoints_original[:,0],waypoints_original[:,1],waypoints_original[:,2],color='green',s=5)
    
    #set limits
    ax3D.set_xlim3d(-1.2,1.2)
    ax3D.set_ylim3d(-1.2,1.2)
    ax3D.set_zlim3d(0.2,1.5)

    ax3D.set_xlabel('x')
    ax3D.set_ylabel('y')
    ax3D.set_zlabel('z')
    ax3D.grid()

def separate_plots(waypoints_original, x, y, z, t_traj, x_wps, y_wps, z_wps, t_wps,t_wps_original, fig3d):
    """
    Plot the generated trajectory and the waypoints in each axis and  3D in separate figures.
    """

    WAYPOINTS_USED_FOR_GEN_MARKER_SIZE=100

    plt.figure()
    plt.suptitle(fig3d.texts[0].get_text())
    plt.subplot(1,3,1)
    plt.plot(t_traj,x,color="orange",label="generated")
    for ii in range(len(t_wps)):
        plt.annotate("wp_{}".format(ii)  ,(t_wps[ii],x_wps[ii]))

    plt.scatter(t_wps,x_wps,color="r",label="waypoints ",s=WAYPOINTS_USED_FOR_GEN_MARKER_SIZE)
    plt.scatter(t_wps_original,waypoints_original[:,0],color="g",label="waypoints_original",alpha=0.5)

    plt.title('x')
    plt.grid()
    plt.legend()

    plt.subplot(1,3,2)
    plt.plot(t_traj,y,color="orange",label="generated")
    for ii in range(len(t_wps)):
        plt.annotate("wp_{}".format(ii)  ,(t_wps[ii],y_wps[ii]))

    plt.scatter(t_wps,y_wps,color="r",label="waypoints",s=WAYPOINTS_USED_FOR_GEN_MARKER_SIZE)
    plt.scatter(t_wps_original,waypoints_original[:,1],color="g",label="waypoints_original",alpha=0.5)

    plt.title('y')
    plt.grid()
    plt.legend()

    plt.subplot(1,3,3)
    plt.plot(t_traj,z,color="orange",label="generated")
    for ii in range(len(t_wps)):
        plt.annotate("wp_{}".format(ii)  ,(t_wps[ii],z_wps[ii]))
    
    plt.scatter(t_wps,z_wps,color="r",label="waypoints",s=WAYPOINTS_USED_FOR_GEN_MARKER_SIZE)
    plt.scatter(t_wps_original,waypoints_original[:,2],color="g",label="waypoints_original",alpha=0.5)

    plt.title('z')
    plt.grid()
    plt.legend()
    
    ax3d=fig3d.add_subplot(111,projection='3d')
    ax3d.plot(x,y,z)
    ax3d.scatter(x_wps,y_wps,z_wps,color='r')
    ax3d.scatter(waypoints_original[:,0],waypoints_original[:,1],waypoints_original[:,2],color='green',s=5)
    
    #set limits
    ax3d.set_xlim3d(-1.2,1.2)
    ax3d.set_ylim3d(-1.2,1.2)
    ax3d.set_zlim3d(0.2,1.5)

    ax3d.set_xlabel('x')
    ax3d.set_ylabel('y')
    ax3d.set_zlabel('z')
    ax3d.grid()
