# Useful link :https://realpython.com/linear-programming-python/
import time
from typing import List
import numpy as np
from numpy.core.function_base import linspace
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.gridspec import GridSpec
from matplotlib import animation

try:
    from uav_trajectory import *
except:
    from .uav_trajectory import *

#################################POL GENERATOR OVERVIEW#######################################
"""
A 7th rank polynomial is used (t^7)

First wp(waipont) conditions:
    x(0) = waypoint(0)
    x'(0) = x''(0)=x'''(0)= 0

i-th waypoint conditions:
    x_i-1(ti)    =  waypoint(i) 
    x_i-1'(ti)   =  x_i'(ti)
    x_i-1''(ti)  =  x_i''(ti)
    x_i-1'''(ti) =  x_i'''(ti)
    x_i-1(4)(ti) =  x_i(4)(ti)
    x_i-1(5)(ti) =  x_i(5)(ti)
    x_i-1(6)(ti) =  x_i(6)(ti)
    (in each block the position constraints are at the end 2 rows of the block
        Block Format(per row):
        x_i-1'(ti)   -  x_i'(ti)    = 0
        x_i-1''(ti)  -  x_i''(ti)   = 0
        x_i-1'''(ti) -  x_i'''(ti)  = 0
        x_i-1(4)(ti) -  x_i(4)(ti)  = 0
        x_i-1(5)(ti) -  x_i(5)(ti)  = 0
        x_i-1(6)(ti) -  x_i(6)(ti)  = 0
        x_i-1(ti)                   =  waypoint(i) 
        x_i(ti)                     =  waypoint(i) 
    )

Last wp(waipont) conditions:
    x(-1) = waypoint(-1)
    x'(-1) = x''(-1)=x'''(-1)= 0 

Each polynomial is expressed in the matrix with the following format:
    c0*t^0 + c1*t^1 + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5 + c6*t^6 + c7*t^
    where c0 is the constant term, c1 is the first term, c2 is the second term, etc.
"""
##############################################################################################


def calculate_trajectory1D(waypoints:List[Point_time], wp_type=Waypoint.WP_TYPE_X):
    """
    waypoints: list of Point_Time

    wp_type: specifies the type of waypoint (x,y,z or yaw)

    returns: 
        piece_pols: list (of Polynomial)

        total pol:  PiecewisePolynomial

    """
    # If m is the number of waypoints, n is the number of polynomials
    m = len(waypoints)
    n = m - 1
    # print("m:", m, "n:", n)
    A = np.zeros((8*n, 8*n),dtype=np.float64)
    b = np.zeros((8*n, 1),dtype=np.float64)
    # print("A.shape:", A.shape)
    # print("b.shape:", b.shape)

    time_points = []
    prev_t = 0
    for i, traj_point in enumerate(waypoints):
        # if (wp_type == Waypoint.WP_TYPE_X):
            # print("i:", i, "waypoint:", traj_point.wp.get_pos(),traj_point.t)

        traj_point: Point_time

        wp = traj_point.wp.getType(wp_type)
        t = traj_point.t-prev_t
        if i != 0:
            time_points.append(t)

        pol = Polynomial([1, 1, 1, 1, 1, 1, 1, 1])

        if (i == 0 or i == n):  # start/end constraints

            for j in range(0, 4):
                arr = np.array(pol.pol_coeffs_at_t(t))

                # padding with zeros
                arr = np.pad(arr, (8-len(arr), 0), 'constant')
                if i == 0:
                    A[j, 8*i:8*(i+1)] = arr
                else:
                    ind = -(4-j)
                    if ind >= 0:
                        continue

                    A[ind, 8*(i-1):8*(i)] = arr
                pol = pol.derivative()

            tmp = np.array([wp, 0, 0, 0]).reshape((4, 1))

            if i == 0:
                b[0:4] = tmp
            else:
                b[-4:] = tmp

        else:  # continuity constraints

            array_to_add_prev = np.zeros((8, 8))
            for j in range(0, 8):
                vec = np.array(pol.pol_coeffs_at_t(t))

                # padding with zeros
                vec = np.pad(vec, (8-len(vec), 0), 'constant')
                array_to_add_prev[j, :] = vec
                pol = pol.derivative()

            # TODO: Make this a separate function
            pol = Polynomial([1, 1, 1, 1, 1, 1, 1, 1])
            array_to_add_next = np.zeros((8, 8))
            for j in range(0, 8):
                # t=0 because it is the start of the next polynomial
                vec = np.array(pol.pol_coeffs_at_t(t=0))

                # padding with zeros
                vec = np.pad(vec, (8-len(vec), 0), 'constant')
                array_to_add_next[j, :] = vec
                pol = pol.derivative()

            # print("array_to_add_prev:", array_to_add_prev)
            # print("array_to_add_next:", array_to_add_next)

            startl = 4+(i-1)*8  # start line index
            endl = 4+(i-1)*8 + 6   # end line index
            # conitnuity constraints
            A[startl:endl, 8*(i-1):8*(i)] = array_to_add_prev[1:7, :]
            A[startl:endl, 8*(i):8*(i+1)] = -array_to_add_next[1:7, :]

            b[startl:endl] = np.zeros((6, 1))

            # waypoints constraints
            A[endl,  8*(i-1):8*(i)] = array_to_add_prev[0, :]
            A[endl+1, 8*(i):8*(i+1)] = array_to_add_next[0, :]

            b[endl] = wp
            b[endl+1] = wp

        # copy the time
        prev_t = traj_point.t

    # if (wp_type == Waypoint.WP_TYPE_X):
        # print("det(A):", np.linalg.det(A))
        # np.savetxt("A.csv", A, delimiter=",")
        # np.savetxt("b.csv", b, delimiter=",")

    polynomials_coefficients = np.linalg.solve(a=A, b=b)

    # print("polynomials_coefficients.shape:", polynomials_coefficients.shape)

    piece_pols:List[Polynomial] = []  # piecewise polynomials
    for i in range(n):
        p = polynomials_coefficients[8*i:8*(i+1)]
        piece_pols.append(Polynomial(p))

    # tests
    DEBUG = 0
    if DEBUG:
        for i, wp in enumerate(waypoints):
            t = wp.t
            print("i:", i)
            if i >= len(waypoints)-2:
                continue

            if wp_type != Waypoint.WP_TYPE_X:
                break
            if i == 0:
                print(f"pos at t={t} and pol={i}  -->{piece_pols[i].eval(t)}")
                print(
                    f"vel at t={t} and pol={i}-->{piece_pols[i+0].derivative().eval(t)}")
                print(
                    f"accel at t={t} and pol={i}-->{piece_pols[i+0].derivative().derivative().eval(t)}")

                t = waypoints[i+1].t
                print(f"pos at t={t} and pol={i}  -->{piece_pols[i].eval(t)}")
                print(
                    f"pos at t={t} and pol={i+1}-->{piece_pols[i+1].eval(0)}")

                print(
                    f"vel at t={t} and pol={i}-->{piece_pols[i+0].derivative().eval(t)}")
                print(
                    f"vel at t={t} and pol={i+1}-->{piece_pols[i+1].derivative().eval(0)}")
                print(
                    f"accel at t={t} and pol={i}-->{piece_pols[i+0].derivative().derivative().eval(t)}")
                print(
                    f"accel at t={t} and pol={i+1}-->{piece_pols[i+1].derivative().derivative().eval(0)}")

            else:
                t = waypoints[i+1].t
                print(f"pos at t={t} and pol={i}  -->{piece_pols[i].eval(t)}")
                print(
                    f"pos at t={t} and pol={i+1}-->{piece_pols[i+1].eval(t)}")

                print(
                    f"vel at t={t} and pol={i}-->{piece_pols[i+0].derivative().eval(t)}")
                print(
                    f"vel at t={t} and pol={i+1}-->{piece_pols[i+1].derivative().eval(t)}")
                print(
                    f"accel at t={t} and pol={i}-->{piece_pols[i+0].derivative().derivative().eval(t)}")
                print(
                    f"accel at t={t} and pol={i+1}-->{piece_pols[i+1].derivative().derivative().eval(t)}")

    # if wp_type == Waypoint.WP_TYPE_X:
        # print("time points:", [wp.t for wp in waypoints])

    total_pol = PiecewisePolynomial(piece_pols, time_points)
    # t_final = sum(total_pol.time_durations)
    # print("t_final:", t_final)
    # for t in linspace(0, t_final, 100):
    # print(f"t={t} --> {total_pol.eval(t)}")

    return piece_pols, total_pol


def calculate_trajectory4D(waypoints):
    # waypoints:list of Point_time instances

    polx, pc_polx = calculate_trajectory1D(waypoints, Waypoint.WP_TYPE_X)
    poly, pc_poly = calculate_trajectory1D(waypoints, Waypoint.WP_TYPE_Y)
    polz, pc_polz = calculate_trajectory1D(waypoints, Waypoint.WP_TYPE_Z)
    polyaw, pc_polyaw = calculate_trajectory1D(waypoints, Waypoint.WP_TYPE_YAW)

    pols_coeffs = [polx, poly, polz, polyaw]
    pc_pols = [pc_polx, pc_poly, pc_polz, pc_polyaw]

    # visualize_trajectory3D(pc_pols)

    return pols_coeffs, pc_pols


def visualize_trajectory3D(pols):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    N = 100
    time_frame = linspace(0, 10, N)
    x = np.zeros(N)
    y = np.zeros(N)
    z = np.zeros(N)
    yaw = np.zeros(N)

    for i, t in enumerate(time_frame):
        x[i] = pols[0].eval(t)
        y[i] = pols[1].eval(t)
        z[i] = pols[2].eval(t)

    ax.scatter(x, y, z, c='r', marker='o')

    plt.show()


def allocateTime(waypoints: np.array, max_vel: float, max_acc: float):
    # waypoints:list of Waypoint instances
    # returns: list of time points

    # exclude last (yaw) column from the waypoints

    waypoints = waypoints[:, :-1]

    # print("waypoints.shape:", waypoints.shape)

    N = len(waypoints)-1
    durations = np.zeros(N)

    for k in range(N):
        p0 = waypoints[k, :]
        p1 = waypoints[k+1, :]

        # get distance between points
        D = np.linalg.norm(np.array(p0)-np.array(p1))

        acct = max_vel / max_acc
        accd = (max_acc * acct * acct / 2)
        dcct = max_vel / max_acc
        dccd = max_acc * dcct * dcct / 2

        if (D < accd + dccd):
            t1 = np.sqrt(max_acc * D) / max_acc
            t2 = (max_acc * t1) / max_acc
            dtxyz = t1 + t2
        else:
            t1 = acct
            t2 = (D - accd - dccd) / max_vel
            t3 = dcct
            dtxyz = t1 + t2 + t3

        durations[k] = dtxyz

    return durations


def allocateTimeProportional(waypoints: np.array, total_time):
    # waypoints:list of waypoints
    # returns: list of time points

    # exclude last (yaw) column from the waypoints

    # TODO: check if this time allocation leads to crazy trajectories and if so,
    #     increase the total time

    waypoints = waypoints[:, :-1]
    
    # print("waypoints.shape:", waypoints.shape)
    # for wp in waypoints:
    #     print(wp)

    N = len(waypoints)-1
    durations = np.zeros(N)
    distances = np.zeros(N)

    for k in range(N):
        p0 = waypoints[k, :]
        p1 = waypoints[k+1, :]

        # get distance between points
        D = np.linalg.norm(np.array(p0)-np.array(p1))
        distances[k] = D

    #find max distance
    dist_sum = np.sum(distances)
    
    for k in range(N):
        durations[k] = distances[k] / dist_sum * total_time

    return durations

def generate_traj(waypoints: np.array,total_time):
    traj_points = []
    t = 0
    # time_allocation = allocateTime(waypoints,MAX_VEL,MAX_ACC)

    time_allocation = allocateTimeProportional(waypoints,total_time)
    # print("time_allocation:", time_allocation)

    for i, point in enumerate(waypoints):
        # time allocation
        if i != 0:
            t = t+time_allocation[i-1]

        traj_points.append(Point_time(
            Waypoint(point[0], point[1], point[2], point[3]), t=t))

    pols_coeffs, pc_pols = calculate_trajectory4D(traj_points)

    return pols_coeffs, pc_pols


def create_traj(pols_coeffs, pc_pols):
    segments_number = len(pols_coeffs[0])
    matrix = np.zeros((segments_number, 4*8+1))

    for i in range(segments_number):
        # prev_time = matrix[i-1, 0] if i > 0 else 0
        matrix[i, 0] = pc_pols[0].time_durations[i] 
        # matrix[i, 0] = i*1
        matrix[i:i+1, 1:9] = pols_coeffs[0][i].p.T
        matrix[i, 9:17] = pols_coeffs[1][i].p.T
        matrix[i, 17:25] = pols_coeffs[2][i].p.T
        matrix[i, 25:33] = pols_coeffs[3][i].p.T

    matrix = np.array(matrix)
    # print("matrix.shape: ", matrix.shape)

    tr = Trajectory()
    tr.load_from_matrix(matrix)

    # tr.plot(timestep=0.2)
    return tr

def min_snap_traj_generation(waypoints,total_time)->Trajectory:
    pols_coeffs, pc_pols = generate_traj(waypoints,total_time)
    tr = create_traj(pols_coeffs, pc_pols)
    
    return tr

# waypoints format: t,x,y,z,yaw
# waypoints = [
#     [0.0, 0.0, height, yaw_ref],
# #
#     [0.0, 1.0 - 0.1, height, yaw_ref],
#     [0.0, 1.0, height, yaw_ref],
#     [0.1, 1.0 + 0.1, height, yaw_ref],
# #
#     [1.0-0.1, 1.0, height, yaw_ref],
#     [1.0, 1.0, height, yaw_ref],
#     [1.0, 1.0-0.1, height, yaw_ref],
# #
#     [1.0, 0.0 + 0.1, height, yaw_ref],
#     [1.0, 0.0, height, yaw_ref],
#     [1.0 - 0.1, 0.0, height, yaw_ref],
# #
#     [0.0, 0.0, height, yaw_ref],
# ]
    
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


if __name__ == "__main__":
    yaw_ref = 0
    height = 1
    
    waypoints = [
        [0.0, 0.0, height, yaw_ref],
        [0.5, 0.0, height, yaw_ref],

        [1.0, 0.0, height, yaw_ref],
        [1.0, 0.5, height, yaw_ref],

        [1.0, 1.0, height, yaw_ref],

        [0.5, 1.0, height, yaw_ref],

        [0.0, 1.0, height, yaw_ref],

        [0.0, 0.5, height, yaw_ref],
        [0.0, 0.0, height, yaw_ref],
    ]
    
    waypoints = [
        [ 0.983 , -0.945 , 0.993 ,0],
        [ 0.949 , -0.916 , 0.981 ,0],
        [ 0.915 , -0.887 , 0.969 ,0],
        [ 0.881 , -0.857 , 0.957 ,0],
        [ 0.847 , -0.828 , 0.945 ,0],
        [ 0.813 , -0.799 , 0.933 ,0 ],
        [ 0.779 , -0.770 , 0.921 ,0],
        [ 0.745 , -0.741 , 0.909 ,0],
        [ 0.711 , -0.712 , 0.897 ,0],
        [ 0.677 , -0.683  , 0.885 ,0],
        [ 0.643 , -0.653 , 0.873 ,0],
        [ 0.609 , -0.624 , 0.861 ,0],
        [ 0.575 , -0.595 , 0.849 ,0],
        [ 0.541 , -0.566 , 0.837 ,0 ],
        [ 0.507 , -0.537 , 0.825 ,0],
        [ 0.473 , -0.508 , 0.813 ,0],
        [ 0.440 , -0.479 , 0.801 ,0],
        [ 0.406 , -0.449 , 0.789 ,0],
        [ 0.372 , -0.418 , 0.777 ,0],
        [ 0.339 , -0.386 , 0.765 ,0],
        [ 0.307 , -0.353 , 0.753 ,0],
        [ 0.275 , -0.320 , 0.741 ,0],
        [ 0.245 , -0.285 , 0.729 ,0],
        [ 0.216 , -0.250 , 0.717 ,0],
        [ 0.188 , -0.214 , 0.705 ,0],
        [ 0.162 , -0.179 , 0.693 ,0],
        [ 0.136 , -0.145 , 0.681 ,0],
        [ 0.112 , -0.112 , 0.669 ,0],
        [ 0.089 , -0.079 , 0.657 ,0],
        [ 0.065 , -0.047 , 0.645 ,0],
        [ 0.043 , -0.016 , 0.633 ,0],
        [ 0.020 ,  0.014 , 0.621 ,0],
        [-0.002 ,  0.045 , 0.609 ,0],
        [-0.025 ,  0.077 , 0.597 ,0 ],
        [-0.050 ,  0.111  , 0.585 ,0 ],
        [-0.078 ,  0.149 , 0.573 ,0],
        [-0.108 ,  0.192 , 0.561 ,0  ],
        [-0.141 ,  0.240  , 0.549 ,0 ],
        [-0.177 ,  0.294  , 0.537 ,0],
        [-0.215 ,  0.354 , 0.525 ,0],
        [-0.256 ,  0.419 , 0.513 ,0],
        [-0.300 ,  0.489 , 0.501 ,0],
        [-0.346 ,  0.564 , 0.490 ,0],
        [-0.395 ,  0.643 , 0.478 ,0],
        [-0.447 ,  0.722 , 0.466 ,0],
        [-0.501 ,  0.802 , 0.454 ,0],
        [-0.558 ,  0.882 , 0.442 ,0 ],
        [-0.618 ,  0.963 , 0.430 ,0],
        [-0.680 ,  1.045 , 0.418 ,0],
        [-0.746 ,  1.128 , 0.406 ,0],
        [-0.779 ,  1.169 , 0.400 ,0 ],
    ]
    # MAX_VEL = 3
    # MAX_ACC = 3

    waypoints=[
     [ 0.98303212 ,-0.94542514 , 0.99399998,0],
     [ 0.94909636 ,-0.91627543 , 0.98199994,0],
     [ 0.9151606  ,-0.88712571 , 0.96999991,0],
     [ 0.88122484 ,-0.85797599 , 0.95799987,0],
     [ 0.84728908 ,-0.82882628 , 0.94599983,0],
     [ 0.81335332 ,-0.79967656 , 0.9339998 ,0],
     [ 0.77941756 ,-0.77052685 , 0.92199976,0],
     [ 0.7454818  ,-0.74137713 , 0.90999972,0],
     [ 0.71154603 ,-0.71222742 , 0.89799968,0],
     [ 0.67761027 ,-0.6830777  , 0.88599965,0],
     [ 0.64367451 ,-0.65392798 , 0.87399961,0],
     [ 0.60973875 ,-0.62477827 , 0.86199957,0],
     [ 0.57580299 ,-0.59562855 , 0.84999953,0],
     [ 0.54186723 ,-0.56647884 , 0.8379995 ,0],
     [ 0.50793147 ,-0.53732912 , 0.82599946,0],
     [ 0.47399571 ,-0.50817941 , 0.81399942,0],
     [ 0.44005995 ,-0.47902969 , 0.80199939,0],
     [ 0.4062736  ,-0.44922623 , 0.78999935,0],
     [ 0.37272632 ,-0.41856135 , 0.77799931,0],
     [ 0.33970393 ,-0.38679147 , 0.76599928,0],
     [ 0.30740045 ,-0.35389865 , 0.75399924,0],
     [ 0.27597276 ,-0.32000234 , 0.74199921,0],
     [ 0.24560811 ,-0.28529865 , 0.72999918,0],
     [ 0.21649164 ,-0.25010282 , 0.71799915,0],
     [ 0.18871876 ,-0.21483843 , 0.70599912,0],
     [ 0.1622432  ,-0.17991379 , 0.69399909,0],
     [ 0.13694193 ,-0.14562244 , 0.68199906,0],
     [ 0.11268735 ,-0.11215223 , 0.66999904,0],
     [ 0.08907383 ,-0.07944733 , 0.65799901,0],
     [ 0.06593392 ,-0.04750054 , 0.64599899,0],
     [ 0.04313557 ,-0.01628442 , 0.63399897,0],
     [ 0.02033101 , 0.01481188 , 0.62199895,0],
     [-0.00257602 , 0.04599376 , 0.60999892,0],
     [-0.02558012 , 0.07726728 , 0.5979989 ,0],
     [-0.05066033 , 0.1114889  , 0.5859989 ,0],
     [-0.07837222 , 0.14977682 , 0.57399894,0],
     [-0.10877433 , 0.19236745 , 0.561999  ,0],
     [-0.1419815  , 0.2405046  , 0.5499991 ,0],
     [-0.17772321 , 0.2947204  , 0.53799925,0],
     [-0.21547425 , 0.35478366 , 0.52599943,0],
     [-0.25624065 , 0.41970496 , 0.51399966,0],
     [-0.30018103 , 0.48957904 , 0.50199991,0],
     [-0.34686339 , 0.56446155 , 0.49000021,0],
     [-0.39595945 , 0.64322563 , 0.47800053,0],
     [-0.44762108 , 0.72244451 , 0.46600085,0],
     [-0.50179798 , 0.80217095 , 0.45400117,0],
     [-0.55841301 , 0.88247215 , 0.4420015 ,0],
     [-0.61800849 , 0.96356163 , 0.43000184,0],
     [-0.68067019 , 1.04548637 , 0.41800218,0],
     [-0.74620039 , 1.12827261 , 0.40600252,0],
     [-0.77999194 , 1.16998877 , 0.4000027 ,0],
     ]

    waypoints=[
    [ 0.96579348, -0.93383903,  0.98799904,0],
    [ 0.93158697, -0.90767805,  0.97599807,0],
    [ 0.89738045, -0.88151708,  0.96399711,0],
    [ 0.86317394, -0.8553561,   0.95199615,0],
    [ 0.82896742, -0.82919513,  0.93999519,0],
    [ 0.79476091, -0.80303415,  0.92799422,0],
    [ 0.76055439, -0.77687318,  0.91599326,0],
    [ 0.72634787, -0.7507122,   0.9039923 ,0],
    [ 0.69214136, -0.72455123,  0.89199134,0],
    [ 0.65793484, -0.69839025,  0.87999037,0],
    [ 0.62372833, -0.67222928,  0.86798941,0],
    [ 0.58952181, -0.6460683,   0.85598845,0],
    [ 0.5553153 , -0.61990733,  0.84398749,0],
    [ 0.52110878, -0.59374635,  0.83198652,0],
    [ 0.48690226, -0.56758538,  0.81998556,0],
    [ 0.45269575, -0.5414244,   0.8079846 ,0],
    [ 0.41848923, -0.51526343,  0.79598363,0],
    [ 0.3847785 , -0.48799991,  0.78398267,0],
    [ 0.35166421, -0.45968488,  0.77198171,0],
    [ 0.31926687, -0.4303965,   0.75998075,0],
    [ 0.28772501, -0.40025399,  0.74797979,0],
    [ 0.25718381, -0.36943317,  0.73597883,0],
    [ 0.22776885, -0.33817304,  0.72397787,0],
    [ 0.19891148, -0.30606086,  0.71197691,0],
    [ 0.1700788 , -0.27280668,  0.69997597,0],
    [ 0.140621  , -0.23697878,  0.68797505,0],
    [ 0.11075782, -0.19857817,  0.67597415,0],
    [ 0.07903686, -0.14871527,  0.66397347,0],
    [ 0.04662563, -0.09871527,  0.65197304,0],
    [ 0.0141407 , -0.04871527,  0.63997286,0],
    [-0.01694145,  0.00128473,  0.62797293,0],
    [-0.04756819,  0.05128473,  0.61597304,0],
    [-0.07798533,  0.10128473,  0.60397316,0],
    [-0.10840447,  0.15128473,  0.59197329,0],
    [-0.13875417,  0.20128473,  0.57997342,0],
    [-0.1701622 ,  0.25128473,  0.56797356,0],
    [-0.20256298,  0.30128473,  0.5559737 ,0],
    [-0.23614926,  0.35128473,  0.54397385,0],
    [-0.27091506,  0.40128473,  0.53197399,0],
    [-0.30685235,  0.45128473,  0.51997414,0],
    [-0.34598571,  0.50128473,  0.50797429,0],
    [-0.38829824,  0.55128473,  0.49597444,0],
    [-0.43376268,  0.60128473,  0.48397459,0],
    [-0.48233135,  0.65128473,  0.47197475,0],
    [-0.53217658,  0.70128473,  0.45997491,0],
    [-0.58058928,  0.75128473,  0.44797507,0],
    [-0.63032873,  0.80128473,  0.43597524,0],
    [-0.68016128,  0.85128473,  0.42397542,0],
    [-0.72998925,  0.90128473,  0.4119756 ,0],
    [-0.77982601,  0.95128473,  0.39997579,0],
    ]

    total_time = 6
    waypoints = np.array(waypoints)
    print(waypoints.shape)
    downsample_step = 4

    tr=min_snap_traj_generation(waypoints[::downsample_step,:],total_time= total_time)    

    debug_traj_generation(waypoints, tr,downsample_step)

    plt.show()
