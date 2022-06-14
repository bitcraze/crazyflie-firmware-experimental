# Useful link :https://realpython.com/linear-programming-python/
import time
import numpy as np
from numpy.core.function_base import linspace
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

Last wp(waipont) conditions:
    x(-1) = waypoint(-1)
    x'(-1) = x''(-1)=x'''(-1)= 0 
"""
##############################################################################################


def calculate_trajectory1D(waypoints, wp_type=Waypoint.WP_TYPE_X):
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
    A = np.zeros((8*n, 8*n))
    b = np.zeros((8*n, 1))
    # print("A.shape:", A.shape)
    # print("b.shape:", b.shape)

    time_points = []
    prev_t = 0
    for i, traj_point in enumerate(waypoints):
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

    # print("det(A):", np.linalg.det(A))
    # np.savetxt("A.csv", A, delimiter=",")
    # np.savetxt("b.csv", b, delimiter=",")

    polynomials_coefficients = np.linalg.solve(a=A, b=b)

    print("polynomials_coefficients.shape:", polynomials_coefficients.shape)

    piece_pols = []  # piecewise polynomials
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
                    f"pos at t={t} and pol={i+1}-->{piece_pols[i+1].eval(t)}")

                print(
                    f"vel at t={t} and pol={i}-->{piece_pols[i+0].derivative().eval(t)}")
                print(
                    f"vel at t={t} and pol={i+1}-->{piece_pols[i+1].derivative().eval(t)}")
                print(
                    f"accel at t={t} and pol={i}-->{piece_pols[i+0].derivative().derivative().eval(t)}")
                print(
                    f"accel at t={t} and pol={i+1}-->{piece_pols[i+1].derivative().derivative().eval(t)}")

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

    total_pol = PiecewisePolynomial(piece_pols, time_points)
    # t_final = sum(total_pol.time_durations)
    # print("t_final:", t_final)
    # for t in linspace(0, t_final, 100):
    # print(f"t={t} --> {total_pol.eval(t)}")

    return piece_pols, total_pol


def calculate_trajectory4D(waypoints) -> Trajectory:
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

    print("waypoints.shape:", waypoints.shape)

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
    # waypoints:list of Waypoint instances
    # returns: list of time points

    # exclude last (yaw) column from the waypoints

    # TODO: check if this time allocation leads to crazy trajectories and if so,
    #     increase the total time

    waypoints = waypoints[:, :-1]
    
    print("waypoints.shape:", waypoints.shape)
    for wp in waypoints:
        print(wp)

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


def generate_traj(waypoints: np.array):
    traj_points = []
    t = 0
    # time_allocation = allocateTime(waypoints,MAX_VEL,MAX_ACC)

    time_allocation = allocateTimeProportional(waypoints, 35)
    print("time_allocation:", time_allocation)

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
        prev_time = matrix[i-1, 0] if i > 0 else 0
        matrix[i, 0] = pc_pols[0].time_durations[i] + prev_time
        matrix[i, 0] = i*1
        matrix[i:i+1, 1:9] = pols_coeffs[0][i].p.T
        matrix[i, 9:17] = pols_coeffs[1][i].p.T
        matrix[i, 17:25] = pols_coeffs[2][i].p.T
        matrix[i, 25:33] = pols_coeffs[3][i].p.T

    matrix = np.array(matrix)
    print("matrix.shape: ", matrix.shape)

    tr = Trajectory()
    tr.load_from_matrix(matrix)

    # tr.plot(timestep=0.2)
    return tr

def main(waypoints):
    pols_coeffs, pc_pols = generate_traj(waypoints)
    tr = create_traj(pols_coeffs, pc_pols)
    
    return tr
    
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
    MAX_VEL = 3
    MAX_ACC = 3
    
    waypoints = np.array(waypoints)
    main(waypoints)