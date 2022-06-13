from numpy import dsplit, matrix
from bezier_trajectory import *
import uav_trajectory
import time
from generate_trajectory import generate_trajectory
from marios_gen import *

def bezier():
    # segment_time = 2
    # z = 1
    # yaw = 0

    # segments = []

    # # Nodes with one control point has not velocity, this is similar to calling
    # # goto in the High-level commander

    # n0 = Node((0, 0, z, yaw))
    # n1 = Node((1, 0, z, yaw))
    # n2 = Node((1, 1, z, yaw))
    # n3 = Node((0, 1, z, yaw))

    # # segments.append(Segment(n0, n1, segment_time))
    # # segments.append(Segment(n1, n2, segment_time))
    # # segments.append(Segment(n2, n3, segment_time))
    # # segments.append(Segment(n3, n0, segment_time))


    # # By setting the q1 control point we get velocity through the nodes
    # # Increase d to 0.7 to get some more action
    # d = 0.1

    # # n5 = Node((1, 0, z, yaw), q1=(1 + d, 0 + d, z, yaw))
    # # n6 = Node((1, 1, z, yaw), q1=(1 - d, 1 + d, z, yaw))
    # # n7 = Node((0, 1, z, yaw), q1=(0 - d, 1 - d, z, yaw))

    # v=1

    # n5 = Node((1, 0, z, yaw), q1=( 0.1, v , 0, 0))
    # n6 = Node((1, 1, z, yaw), q1=( 0.1,-v , 0, 0))
    # n7 = Node((0, 1, z, yaw), q1=(-v, -0.1 , 0, 0))

    # #measure time
    # t0=time.time()
    # segments.append(Segment(n0, n5, segment_time))
    # segments.append(Segment(n5, n6, segment_time))
    # segments.append(Segment(n6, n7, segment_time))
    # segments.append(Segment(n7, n0, segment_time))
    # dt=time.time()-t0
    # print("Time to generate segments (msec):",dt*1000)

    # tr=uav_trajectory.Trajectory()
    # tr.load_from_Bezier_segmnents(segments)
    # tr.plot(0.2)
    pass

def wolfgang(waypoints):
    segments_number = len(waypoints) - 1
    t0 = time.time()
    traj = generate_trajectory(waypoints, segments_number)
    dt = time.time()-t0
    print("Generated trajectory in %f seconds:" % (dt))

    traj.plot(timestep=0.2)

def marios(waypoints):
    #remove fist column of waypoints
    waypoints = waypoints[:,1:]

    traj_points=[]
    for i, point in enumerate(waypoints):
        traj_points.append(Point_time(
            Waypoint(point[0], point[1], point[2], point[3]), t=i*timestep))

    pols_coeffs, pc_pols = calculate_trajectory4D(traj_points)
    traj = Trajectory()
    traj.load_from_pol_segments(pols_coeffs)
    traj.plot(timestep=0.2)

if __name__ == "__main__":
    # waypoints format: t,x,y,z,yaw
    yaw_ref = 0
    height = 1
    waypoints = [
        [0, 0.0, 0.0, height, yaw_ref],

        # [0.9, 0.0, 1.0 - 0.1, height, yaw_ref],
        [1.0, 0.0, 1.0, height, yaw_ref],        
        # [1.1, 0.1, 1.0 + 0.1, height, yaw_ref],

        # [2-0.1, 1.0-0.1, 1.0, height, yaw_ref],
        [2,     1.0, 1.0, height, yaw_ref],
        # [2+0.1, 1.0, 1.0-0.1, height, yaw_ref],
        
        # [3-0.1, 1.0, 0.0 + 0.1, height, yaw_ref],
        [3, 1.0, 0.0, height, yaw_ref],
        # [3+0.1, 1.0 - 0.1, 0.0, height, yaw_ref],
        
        [4, 0.0, 0.0, height, yaw_ref],
    ]

    waypoints = np.array(waypoints)

    wolfgang(waypoints)

    marios(waypoints)
