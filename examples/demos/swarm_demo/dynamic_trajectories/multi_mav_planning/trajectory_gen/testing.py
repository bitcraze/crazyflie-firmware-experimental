from numpy import dsplit, matrix
try:
    from bezier_trajectory import *
    import uav_trajectory
    from generate_trajectory import generate_trajectory
    import marios_gen

except:
    print("Could not import bezier_trajectory")
    from .bezier_trajectory import *
    from . import uav_trajectory
    from .generate_trajectory import generate_trajectory
    from . import marios_gen
import time


def bezier(waypoints):
    # segment_time = 2
    z = 1
    yaw = 0

    segments = []

    time_durations = marios_gen.allocateTimeProportional(waypoints,5)

    # # Nodes with one control point has not velocity, this is similar to calling
    # # goto in the High-level commander

    DT=8
    for i in range(len(waypoints) - 1):
        yaw=0
        pos0=(waypoints[i][0],waypoints[i][1],waypoints[i][2],yaw)
        pos1=(waypoints[i+1][0],waypoints[i+1][1],waypoints[i+1][2],yaw)

        vel0=[waypoints[i+1][0]-waypoints[i][0],waypoints[i+1][1]-waypoints[i][1],0,0]
        vel0=[i/DT for i in vel0]

        if i !=len(waypoints)-2:
            vel1=[waypoints[i+2][0]-waypoints[i+1][0],waypoints[i+2][1]-waypoints[i+1][1],0,0]
            vel1=[i/DT for i in vel1]
        else:
            vel1=(0,0,0,0)
        print ("================================================================")
        print ("pos0:",pos0)
        print ("pos1:",pos1)
        print ("vel0:",vel0)
        print ("vel1:",vel1)


        n0=Node(pos0,yaw,vel0)
        n1=Node(pos1,yaw,vel1)

        seg=Segment(n0,n1,time_durations[i])
        segments.append(seg)


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

    tr=uav_trajectory.Trajectory()
    tr.load_from_Bezier_segmnents(segments)
    tr.plot(0.2)
    pass


def wolfgang(waypoints):
    time_durations=marios_gen.allocateTimeProportional(waypoints,5)
    print("time_durations:",time_durations)
    #add empty column to waypoints
    waypoints=np.insert(waypoints,0,0,axis=1)

    for i in range(len(waypoints)):
        waypoints[i][0]=waypoints[i-1][0]+time_durations[i-1] if i!=0 else 0

    print(waypoints[:,0])

    segments_number = len(waypoints) - 1
    t0 = time.time()
    traj = generate_trajectory(waypoints, segments_number)
    dt = time.time()-t0
    print("Generated trajectory in %f seconds:" % (dt))

    traj.plot(timestep=0.2)


def marios(waypoints):
    # remove fist column (time) of waypoints
    return marios_gen.main(waypoints)

test=[
 [0.02382189, 0.0156142,  1.00382214,0.0],
 [0.04764378, 0.0312284 , 1.00764428,0.0],
 [0.07146567, 0.04684261, 1.01146642,0.0],
 [0.09528756, 0.06245681, 1.01528856,0.0],
 [0.11910945, 0.07807101, 1.0191107 ,0.0],
 [0.14293134, 0.09368521, 1.02293284,0.0],
 [0.16675323, 0.10929942, 1.02675498,0.0],
 [0.19057512, 0.12491362, 1.03057712,0.0],
 [0.21439701, 0.14052782, 1.03439926,0.0],
 [0.2382189 , 0.15614202, 1.0382214 ,0.0],
 [0.26204079, 0.17175622, 1.04204355,0.0],
 [0.28586268, 0.18737043, 1.04586569,0.0],
 [0.30968457, 0.20298463, 1.04968783,0.0],
 [0.33350646, 0.21859883, 1.05350997,0.0],
 [0.35732835, 0.23421303, 1.05733211,0.0],
 [0.38115024, 0.24982723, 1.06115425,0.0],
 [0.40497213, 0.26544144, 1.06497639,0.0],
 [0.42879402, 0.28105564, 1.06879853,0.0],
 [0.45261591, 0.29666984, 1.07262067,0.0],
 [0.4764378 , 0.31228404, 1.07644281,0.0],
 [0.50025969, 0.32789825, 1.08026495,0.0],
 [0.52408158, 0.34351245, 1.08408709,0.0],
 [0.54790347, 0.35912665, 1.08790923,0.0],
 [0.57167666, 0.37487056, 1.09168267,0.0],
 [0.59539411, 0.39073839, 1.09540038,0.0],
 [0.61904781, 0.40672213, 1.09905432,0.0],
 [0.64263759, 0.42280228, 1.10264436,0.0],
 [0.66615471, 0.43896544, 1.10616173,0.0],
 [0.6895907 , 0.45519383, 1.10959797,0.0],
 [0.71010321, 0.47258198, 1.11011073,0.0],
 [0.73052341, 0.4899886 , 1.11053118,0.0],
 [0.75055136, 0.50764696, 1.11055938,0.0],
 [0.76978155, 0.52547048, 1.10978982,0.0],
 [0.78858599, 0.54341183, 1.10859451,0.0],
 [0.80689824, 0.56143903, 1.10690702,0.0],
 [0.82470495, 0.58000492, 1.10471398,0.0],
 [0.84200057, 0.59904032, 1.10200985,0.0],
 [0.85918867, 0.61833408, 1.09919821,0.0],
 [0.87274196, 0.63899341, 1.09275175,0.0],
 [0.88606542, 0.66098935, 1.08607547,0.0],
 [0.8989735 , 0.68542851, 1.0789838 ,0.0],
 [0.91146292, 0.71222568, 1.07147348,0.0],
 [0.9234396 , 0.74392952, 1.06345041,0.0],
 [0.93506358, 0.78043096, 1.05507465,0.0],
 [0.94640965, 0.8168792 , 1.04642097,0.0],
 [0.95724308, 0.85332744, 1.03725467,0.0],
 [0.96797325, 0.88977569, 1.02798509,0.0],
 [0.97868214, 0.92622393, 1.01869423,0.0],
 [0.98937839, 0.96319094, 1.00939074,0.0],
 [1.00007342, 1.00013866, 1.00008603,0.0],]

if __name__ == "__main__":
    # waypoints format: t,x,y,z,yaw
    yaw_ref = 0
    height = 1
    waypoints = [
        [0.0, 0.0, height, yaw_ref],#

        [0.0,0.5,height,yaw_ref],

        [0.0, 1.0, height, yaw_ref],#
        
        [0.5, 1.0, height, yaw_ref],

        [1.0, 1.0, height, yaw_ref],#

        [1.0, 0.5, height, yaw_ref],
        
        [1.0, 0.0, height, yaw_ref],#

        [0.5 ,0.0, height, yaw_ref],
        
        [0.0, 0.0, height, yaw_ref],#
    ]

    # waypoints = np.array(waypoints)
    waypoints = np.array(test)
    
    #skip some rows
    waypoints = waypoints[::6,:]
    # wolfgang(waypoints)

    marios(waypoints)

    # bezier(waypoints)
