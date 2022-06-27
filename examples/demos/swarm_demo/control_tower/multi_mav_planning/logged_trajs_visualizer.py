from typing import List
import numpy as np 
import matplotlib.pyplot as plt
from trajectory_gen import uav_trajectory
from optim_problem import plotting

def plot_trajs_from_file(filename="traj_matrices_0.npy"):
    path="/home/oem/MARIOS/crazyflie-firmware-experimental/examples/demos/swarm_demo/control_tower/multi_mav_planning/logged_trajs/"
    trajs_mat=np.load(path+filename)
    print(trajs_mat.shape)
    
    trajs:List[uav_trajectory.Trajectory]=[]
    
    for i in range(len(trajs_mat)):
        traj=uav_trajectory.Trajectory()
        traj.load_from_matrix(trajs_mat[i])
        trajs.append(traj)
    
    #create 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    MAV_sequence=[]
    
    for i in range(len(trajs)):
        # trajs[i].plot(timestep=0.1,ax=ax,label=str(i))
        x,y,z=trajs[i].get_path(timestep=0.1)
        traj=[]
        for j in range(len(x)):
            traj.append([x[j],y[j],z[j]])
        traj=np.array(traj)
        print(traj.shape)
        MAV_sequence.append(traj)

    lengths=[len(i) for i in MAV_sequence]
    min_len=min(lengths)
    print("min_len:", min_len)
    for i in range(len(MAV_sequence)):
        MAV_sequence[i]=MAV_sequence[i][:min_len]
    MAV_sequence=np.array(MAV_sequence)
    plotting.animate3D(MAV_sequence)

    plt.show()

if __name__=="__main__":
    traj_numbers=range(6)
    traj_numbers=[1,2]
    # filename="traj_matrices_0.npy"
    for i in traj_numbers:
        filename="traj_matrices_{}.npy".format(i)
        plot_trajs_from_file(filename)

    

