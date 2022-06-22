from typing import List
import numpy as np 
import matplotlib.pyplot as plt
from trajectory_gen import uav_trajectory

def plot_trahs_from_file(filename="traj_matrices_0.npy"):
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

    for i in range(len(trajs)):
        trajs[i].plot(timestep=0.1,ax=ax,label=str(i))

    plt.show()

if __name__=="__main__":
    # filename="traj_matrices_0.npy"
    for i in range(4):
        filename="traj_matrices_{}.npy".format(i)
        plot_trahs_from_file(filename)

    

