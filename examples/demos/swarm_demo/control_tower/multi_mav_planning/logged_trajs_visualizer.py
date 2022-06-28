from typing import List
from colorama import Fore
import numpy as np 
import matplotlib.pyplot as plt
from trajectory_gen import uav_trajectory
from optim_problem import plotting


def get_sequences(trajs):
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
    return MAV_sequence

def align_sequences(MAV_sequence):
    """Aligns MAV sequences because sometimes they are not the same size """

    lengths=[len(i) for i in MAV_sequence]
    min_len=min(lengths)
    print("min_len:", min_len)
    for i in range(len(MAV_sequence)):
        MAV_sequence[i]=MAV_sequence[i][:min_len]
    MAV_sequence=np.array(MAV_sequence)
    return MAV_sequence

def load_trajs_from_matrix(trajs_mat)->List[uav_trajectory.Trajectory]:
    trajs:List[uav_trajectory.Trajectory]=[]
    
    for i in range(len(trajs_mat)):
        traj=uav_trajectory.Trajectory()
        traj.load_from_matrix(trajs_mat[i])
        trajs.append(traj)
    return trajs

def find_shortest_distance(MAV_sequences):
    """Finds the shortest distance between MAVs
        MAV_sequences: list of MAV paths 
           dimensions: [N_MAV, N_timesteps, 3]    
    """

    print("MAV_sequences.shape:", MAV_sequences.shape)
    dist=np.ones((len(MAV_sequences),len(MAV_sequences),MAV_sequences.shape[1]))
    
    for i in range(len(MAV_sequences)):
        for j in range(len(MAV_sequences)):
            if i==j:
                continue
            for k in range(MAV_sequences.shape[2]):
                dist[i,j,k]=np.linalg.norm(MAV_sequences[i][k]-MAV_sequences[j][k])
    
    print("min distance:", min(dist.flatten()))

def get_xos_xrefs_from_sequences(MAV_sequences):
    """Returns x0s and xrefs of MAVs"""
    x0s=[ i[0] for i in MAV_sequences]
    xrefs=[ i[-1] for i in MAV_sequences]
    
    return x0s, xrefs

def plot_trajs_from_file(filename="traj_matrices_0.npy"):
    path="/home/oem/MARIOS/crazyflie-firmware-experimental/examples/demos/swarm_demo/control_tower/multi_mav_planning/logged_trajs/"
    trajs_mat=np.load(path+filename)
    
    x0s,xrefs=None,None

    print("trajs_mat.shape:", trajs_mat.shape)
    if trajs_mat.shape[2]!=33: # if x0s and xrefs have been logged (normally it should be 33 because of pol coefficients)
        print("x0s and xrefs have been logged")
        x0s=trajs_mat[0]
        xrefs=trajs_mat[1]
        trajs_mat=trajs_mat[2:]
    
    
    trajs = load_trajs_from_matrix(trajs_mat)

    MAV_sequence = get_sequences(trajs)
    
    if x0s is None:
        x0s,xrefs=get_xos_xrefs_from_sequences(MAV_sequence)

    print(Fore.GREEN+"x0s :")
    for i,x0 in enumerate(x0s):
        print(i,": {:.2f} {:.2f} {:.2f}".format(x0[0],x0[1],x0[2]))

    print(Fore.GREEN+"xrefs :")
    for i,xref in enumerate(xrefs):
        print(i,": {:.2f} {:.2f} {:.2f}".format(xref[0],xref[1],xref[2]))
    print(Fore.RESET)

    MAV_sequence = align_sequences(MAV_sequence)


    #check shortest distance between MAVs
    find_shortest_distance(MAV_sequence)

    #Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    plotting.animate3D(MAV_sequence,ax=ax,fig=fig)

    plt.show()


if __name__=="__main__":
    traj_numbers=range(6)
    traj_numbers=[1,2]
    # filename="traj_matrices_0.npy"
    for i in traj_numbers:
        filename="traj_matrices_{}.npy".format(i)
        plot_trajs_from_file(filename)

    

