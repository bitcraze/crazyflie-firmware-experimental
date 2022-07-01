from typing import List, Tuple
from colorama import Fore
import numpy as np 
import matplotlib.pyplot as plt

try:
    from trajectory_gen import uav_trajectory
    from optim_problem import plotting
except:
    from .trajectory_gen import uav_trajectory
    from .optim_problem import plotting


def analyse_trajs_from_matrix(trajs_mat):
    x0s,xrefs=None,None

    if type(trajs_mat)!=np.array:
        trajs_mat= np.array(trajs_mat) 

    print("trajs_mat.shape:", trajs_mat.shape)
    print("trajs_mat[0].shape:", np.array(trajs_mat[0]).shape)

    print("len(trajs_mat[0])",len(trajs_mat[0]))

    if len(trajs_mat[0])!=33 and  len(trajs_mat[0][0])!=33: # if x0s and xrefs have been logged (normally it should be 33 because of pol coefficients)
        print("x0s and xrefs have been logged")
        x0s=trajs_mat[0]
        xrefs=trajs_mat[1]
        trajs_mat=trajs_mat[2:]
    
    
    trajs = load_trajs_from_matrix(trajs_mat)

    MAV_sequence = get_sequences_from_trajs(trajs)
    
    analyze_MAV_sequences(x0s,xrefs, MAV_sequence)

def analyze_MAV_sequences(x0s,xrefs, MAV_sequence):
    """
        Finds the minimum distance between MAVs while flying and plots it vs time
        Animate the MAVs sequence at the end
    """
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
    dists=calculate_distances(MAV_sequence)

    #find index of shortest distance
    min_dist_index=np.unravel_index(dists.argmin(), dists.shape)
    MAV_pairs=[min_dist_index[0],min_dist_index[1]]
    #plot distance between MAVs
    plot_distance_in_MAV_pairs(dists,MAV_pairs)

    #Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    plotting.animate3D(MAV_sequence,ax=ax,fig=fig)

    # plotting.plotGridSpec(MAV_sequences=MAV_sequence)
    plt.show()

def plot_distance_in_MAV_pairs(dists,MAV_pairs):
    """
    Plots distance between MAVs included MAV_pairs
    @param dists: distance matrix in shape (N_MAVs,N_MAVs,horizon_length)
    @param MAV_pairs: pair of MAVs to plot
    """

    plt.figure()

    plt.title("Distance between MAVs {} and {}".format(MAV_pairs[0],MAV_pairs[1]))
    plt.xlabel("ticks")
    plt.ylabel("distance")
    plt.plot(dists[MAV_pairs[0],MAV_pairs[1],:])

    plt.grid()
    plt.show()

def get_sequences_from_trajs(trajs:List[uav_trajectory.Trajectory])->List[List[np.array]]:
    """Returns MAV sequences from list of Trajectory objects"""
    MAV_sequence=[]
    
    for i in range(len(trajs)):
        # trajs[i].plot(timestep=0.1,ax=ax,label=str(i))
        positions,traj_time = trajs[i].get_path(timestep=0.1)
        x,y,z=positions[0],positions[1],positions[2]
        seq=[]
        for j in range(len(x)):
            seq.append([x[j],y[j],z[j]])
        seq=np.array(seq)
        print(seq.shape)
        MAV_sequence.append(seq)
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
    """Loads trajs from matrix into list of Trajectory objects"""
    trajs:List[uav_trajectory.Trajectory]=[]
    
    for i in range(len(trajs_mat)):
        traj=uav_trajectory.Trajectory()
        traj.load_from_matrix(trajs_mat[i])
        trajs.append(traj)
    return trajs

def calculate_distances(MAV_sequences)->np.ndarray:
    """Finds the shortest distance between MAVs
        MAV_sequences: list of MAV paths 
           dimensions: [N_MAV, N_timesteps, 3]    
        
        returns: array of distances between MAVs 
              dimensions: [N_MAV, N_MAV,N_timesteps]
        
        e.g: (i,j,k) = distance between MAV i and MAV j at "time" k
    """

    print("MAV_sequences.shape:", MAV_sequences.shape)
    dist=np.ones((len(MAV_sequences),len(MAV_sequences),MAV_sequences.shape[1]))
    
    for i in range(len(MAV_sequences)):
        for j in range(len(MAV_sequences)):
            if i==j:
                continue
            for k in range(MAV_sequences.shape[1]):
                dist[i,j,k]=np.linalg.norm(MAV_sequences[i][k]-MAV_sequences[j][k])
    
    print("min distance:", min(dist.flatten()))

    return dist

def get_xos_xrefs_from_sequences(MAV_sequences)->Tuple(List[np.array],List[np.array]):
    """Returns x0s and xrefs of MAVs"""
    x0s=[ i[0] for i in MAV_sequences]
    xrefs=[ i[-1] for i in MAV_sequences]
    
    return x0s, xrefs


