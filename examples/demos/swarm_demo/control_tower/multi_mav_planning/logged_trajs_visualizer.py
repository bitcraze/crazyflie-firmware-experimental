from typing import List
from colorama import Fore
import numpy as np 
import matplotlib.pyplot as plt
try:
    from trajectory_gen import uav_trajectory
    from optim_problem import plotting
    from MAV_utils import analyse_trajs_from_matrix
except:
    from .trajectory_gen import uav_trajectory
    from .optim_problem import plotting
    from .MAV_utils import analyse_trajs_from_matrix

def analyse_trajs_from_file(filename="traj_matrices_0.npy"):
    path="/home/oem/MARIOS/crazyflie-firmware-experimental/examples/demos/swarm_demo/control_tower/multi_mav_planning/logged_trajs/"
    trajs_mat=np.load(path+filename, allow_pickle = True)
    analyse_trajs_from_matrix(trajs_mat)

if __name__=="__main__":
    # traj_numbers=range(6)
    traj_numbers=[5]
    # filename="traj_matrices_0.npy"
    for i in traj_numbers:
        filename="traj_matrices_{}.npy".format(i)
        analyse_trajs_from_file(filename)
        
        plt.show()