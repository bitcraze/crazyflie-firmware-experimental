# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2022 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

from typing import List, Tuple
from colorama import Fore
import numpy as np 
import matplotlib.pyplot as plt

try:
    from trajectory_gen import uav_trajectory,traj_utils
    from optim_problem import plotting
    from trajectory_gen import min_snap_traj_gen as min_snap_tg
    from trajectory_gen.uav_trajectory import Trajectory

except:
    from .trajectory_gen import uav_trajectory,traj_utils
    from .optim_problem import plotting
    from .trajectory_gen import min_snap_traj_gen as min_snap_tg
    from .trajectory_gen.uav_trajectory import Trajectory

MIN_DISTANCE_TO_INCREASE_RESOLUTION = 0.7

class MAVPlannerDebugger:
    """
    Class for visualizing and analyzing MAV sequences either produced by planning or trajectory generation.
    """
    @staticmethod
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
        
        
        trajs = MAVUtils.load_trajs_from_matrix(trajs_mat)

        MAV_sequence = MAVUtils.get_sequences_from_trajs(trajs,timestep=0.1)# load sequence from traj
        
        MAVPlannerDebugger.analyze_MAV_sequences(x0s,xrefs, MAV_sequence)
    
    @staticmethod
    def analyze_MAV_sequences(x0s,xrefs, MAV_sequence):
        """
            Finds the minimum distance between MAVs while flying and plots it vs time
            Animate the MAVs sequence at the end
        """
        if x0s is None:
            x0s,xrefs=MAVUtils.get_xos_xrefs_from_sequences(MAV_sequence)

        print(Fore.GREEN+"x0s :")
        for i,x0 in enumerate(x0s):
            print(i,": {:.2f} {:.2f} {:.2f}".format(x0[0],x0[1],x0[2]))

        print(Fore.GREEN+"xrefs :")
        for i,xref in enumerate(xrefs):
            print(i,": {:.2f} {:.2f} {:.2f}".format(xref[0],xref[1],xref[2]))
        print(Fore.RESET)

        MAV_sequence = MAVUtils.align_sequences(MAV_sequence)

        #check shortest distance between MAVs
        dists=MAVUtils.calculate_distances(MAV_sequence)

        #find index of shortest distance
        min_dist_index=np.unravel_index(dists.argmin(), dists.shape)
        MAV_pairs=[min_dist_index[0],min_dist_index[1]]
        #plot distance between MAVs
        MAVPlannerDebugger.plot_distance_in_MAV_pairs(dists,MAV_pairs)

        #Plotting
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # plotting.animate3D(MAV_sequence,ax=ax,fig=fig)

        plotting.plotGridSpec(MAV_sequences=MAV_sequence)
        plt.show()
    
    @staticmethod
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


class MAVUtils:
    """
    Class for MAV utilities needed for planning and trajectory generation.
    """
    @staticmethod
    def get_sequences_from_trajs(trajs:List[uav_trajectory.Trajectory] , timestep:float = 0.1 )->List[List[np.array]]:
        """Returns MAV sequences from list of Trajectory objects"""
        MAV_sequence=[]
        
        for i in range(len(trajs)):
            # trajs[i].plot(timestep=0.1,ax=ax,label=str(i))
            positions,traj_time = trajs[i].get_path(timestep=timestep)
            x,y,z=positions[0],positions[1],positions[2]
            seq=[]
            for j in range(len(x)):
                seq.append([x[j],y[j],z[j]])
            seq=np.array(seq)
            print(seq.shape)
            MAV_sequence.append(seq)
        return MAV_sequence
    
    @staticmethod
    def align_sequences(MAV_sequence):
        """Aligns MAV sequences because sometimes they are not the same size """

        lengths=[len(i) for i in MAV_sequence]
        min_len=min(lengths)
        print("min_len:", min_len)
        for i in range(len(MAV_sequence)):
            MAV_sequence[i]=MAV_sequence[i][:min_len]
        MAV_sequence=np.array(MAV_sequence)
        return MAV_sequence
    
    @staticmethod
    def load_trajs_from_matrix(trajs_mat)->List[uav_trajectory.Trajectory]:
        """Loads trajs from matrix into list of Trajectory objects"""
        trajs:List[uav_trajectory.Trajectory]=[]
        
        for i in range(len(trajs_mat)):
            traj=uav_trajectory.Trajectory()
            traj.load_from_matrix(trajs_mat[i])
            trajs.append(traj)
        return trajs

    @staticmethod
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

    @staticmethod
    def get_xos_xrefs_from_sequences(MAV_sequences)->Tuple[List[np.ndarray],List[np.ndarray]]:
        """Returns x0s and xrefs of MAVs"""
        x0s=[ i[0] for i in MAV_sequences]
        xrefs=[ i[-1] for i in MAV_sequences]
        
        return x0s, xrefs


    @staticmethod
    def downsample_waypoints(waypoints:List[List[float]],downsample_step,extra_times_to_add:List[int])->List[List[float]]:
        """
        Downsamples the waypoints using the given step and includes the extra times(horizon ticks) provided.
        
        @param waypoints: List of waypoints for MAV (HORIZON_N,XYZ).
        @param downsample_step: Step of the downsampling.
        @param extra_times_to_add: List of extra times(indices of waypoints array) to add to the waypoints.
        
        @return: List of downsampled waypoints (downsampled_wps_number,XYZ).
        """

        new_waypoints=[]
        downsampled_times= range(0,len(waypoints),downsample_step)
        times_added=[]
        for ii in range(len(downsampled_times) ):
            # print("ii: {}".format(downsampled_times[ii]))
            new_waypoints.append(waypoints[downsampled_times[ii],:])
            times_added.append(downsampled_times[ii])
            
            while len(extra_times_to_add)>0  and (downsampled_times[ii]<extra_times_to_add[0] and extra_times_to_add[0]<downsampled_times[ii+1] )  :
                index=extra_times_to_add[0]
                
                # print("Adding extra waypoint at time {}".format(index))
                new_waypoints.append(waypoints[index,:])
                times_added.append(index)
                extra_times_to_add.pop(0)

        # print("Times added: {}".format(times_added))
        return new_waypoints

    @staticmethod
    def find_times_of_closest_distances(dists:List[List[List[float]]],MAV_index:int,number_of_times:int=3):
        """
        Finds the times of closest distance between MAVs
        @param dists: distance matrix in shape (N_MAVs,N_MAVs,horizon_length)
        @param MAV_index: index of MAV to find closest distance to
        @param number_of_times: number of times to find closest distance to
        @return: list of times of closest distance
        """
        min_distances_dict={}
        
        for i in range(len(dists)):
            if i==MAV_index:
                continue

            #distances between MAV_index and i
            distances=dists[MAV_index,i,:]
            #find the 3 minimum distances and their indices
            min_distances_indices=np.argsort(distances)[:3]
            #make a dictionary of the indices and their distances
            for j in range(len(min_distances_indices)):
                min_distances_dict[min_distances_indices[j]]=distances[j]
        
        #sort the dictionary by distance
        min_distances_dict=sorted(min_distances_dict.items(), key=lambda x: x[1])
        
        #add times that occur at the minimum distances being below the threshold
        for i in range(number_of_times):
            if min_distances_dict[i][1]>MIN_DISTANCE_TO_INCREASE_RESOLUTION:
                number_of_times -= 1

        #get the times of the 3 closest distances
        times=[]
        while len(times)<number_of_times:
            if min_distances_dict[0][0] not in times:        
                times.append(min_distances_dict[0][0])

            min_distances_dict.pop(0)
        
        return times

    @staticmethod
    def compare_planned_with_generated(MAV_sequences:List[List[List[float]]], trajs:List[Trajectory],waypoints_used_for_gen:List[List[List[float]]],MAVs_to_plot:List[int]=[0,1]):
        """Plots the generated trajectories and the waypoints of the planning in order to compare and debug."""
        
        for i,tr in enumerate(trajs):
            if i in MAVs_to_plot:
                waypoints=MAV_sequences[i]
                min_snap_tg.TrajectoryDebugger.debug_traj_generation(waypoints,tr,waypoints_used_for_gen[i],plt_title='Drone {}'.format(i))
