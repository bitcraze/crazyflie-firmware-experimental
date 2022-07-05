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

import numpy as np
import matplotlib.pyplot as plt
from scipy.special import comb

"""
    Based on the following :
        https://www.uio.no/studier/emner/matnat/math/MAT-INF4160/h15/undervisningsmateriale/bezier1.pdf
"""

def eval_pol(f,t):
    return sum([f[i]*t**i for i in range(len(f))])
    
def plot_pol(f):
    t = np.linspace(0,1,100)
    
    x = [eval_pol(f[0,:],i) for i in t]
    y = [eval_pol(f[1,:],i) for i in t]

    plt.plot(x,y,label='Polynomial')

def convert_to_bezier(f):

    n = len(f[0])
    
    controls = np.zeros((2,n))
    
    for j in range(n):
        controls[0,j]=get_control_point(j,n,f[0,:])
        controls[1,j]=get_control_point(j,n,f[1,:])

    return controls

def get_control_point(j,n,ais):
    cj=1
    cj*=1/comb(n,j)
    athr=0
    for i in range(j):
        athr+=comb( n-i, j-i ) * ais[i]
    cj*=athr

    return cj

def P(t, X):
    '''
     xx = P(t, X)
     
     Evaluates a Bezier curve for the points in X.
     
     Inputs:
      X is a list (or array) or 2D coords
      t is a number (or list of numbers) in [0,1] where you want to
        evaluate the Bezier curve
      
     Output:
      xx is the set of 2D points along the Bezier curve
    '''
    X = np.array(X)
    N,d = np.shape(X)   # Number of points, Dimension of points
    N = N - 1
    xx = np.zeros((len(t), d))
    
    for i in range(N+1):
        xx += np.outer(B(i, N, t), X[i])
    
    return xx

def B(i, N, t):
    val = comb(N,i) * t**i * (1.-t)**(N-i)
    return val

if __name__ == "__main__":
    fx= [1 ,1 , 1,0,1]
    fy= [1 ,1 ,-1,0,1]

    f=np.array([fx,fy])

    plot_pol(f)

    
    # controls=[
    #     [0, 0],
	# [2, 4],
	# [5, 3]
    # ]

    # controls = np.array(controls)
    # controls=controls.T

    controls=convert_to_bezier(f)
    print("controls:", controls.T)

    t=np.linspace(0,1,100)

    xx=P(t,controls.T)
    
    plt.plot(controls.T[:,0], controls.T[:,1], 'ro')
    plt.plot(xx[:,0],xx[:,1],label='Bezier')
    plt.grid()
    plt.legend()
    plt.show()


    # plot_bezier(controls)

    # plt.legend()
    plt.show()

    # P0, P1, P2 = np.array([
    # 	[0, 0],
    # 	[2, 4],
    # 	[5, 3]
    # ])

    # # define bezier curve
    # P = lambda t: (1 - t)**2 * P0 + 2 * t * (1 - t) * P1 + t**2 * P2

    # # evaluate the curve on [0, 1] sliced in 50 points
    # points = np.array([P(t) for t in np.linspace(0, 1, 50)])

    # # get x and y coordinates of points separately
    # x, y = points[:,0], points[:,1]

    # # plot
    # plt.plot(x, y, 'b-')
    # plt.plot(*P0, 'r.')
    # plt.plot(*P1, 'r.')
    # plt.plot(*P2, 'r.')
    # plt.show()