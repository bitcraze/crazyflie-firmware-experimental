#!/usr/bin/env python
from typing import List, Tuple
import numpy as np
import matplotlib.pyplot as plt


def normalize(v):
    norm = np.linalg.norm(v)
    assert norm > 0
    return v / norm


class Polynomial:
    def __init__(self, p):
        self.p = p

    # evaluate a polynomial using horner's rule
    def eval(self, t):
        assert t >= 0
        x = 0.0
        for i in range(0, len(self.p)):
            x = x * t + self.p[len(self.p) - 1 - i]
        return x

    # compute and return derivative
    def derivative(self):
        return Polynomial([(i+1) * self.p[i+1] for i in range(0, len(self.p) - 1)])

    def pol_coeffs_at_t(self, t):
        # calculate the coefficients of the polynomial at time t
        assert t >= 0
        coeffs = np.zeros(len(self.p))

        for i in range(0, len(self.p)):
            coeffs[i] = self.p[i] * (t**i)

        return coeffs

class PiecewisePolynomial():
    """
    Piece-wise polynomial.
    THis class represent a piece-wise polynomail where the used polynomial
    to calculate the evaluation depends on time.
    Parameters
    ----------
    pols : list of Polynomial classes
        All the polynomials used.
    time_setpoints: list of floats
        The time points used to decide which polynomial to use
    """

    def __init__(self, pols: list, time_durations: list):
        self.pols = pols
        self.nOfPols = len(pols)

        # self.time_setpoints = np.zeros(self.nOfPols+1)
        self.time_durations = time_durations

    def eval(self, t):
        assert t >= 0
        # print("Durations: ", self.time_durations)
        # Evaluate resylt at time t.
        t_counting = 0
        for i in range(self.nOfPols+1):

            if i >= self.nOfPols:  # t bigger than whole duration
                # print("Calling pol {} with t:{}".format(i, t-sum(self.time_durations[:-1])))
                return self.pols[-1].eval(t-sum(self.time_durations[:-1]))

            if t < t_counting+self.time_durations[i]:
                # print("Calling pol {} with t:{}".format(i, t-t_counting))
                return self.pols[i].eval(t-t_counting)

            t_counting = t_counting + self.time_durations[i]


class TrajectoryOutput:
    def __init__(self):
        self.pos = None    # position [m]
        self.vel = None    # velocity [m/s]
        self.acc = None    # acceleration [m/s^2]
        self.omega = None  # angular velocity [rad/s]
        self.yaw = None    # yaw angle [rad]


# 4d single polynomial piece for x-y-z-yaw, includes duration.
class Polynomial4D:
    def __init__(self, duration, px, py, pz, pyaw):
        self.duration = duration
        self.px = Polynomial(px)
        self.py = Polynomial(py)
        self.pz = Polynomial(pz)
        self.pyaw = Polynomial(pyaw)

    # compute and return derivative
    def derivative(self):
        return Polynomial4D(
            self.duration,
            self.px.derivative().p,
            self.py.derivative().p,
            self.pz.derivative().p,
            self.pyaw.derivative().p)

    def eval(self, t):
        result = TrajectoryOutput()
        # flat variables
        result.pos = np.array(
            [self.px.eval(t), self.py.eval(t), self.pz.eval(t)])
        result.yaw = self.pyaw.eval(t)

        # 1st derivative
        derivative = self.derivative()
        result.vel = np.array(
            [derivative.px.eval(t), derivative.py.eval(t), derivative.pz.eval(t)])
        dyaw = derivative.pyaw.eval(t)

        # 2nd derivative
        derivative2 = derivative.derivative()
        result.acc = np.array([derivative2.px.eval(
            t), derivative2.py.eval(t), derivative2.pz.eval(t)])

        # 3rd derivative
        derivative3 = derivative2.derivative()
        jerk = np.array([derivative3.px.eval(
            t), derivative3.py.eval(t), derivative3.pz.eval(t)])

        thrust = result.acc + np.array([0, 0, 9.81])  # add gravity

        z_body = normalize(thrust)
        x_world = np.array([np.cos(result.yaw), np.sin(result.yaw), 0])
        y_body = normalize(np.cross(z_body, x_world))
        x_body = np.cross(y_body, z_body)

        jerk_orth_zbody = jerk - (np.dot(jerk, z_body) * z_body)
        h_w = jerk_orth_zbody / np.linalg.norm(thrust)

        result.omega = np.array(
            [-np.dot(h_w, y_body), np.dot(h_w, x_body), z_body[2] * dyaw])
        return result


class Trajectory:
    def __init__(self):
        self.polynomials :List[Polynomial4D] = None
        self.duration = None

    def n_pieces(self):
        return len(self.polynomials)

    def loadcsv(self, filename, skip_first_row):
        skip_rows = 1 if skip_first_row else 0

        data = np.loadtxt(filename, delimiter=",",
                          skiprows=skip_rows, usecols=range(33))

        print(data.shape)
        if len(data) == 33:
            print("Reshaping data to (1,33)")
            data = data.reshape(1, 33)

        self.polynomials = [Polynomial4D(
            row[0], row[1:9], row[9:17], row[17:25], row[25:33]) for row in data]

        self.duration = np.sum(data[:, 0])

    def load_from_matrix(self, matrix):
        if type(matrix) is not np.ndarray:
            matrix = np.array(matrix)
            
        self.polynomials = [Polynomial4D(
            row[0], row[1:9], row[9:17], row[17:25], row[25:33]) for row in matrix]
        
        # print("matrix[:, 0].shape: ", matrix[:, 0].shape)
        self.duration = np.sum(matrix[:, 0])

    def load_from_pol_segments(self, pols_coeffs):
      segments_number = len(pols_coeffs[0])
      matrix = np.zeros((segments_number, 4*8+1))

      for i in range(segments_number):
          matrix[i, 0] = i*1
          matrix[i:i+1, 1:9] = pols_coeffs[0][i].p.T
          matrix[i, 9:17] = pols_coeffs[1][i].p.T
          matrix[i, 17:25] = pols_coeffs[2][i].p.T
          matrix[i, 25:33] = pols_coeffs[3][i].p.T
      
      matrix = np.array(matrix)
      self.load_from_matrix(matrix)

    def load_from_Bezier_segmnents(self, segments):
        # vertical concatenation of all segments
        matr_shape = (len(segments), 4*8+1)
        all_segments = np.zeros(matr_shape)

        for i, s in enumerate(segments):
            all_segments[i, :] = np.concatenate(
                ([s._scale], s._polys[0], s._polys[1], s._polys[2], s._polys[3]))

        self.load_from_matrix(all_segments)

    def eval(self, t):
        assert t >= 0
        assert t <= self.duration

        current_t = 0.0
        for p in self.polynomials:
            if t <= current_t + p.duration:
                return p.eval(t - current_t)
            current_t = current_t + p.duration

    def savecsv(self, filename):
        data = self.get_matrix()
        np.savetxt(filename, data, fmt="%.6f", delimiter=",",
                   header="duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7")

    def get_matrix(self):
        data = np.empty((len(self.polynomials), 8*4+1))
        for i, p in enumerate(self.polynomials):
            data[i, 0] = p.duration
            data[i, 1:9] = p.px.p
            data[i, 9:17] = p.py.p
            data[i, 17:25] = p.pz.p
            data[i, 25:33] = p.pyaw.p
        return data

    def plot(self, timestep: float,ax=None,label=None):
        pos,traj_time = self.get_path(timestep)
        x,y,z = pos[0],pos[1],pos[2]

        if ax==None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

        ax.plot(x, y, z,label=label)
        #plot start and end point
        ax.scatter(x[0], y[0], z[0], color='green')
        ax.scatter(x[-1], y[-1], z[-1], color='red')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_xlim3d(-1, 1)
        ax.set_ylim3d(-1, 1)
        ax.set_zlim3d(0.5, 1.5)

        if ax==None:
            plt.show()

    def get_path(self,timestep: float)->Tuple[np.ndarray,np.ndarray]:
        """
        @param timestep: time step in seconds  
        @return: tuple (positions,time)
                positions: 2D array of shape (n_points,3)
                time: 1D array of shape (n_points)
        """
        size = int(self.duration / timestep+0.5)
        x = np.zeros(size)
        y = np.zeros(size)
        z = np.zeros(size)
        
        time=np.arange(0, self.duration, timestep)
        for i, t in enumerate(time):
            out = self.eval(t)
            out: TrajectoryOutput
            if i==len(x):
                #delete last element
                x = x[:-1]
                y = y[:-1]
                z = z[:-1]
                continue
            
            x[i], y[i], z[i] = out.pos[0], out.pos[1], out.pos[2]
        
        positions = np.array([x,y,z])

        return positions,time

class Waypoint():
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    # constants
    WP_TYPE_X = 0
    WP_TYPE_Y = 1
    WP_TYPE_Z = 2
    WP_TYPE_YAW = 3

    def getType(self, type):
        if (type == Waypoint.WP_TYPE_X):
            return self.x
        elif (type == Waypoint.WP_TYPE_Y):
            return self.y
        elif (type == Waypoint.WP_TYPE_Z):
            return self.z
        elif (type == Waypoint.WP_TYPE_YAW):
            return self.yaw
        else:
            print("Sorry, invalid type")

    def get_pos(self):
        return self.x, self.y, self.z
class Point_time():
    # Class that combines waypoint and desired time for trajectory generation
    def __init__(self, wp: Waypoint, t: float):
        self.wp = wp
        self.t = t


class Point_time1D():
    # Class that combines waypoint and desired time for trajectory generation
    def __init__(self, wp: float, t: float):
        self.wp = wp
        self.t = t


if __name__ == "__main__":
    traj = Trajectory()
    # duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
    data = [
        [1.000000, 0.000041, 0.000000, -0.000000, -0.289936, 0.075625, 0.213991, 0.138341, -0.138147, -0.000069, -0.000000, -0.000000, 1.933871, 0.109732, -0.870394, -0.780097,
            0.607033, 1.000518, 0.000000, 0.000000, -0.003444, 0.000062, 0.001769, 0.001427, -0.001213, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
        [1.000000, -0.000086, 0.365658, 0.897868, 0.084135, -0.164938, -0.210947, -0.096047, 0.124430, 1.000076, 1.457219, -1.197699, -0.686929, 0.040958, 0.335767, 0.288162, -
            0.237604, 0.999120, -0.001166, 0.003667, 0.000585, -0.000799, -0.000995, -0.000454, 0.000619, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
        [1.000000, 1.000075, 0.994048, -0.776486, -0.250959, -0.107534, 0.079546, 0.132473, -0.071230, 0.999950, -1.090554, -0.322316, 0.281684, 0.155308, 0.042165, -0.057692, -
            0.008461, 1.000576, 0.001361, -0.003134, 0.000020, 0.000505, 0.000613, 0.000271, -0.000391, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
        [1.000000, 0.999933, -1.047978, -0.887843, 0.270769, 0.492080, 0.499279, 0.169886, -0.496096, 0.000084, -0.463456, 0.833171, -0.125414, -0.150346, -0.178943, -0.075405,
            0.160269, 0.999821, -0.000874, 0.001939, -0.000096, -0.000482, -0.000575, -0.000245, 0.000508, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    ]

    # duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
    generated = [
    # duration,x^0,x^1,x^2,x^3,x^4,x^5,x^6,x^7,y^0,y^1,y^2,y^3,y^4,y^5,y^6,y^7,z^0,z^1,z^2,z^3,z^4,z^5,z^6,z^7,yaw^0,yaw^1,yaw^2,yaw^3,yaw^4,yaw^5,yaw^6,yaw^7
        [0.607070,0.023822,0.000000,0.000000,0.000000,5.677483,-12.861443,10.477816,-3.033793,0.015614,0.000000,0.000000,0.000000,3.697061,-8.348798,6.781650,-1.958805,1.003822,0.000000,0.000000,0.000000,0.919415,-2.091987,1.711222,-0.497150,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,-0.000000],
        [0.607070,0.166753,0.467223,-0.127178,-1.150545,0.804131,1.823992,-2.414253,0.816415,0.109299,0.307552,-0.079086,-0.757424,0.506325,1.193214,-1.542268,0.514301,1.026755,0.074508,-0.021896,-0.183447,0.136266,0.293452,-0.401411,0.138293,0.000000,-0.000000,-0.000000,0.000000,0.000000,0.000000,0.000000,-0.000000],
        [0.607070,0.309685,0.090744,0.131409,0.602434,-0.612568,-0.651324,1.055093,-0.371136,0.202985,0.054760,0.079094,0.413349,-0.370340,-0.444090,0.643248,-0.212801,1.049688,0.016202,0.023519,0.090183,-0.109121,-0.098380,0.186261,-0.070353,0.000000,-0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
        [0.607286,0.452616,0.317774,-0.076234,-0.328604,0.336852,0.319465,-0.522046,0.180314,0.296670,0.218454,-0.038545,-0.255909,0.171252,0.251975,-0.261049,0.063164,1.072621,0.047497,-0.015980,-0.038358,0.071022,0.035585,-0.112703,0.048397,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
        [0.592681,0.595394,0.185237,0.033668,0.187795,-0.167606,-0.186241,0.244469,-0.073027,0.390738,0.106084,0.015484,0.220733,-0.012622,-0.210025,0.007461,0.061352,1.095400,0.034613,0.005003,-0.009046,-0.065026,-0.000256,0.093031,-0.048555,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
        [0.549856,0.730523,0.253680,-0.033088,-0.161220,0.036478,0.144411,-0.058505,-0.011896,0.489989,0.238850,0.052130,-0.250921,-0.148646,0.269080,0.261996,-0.230705,1.110531,0.002790,-0.051045,0.013569,0.070599,-0.027602,-0.108411,0.069238,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
        [0.600705,0.842001,0.141378,-0.085560,0.123041,0.098957,-0.124137,-0.104294,0.086479,0.599040,0.127103,-0.068377,0.368680,0.436942,-0.331350,-0.625988,0.415640,1.102010,-0.026006,-0.022046,-0.053543,-0.094078,0.054341,0.158086,-0.099364,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000],
        [0.828262,0.923440,0.156260,0.019688,-0.145150,-0.182312,0.155285,0.259346,-0.198118,0.743930,0.450113,0.283868,-0.596685,-0.793230,0.562075,1.121753,-0.812234,1.063450,-0.115136,-0.058880,0.148996,0.170966,-0.128837,-0.259732,0.189417,0.000000,0.000000,0.000000,0.000000,-0.000000,-0.000000,0.000000,-0.000000],
    ]
    data=generated

    data = np.array(data)
    traj.load_from_matrix(data)
    traj.plot(0.01)
    plt.show()
