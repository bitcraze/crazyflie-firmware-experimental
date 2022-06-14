#!/usr/bin/env python
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
        self.polynomials = None
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
        self.polynomials = [Polynomial4D(
            row[0], row[1:9], row[9:17], row[17:25], row[25:33]) for row in matrix]
        
        print("matrix[:, 0].shape: ", matrix[:, 0].shape)
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

    def plot(self, timestep: float,ax=None):
        size = int(self.duration / timestep+0.5)
        print("size:", size)
        x = np.zeros(size)
        y = np.zeros(size)
        z = np.zeros(size)

        for i, t in enumerate(np.arange(0, self.duration, timestep)):
            out = self.eval(t)
            out: TrajectoryOutput
            x[i], y[i], z[i] = out.pos[0], out.pos[1], out.pos[2]
            # print(x[i], y[i], z[i])
        
        if ax==None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

        ax.plot(x, y, z)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_xlim3d(-1, 1)
        ax.set_ylim3d(-1, 1)
        ax.set_zlim3d(0.5, 1.5)

        if ax==None:
            plt.show()

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

    data = np.array(data)
    traj.load_from_matrix(data)
    traj.plot(0.01)
