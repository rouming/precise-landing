#!/usr/bin/python3

"""Drone localization

Usage:
  droneloc.py --trajectory <file> [--post-smoother <smoother>] [--noise-std <sigma>] [--seed <seed>]

Options:
  -h --help                  Show this screen.
  --trajectory <file>        File of the drone trajectory.
  --post-smoother <smoother> Smoother after kalman processing, can be: savgol, uniform, gaussian
  --noise-std <sigma>        Add gaussian noise in mm to the anchor calculated distance,
                             e.g. standard deviation of real a DWM1001 device can vary
                             from 20mm to 200mm.
  --seed <seed>              Seed value for the random generator. 0 is the default value.

"""

from docopt import docopt
from math import cos, sin, sqrt, atan2
import matplotlib.pyplot as plt
import numpy as np
from numpy import array, dot
from numpy.linalg import pinv
from numpy.random import randn
import random
from filterpy.stats import plot_covariance_ellipse
from scipy.linalg import block_diag
from scipy.ndimage import uniform_filter1d
from scipy.ndimage import gaussian_filter1d
from scipy.signal import savgol_filter
import filterpy.kalman
import time
import enum

class smoother_type(enum.Enum):
    SAVGOL   = 0
    UNIFORM  = 1
    GAUSSIAN = 2

sigma_a = 0.125
sigma_r = 0.2
m_R_scale = 1
m_Q_scale = 1
m_z_damping_factor = 1

dt = 0.2
hist_window = 25

anchors = [
    # Less error in covariance matrix if anchors stay apart
    {'pos': {'coords': [-3.0, -3.0, 0.0]}},
    {'pos': {'coords': [ 3.0, -3.0, 0.0]}},
    {'pos': {'coords': [ 3.0,  3.0, 0.0]}},
    {'pos': {'coords': [-3.0,  3.0, 0.0]}},

    # More error in covariance matrix if anchors close to each other
#    {'pos': {'coords': [0.0, 0.0, 0.0]}},
#    {'pos': {'coords': [1.3, 0.0, 0.0]}},
#    {'pos': {'coords': [1.3, 1.3, 0.0]}},
#    {'pos': {'coords': [0.0, 1.3, 0.0]}},

]

#
# State vector
#
# X_6 = [Px, Vx, Py, Vy, Pz, Vz]
# X_9 = [Px, Vx, Ax, Py, Vy, Ay, Pz, Vz, Az]
#
# where P - position,
#       V - velocity
#       A - acceleration
#

def F_6(x, dt):
    F = np.array([[1, dt,  0,  0,  0, 0],
                  [0,  1,  0,  0,  0, 0],
                  [0,  0,  1, dt,  0, 0],
                  [0,  0,  0,  1,  0, 0],
                  [0,  0,  0,  0,  1, dt],
                  [0,  0,  0,  0,  0, 1]
                  ])
    return F @ x


def Q_6(dt):
    #Q = filterpy.common.Q_discrete_white_noise(dim=2, dt=dt, var=sigma_a**2, block_size=3)
    q = [[dt**4 / 3, dt**3 / 2],
         [dt**3 / 2, dt**2]]
    qz = np.array(q) * m_z_damping_factor
    Q = block_diag(q, q, qz)
    Q *= sigma_a**2
    Q *= m_Q_scale

    return Q

def Hx_6(x, loc):
    """ takes a state X and returns the measurement that would
    correspond to that state.
    """

    r_pred = []
    for anchor in loc['anchors']:
        coords = anchor["pos"]["coords"]
        anch_x = coords[0]
        anch_y = coords[1]
        anch_z = coords[2]

        anch = np.array([anch_x, anch_y, anch_z])
        pos = np.array([x[0], x[2], x[4]])
        r = np.linalg.norm(pos - anch) + 1e-6
        r_pred.append(r)

    return np.array(r_pred).T

def get_measurements(loc):
    ranges = []
    for anchor in loc['anchors']:
        dist = anchor["dist"]["dist"]
        ranges.append(dist)

    return np.array(ranges).T


class drone_localization():
    kf = None
    dt = None
    process_ts = None
    post_smoother = None
    x_hist = []

    def __init__(self, dt=None, post_smoother=None):
        points = filterpy.kalman.MerweScaledSigmaPoints(n=6, alpha=.1, beta=2, kappa=0)
        kf = filterpy.kalman.UnscentedKalmanFilter(dim_x=6, dim_z=4, fx=F_6, hx=Hx_6,
                                                   dt=dt, points=points)
        kf.x = np.array([1, 0, 1, 0, 1, 0])

        # set cov of vel
        kf.P[1][1] = 0.1
        kf.P[3][3] = 0.1
        kf.P[5][5] = 0.1

        self.kf = kf
        self.dt = dt
        self.post_smoother = post_smoother


    def get_dt(self, loc):
        dt = 0.1
        if self.dt is None:
            if self.process_ts is not None:
                dt = loc["ts"] - self.process_ts
            self.process_ts = loc["ts"]
        else:
            dt = self.dt

        return dt


    def kf_process(self, loc):
        old_x = self.kf.x
        old_P = self.kf.P
        R = np.eye(len(loc["anchors"])) * (sigma_r**2 * m_R_scale)
        dt = self.get_dt(loc)

        self.kf.Q = Q_6(dt)
        self.kf.dim_z = len(loc['anchors'])
        self.kf.predict(dt=dt)

        z = get_measurements(loc)
        self.kf.R = R
        self.kf.update(z, loc=loc)

        if np.any(np.abs(self.kf.y) > 2):
            print("innovation is too large: ", self.kf.y)
            self.kf.x = old_x
            self.kf.P = old_P

        Xk = self.kf.x

        if self.post_smoother is not None:
            if len(self.x_hist) > hist_window:
                self.x_hist.pop(0)

            self.x_hist.append(Xk)
            if len(self.x_hist) < hist_window:
                return [0.0, 0.0, 0.0]

            if self.post_smoother == smoother_type.SAVGOL:
                Xk_f = savgol_filter(self.x_hist, hist_window, 5,
                                     axis=0, mode="nearest")
            elif self.post_smoother == smoother_type.UNIFORM:
                Xk_f = uniform_filter1d(self.x_hist, size=hist_window,
                                        axis=0, mode="reflect")
            elif self.post_smoother == smoother_type.GAUSSIAN:
                Xk_f = gaussian_filter1d(self.x_hist, sigma=6,
                                         axis=0, mode="reflect")
            return [Xk_f[-1, 0], Xk_f[-1, 2], Xk_f[-1, 4]]

        return [Xk[0], Xk[2], Xk[4]]


def get_anchors_coords(anchors):
    coords = []
    for anchor in anchors:
        coords += [anchor['pos']['coords']]

    return np.array(coords)


if __name__ == '__main__':
    args = docopt(__doc__)
    data_file = open(args['--trajectory'], 'r')

    if args['--post-smoother'] == 'savgol':
        post_smoother = smoother_type.SAVGOL
    elif args['--post-smoother'] == 'uniform':
        post_smoother = smoother_type.UNIFORM
    elif args['--post-smoother'] == 'gaussian':
        post_smoother = smoother_type.GAUSSIAN
    else:
        post_smoother = None

    seed = 0
    if args['--seed']:
        seed = int(args['--seed'])

    np.set_printoptions(precision=3)

    np.random.seed(seed)

    noise_std = 0.0
    if args['--noise-std']:
        noise_std = float(args['--noise-std'])

    droneloc = drone_localization(dt, post_smoother=post_smoother)

    # Plot anchors
    anchors_coords = get_anchors_coords(anchors)
    plt.scatter(anchors_coords[:, 0], anchors_coords[:, 1])

    n = 0
    while True:
        line = data_file.readline()
        if not line:
            break

        true_coords = line.split(',')
        if len(true_coords) != 3:
            print("Error: trajectory file is incorrect format")
            sys.exit(-1)

        true_coords = [float(v)/1000 for v in true_coords]
        loc = {
            'pos': {
                'coords':  true_coords,
                'qf': 100,
                'valid': True,
            },
        }

        for anchor in anchors:
            acoords = np.array(anchor['pos']['coords'])

            # Find distance from an anchor to a trajectory position
            vec = true_coords - acoords
            dist = np.sqrt(vec.dot(vec))
            dist += np.random.normal(0, noise_std/1000.0)

            anchor['dist'] = {
                'dist': dist,
                'qf': 100,
            }

        loc['anchors'] = anchors

        coords = droneloc.kf_process(loc)
        if coords is None:
            continue

        # Plot true drone position
        plt.plot(true_coords[0], true_coords[1], ',', color='g')

        # Plot filtered position
        plt.plot(coords[0], coords[1], ',', color='r')

        if n % 100 == 0:
            # Extract X (Px, Py) and P (Px, Py)
            plot_covariance_ellipse((coords[0], coords[1]),
                                    droneloc.kf.P[0:3:2, 0:3:2],
                                    std=10, facecolor='g', alpha=0.3)
        n += 1


    plt.axis('equal')
    plt.show()
