#!/usr/bin/python3

"""Drone localization

Usage:
  droneloc.py ekf --trajectory <file> [--noise-std <sigma>] [--seed <seed>]
  droneloc.py ukf --trajectory <file> [--noise-std <sigma>] [--seed <seed>]

Options:
  -h --help              Show this screen.
  --trajectory <file>    File of the drone trajectory.
  --noise-std <sigma>    Add gaussian noise in mm to the anchor calculated distance,
                         e.g. standard deviation of real a DWM1001 device can vary
                         from 20mm to 200mm.
  --seed <seed>          Seed value for the random generator. 0 is the default value.

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
import filterpy.kalman
import time
import enum

class kalman_type(enum.Enum):
    EKF    = 0,
    UKF    = 1,

kf_type = kalman_type.EKF
sigma_a = 0.125
sigma_r = 0.2
m_R_scale = 1
m_Q_scale = 1
m_z_damping_factor = 1
initialized = False

dt = 0.2

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

def ekf_F_6(dt):
    F = np.array([[1, dt,  0,  0,  0, 0],
                  [0,  1,  0,  0,  0, 0],
                  [0,  0,  1, dt,  0, 0],
                  [0,  0,  0,  1,  0, 0],
                  [0,  0,  0,  0,  1, dt],
                  [0,  0,  0,  0,  0, 1]
                  ])
    return F


def ukf_F_6(x, dt):
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


def ekf_H_6(Xk, loc):
    """ compute Jacobian of H matrix for state X """

    H = np.empty([0, 6])
    for anchor in loc['anchors']:
        coords = anchor["pos"]["coords"]
        anch_x = coords[0]
        anch_y = coords[1]
        anch_z = coords[2]

        anch = np.array([anch_x, 0, anch_y, 0, anch_z, 0])
        pos = np.array([Xk[0], 0, Xk[2], 0, Xk[4], 0])
        r = np.linalg.norm(pos - anch) + 1e-6
        h_row = (pos - anch) / r
        H = np.append(H, [h_row], axis=0)

    return H


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


def kf_process(kf, loc, dt):
    global initialized

    if not initialized:
        # set cov of vel
        kf.P[1][1] = 0.1
        kf.P[3][3] = 0.1
        kf.P[5][5] = 0.1
        initialized = True

    old_x = kf.x
    old_P = kf.P
    R = np.eye(len(loc["anchors"])) * (sigma_r**2 * m_R_scale)

    if kf_type == kalman_type.EKF:
        kf.F = ekf_F_6(dt)
    kf.Q = Q_6(dt)

    kf.dim_z = len(loc['anchors'])

    kf.predict()

    z = get_measurements(loc)

    kf.R = R

    if kf_type == kalman_type.EKF:
        kf.update(z, HJacobian=ekf_H_6, Hx=Hx_6, args=loc, hx_args=loc)
    elif kf_type == kf_type.UKF:
        kf.update(z, loc=loc)

    if kf.x[4] < 0:
        kf.x[4] = np.abs(kf.x[4])

    if np.any(np.abs(kf.y) > 2):
        print("innovation is too large: ", kf.y)
        kf.x = old_x
        kf.P = old_P
        return None
    Xk = kf.x

    return [Xk[0], Xk[2], Xk[4]]


def get_anchors_coords(anchors):
    coords = []
    for anchor in anchors:
        coords += [anchor['pos']['coords']]

    return np.array(coords)


if __name__ == '__main__':
    args = docopt(__doc__)
    data_file = open(args['--trajectory'], 'r')

    if args['ekf']:
        kf_type = kalman_type.EKF
    elif args['ukf']:
        kf_type = kalman_type.UKF

    seed = 0
    if args['--seed']:
        seed = int(args['--seed'])

    np.set_printoptions(precision=3)

    np.random.seed(seed)

    noise_std = 0.0
    if args['--noise-std']:
        noise_std = float(args['--noise-std'])

    if kf_type == kalman_type.EKF:
        kf = filterpy.kalman.ExtendedKalmanFilter(dim_x=6, dim_z=4)
    elif kf_type == kalman_type.UKF:
        points = filterpy.kalman.MerweScaledSigmaPoints(n=6, alpha=.1, beta=2, kappa=0)
        kf = filterpy.kalman.UnscentedKalmanFilter(dim_x=6, dim_z=1, fx=ukf_F_6, hx=Hx_6,
                                                   dt=dt, points=points)

    kf.x = np.array([1, 0, 1, 0, 1, 0])

    # Plot anchors
    anchors_coords = get_anchors_coords(anchors)
    plt.scatter(anchors_coords[:, 0], anchors_coords[:, 1])

    n = 0
    while True:
        line = data_file.readline()
        if not line:
            break

        coords = line.split(',')
        if len(coords) != 3:
            print("Error: trajectory file is incorrect format")
            sys.exit(-1)

        coords = [float(v)/1000 for v in coords]
        loc = {
            'pos': {
                'coords':  coords,
                'qf': 100,
                'valid': True,
            },
        }

        for anchor in anchors:
            acoords = np.array(anchor['pos']['coords'])

            # Find distance from an anchor to a trajectory position
            vec = coords - acoords
            dist = np.sqrt(vec.dot(vec))
            dist += np.random.normal(0, noise_std/1000.0)

            anchor['dist'] = {
                'dist': dist,
                'qf': 100,
            }

        loc['anchors'] = anchors

        kf_process(kf, loc, dt)

        # Plot true drone position
        plt.plot(coords[0], coords[1], ',', color='g')

        # Plot filtered position
        plt.plot(kf.x[0], kf.x[2], ',', color='r')

        if n % 100 == 0:
            # Extract X (Px, Py) and P (Px, Py)
            plot_covariance_ellipse((kf.x[0], kf.x[2]),
                                    kf.P[0:3:2, 0:3:2],
                                    std=10, facecolor='g', alpha=0.3)
        n += 1


    plt.axis('equal')
    plt.show()
