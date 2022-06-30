#!/usr/bin/python3

"""Drone localization

Usage:
  droneloc.py --trajectory <file> [--fuse-velocity] [--fuse-altitude] [--post-smoother <smoother>] [--noise-std <sigma>] [--seed <seed>] [--kf-type <type>]

Options:
  -h --help                  Show this screen.
  --fuse-velocity            Fuse velocity
  --fuse-altitude            Fuse altitude
  --trajectory <file>        File of the drone trajectory.
  --post-smoother <smoother> Smoother after kalman processing, can be: savgol, uniform, gaussian
  --noise-std <sigma>        Add gaussian noise in mm to the anchor calculated distance,
                             e.g. standard deviation of real a DWM1001 device can vary
                             from 20mm to 200mm.
  --seed <seed>              Seed value for the random generator. 0 is the default value.
  --kf-type <type>           Kalman filter type. can be: "ekf6" or "ekf9 for Extended Kalman filter,
                             "ukf6" or "ukf9 for Unscented Kalman filter.

"""

from docopt import docopt
import matplotlib.pyplot as plt
import numpy as np
from numpy import array, dot
from numpy.linalg import pinv, inv
from numpy.random import randn
from filterpy.stats import plot_covariance
from scipy.linalg import block_diag
from scipy.ndimage import uniform_filter1d
from scipy.ndimage import gaussian_filter1d
from scipy.signal import savgol_filter
import filterpy.kalman
import sys
import enum
import time

class smoother_type(enum.Enum):
    SAVGOL   = 0
    UNIFORM  = 1
    GAUSSIAN = 2

class kalman_type(enum.Enum):
    UKF6   = 0
    EKF6   = 1
    UKF9   = 2
    EKF9   = 3

sigma_accel = 0.3
sigma_dist = 0.2
# Values taken from hovering drone
#sigma_vel = 0.01
#sigma_alt = 0.005
sigma_vel = 0.05
sigma_alt = 0.02

# Gate limit in standard deviations
DST_GATE_LIMIT = 15
VEL_GATE_LIMIT = 15
ALT_GATE_LIMIT = 20

R_scale = 1
Q_scale = 1
z_damping_factor = 2

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

def F_6(dt):
    F = np.array([[1, dt,  0,  0,  0, 0],
                  [0,  1,  0,  0,  0, 0],
                  [0,  0,  1, dt,  0, 0],
                  [0,  0,  0,  1,  0, 0],
                  [0,  0,  0,  0,  1, dt],
                  [0,  0,  0,  0,  0, 1]
                  ])
    return F

def ukf_F_6(x, dt):
    return F_6(dt) @ x


def F_9(dt):
    f = np.array([[1, dt, dt**2 / 2.0],
                 [0, 1, dt],
                 [0, 0, 1]])

    # block diag transition matrix
    F = block_diag(f, f, f)
    return F


def ukf_F_9(x, dt):
    return F_9(dt) @ x


def Q_6(dt):
    #Q = filterpy.common.Q_discrete_white_noise(dim=2, dt=dt, var=sigma_accel**2, block_size=3)
    q = np.array([[dt**4 / 4, dt**3 / 2],
                  [dt**3 / 2, dt**2]])
    # Take damping into considiration on Z axis
    qz = q * z_damping_factor
    Q = block_diag(q, q, qz)
    Q *= sigma_accel**2
    Q *= Q_scale

    return Q


def Q_9(dt):
    #Q = filterpy.common.Q_discrete_white_noise(dim=2, dt=dt, var=sigma_accel**2, block_size=3)
    q = np.array([[dt**4 / 4, dt**3 / 2, dt**2 / 2],
                  [dt**3 / 2, dt**2, dt],
                  [dt**2 / 2, dt, 1]])
    # Take damping into consideration on Z axis
    qz = q * z_damping_factor
    Q = block_diag(q, q, qz)
    Q *= sigma_accel**2
    Q *= Q_scale

    return Q


def Hx_6_dist(x, loc):
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

    return np.array(r_pred)


def HJacobian_6_dist(x, loc):
    H = np.empty([0, 6])
    for anchor in loc['anchors']:
        coords = anchor["pos"]["coords"]
        anch_x = coords[0]
        anch_y = coords[1]
        anch_z = coords[2]

        anch = np.array([anch_x, 0, anch_y, 0, anch_z, 0])
        pos = np.array([x[0], 0, x[2], 0, x[4], 0])
        r = np.linalg.norm(pos - anch) + 1e-6
        h_row = (pos - anch) / r
        H = np.append(H, [h_row], axis=0)

    return H


def Hx_9_dist(x, loc):
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
        pos = np.array([x[0], x[3], x[6]])
        r = np.linalg.norm(pos - anch) + 1e-6
        r_pred.append(r)

    return np.array(r_pred)


def HJacobian_9_dist(x, loc):
    H = np.empty([0, 9])
    for anchor in loc['anchors']:
        coords = anchor["pos"]["coords"]
        anch_x = coords[0]
        anch_y = coords[1]
        anch_z = coords[2]

        anch = np.array([anch_x, 0, 0, anch_y, 0, 0, anch_z, 0, 0])
        pos = np.array([x[0], 0, 0, x[3], 0, 0, x[6], 0, 0])
        r = np.linalg.norm(pos - anch) + 1e-6
        h_row = (pos - anch) / r
        H = np.append(H, [h_row], axis=0)

    return H


def Hx_6_alt(x):
    """ takes a state X and returns the measurement that would
    correspond to that state.
    """

    # Altitude, or Z coordinate
    return np.array([x[4]])


def HJacobian_6_alt(x):
    # X_6 = [Px, Vx, Py, Vy, Pz, Vz]
    # Altitude, or Z coordinate
    return np.array([0, 0, 0, 0, 1, 0]).reshape(1, 6)


def Hx_9_alt(x):
    """ takes a state X and returns the measurement that would
    correspond to that state.
    """

    # Altitude, or Z coordinate
    return np.array([x[6]])


def HJacobian_9_alt(x):
    # X_6 = [Px, Vx, Py, Vy, Pz, Vz]
    # Altitude, or Z coordinate
    return np.array([0, 0, 0, 0, 0, 0, 1, 0, 0]).reshape(1, 9)


def Hx_6_vel(x):
    """ takes a state X and returns the measurement that would
    correspond to that state.
    """

    # X_6 = [Px, Vx, Py, Vy, Pz, Vz]
    return np.array([x[1], x[3], x[5]])


def HJacobian_6_vel(x):
    # X_6 = [Px, Vx, Py, Vy, Pz, Vz]
    return np.array([[0, 1, 0, 0, 0, 0],
                     [0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 0, 1]])


def Hx_9_vel(x):
    """ takes a state X and returns the measurement that would
    correspond to that state.
    """

    # X_6 = [Px, Vx, Py, Vy, Pz, Vz]
    return np.array([x[1], x[4], x[7]])


def HJacobian_9_vel(x):
    # X_9 = [Px, Vx, Ax, Py, Vy, Ay, Pz, Vz, Az]
    return np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 1, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 1, 0]])


def get_measurements_dist(loc):
    ranges = []
    for anchor in loc['anchors']:
        dist = anchor["dist"]["dist"]
        ranges.append(dist)

    return np.array(ranges)


def get_measurements_alt(alt):
    return np.array([alt['alt']])


def get_measurements_vel(vel):
    return np.array(vel['vel'])


class drone_localization():
    kf = None
    dt = None
    process_ts = None
    post_smoother = None
    x_hist = []
    kf_type = None

    def __init__(self, kf_type, dt=None, post_smoother=None):
        if kf_type == kalman_type.UKF6:
            points = filterpy.kalman.MerweScaledSigmaPoints(n=6, alpha=.1, beta=2, kappa=0)
            kf = filterpy.kalman.UnscentedKalmanFilter(dim_x=6, dim_z=4, fx=ukf_F_6, hx=Hx_6_dist,
                                                       dt=dt, points=points)
        elif kf_type == kalman_type.UKF9:
            points = filterpy.kalman.MerweScaledSigmaPoints(n=9, alpha=.1, beta=2, kappa=0)
            kf = filterpy.kalman.UnscentedKalmanFilter(dim_x=9, dim_z=4, fx=ukf_F_9, hx=Hx_9_dist,
                                                       dt=dt, points=points)
        elif kf_type == kalman_type.EKF6:
            kf = filterpy.kalman.ExtendedKalmanFilter(dim_x=6, dim_z=4)
        elif kf_type == kalman_type.EKF9:
            kf = filterpy.kalman.ExtendedKalmanFilter(dim_x=9, dim_z=4)
        else:
            raise ValueError("incorrect kalman filter type %d." % kf_type)

        if kf_type == kalman_type.UKF6 or kf_type == kalman_type.EKF6:
            kf.x = np.array([1, 0, 1, 0, 1, 0])
        if kf_type == kalman_type.UKF9 or kf_type == kalman_type.EKF9:
            kf.x = np.array([1, 0, 0, 1, 0, 0, 1, 0, 0])

        self.kf = kf
        self.kf_type = kf_type
        self.dt = dt
        self.post_smoother = post_smoother


    def get_dt(self, loc):
        dt = 0.1
        if self.dt is None:
            #XXX
            #XXX 'ts' for dwm and parrot can be swapped in
            #XXX receive loop, so that dt goes negative.
            #XXX Temporal solution set current local time
            #XXX for each package
            #XXX
            loc['ts'] = time.time()
            if self.process_ts is not None:
                dt = loc['ts'] - self.process_ts
            self.process_ts = loc['ts']
        else:
            dt = self.dt

        if dt > 1:
            print("Warning: huge dt=%d" % dt)
            dt = 0.1

        assert(dt > 0)
        return dt


    def post_smooth(self, pos):
        if self.post_smoother is None:
            return pos

        if len(self.x_hist) > hist_window:
            self.x_hist.pop(0)

        self.x_hist.append(pos)
        if len(self.x_hist) < hist_window:
            return [0.0, 0.0, 0.0]

        if self.post_smoother == smoother_type.SAVGOL:
            pos_f = savgol_filter(self.x_hist, hist_window, 5,
                                  axis=0, mode="nearest")
        elif self.post_smoother == smoother_type.UNIFORM:
            pos_f = uniform_filter1d(self.x_hist, size=hist_window,
                                     axis=0, mode="reflect")
        elif self.post_smoother == smoother_type.GAUSSIAN:
            pos_f = gaussian_filter1d(self.x_hist, sigma=6,
                                      axis=0, mode="reflect")
        else:
            assert(0)

        return pos_f[-1]


    def is_z_coord_negative(self):
        if self.kf_type == kalman_type.EKF6 or self.kf_type == kalman_type.UKF6:
            return self.kf.x[4] < 0
        if self.kf_type == kalman_type.EKF9 or self.kf_type == kalman_type.UKF9:
            return self.kf.x[6] < 0


    def get_position(self):
        x = self.kf.x
        if self.kf_type == kalman_type.EKF6 or self.kf_type == kalman_type.UKF6:
            return np.array([x[0], x[2], x[4]])
        if self.kf_type == kalman_type.EKF9 or self.kf_type == kalman_type.UKF9:
            return np.array([x[0], x[3], x[6]])


    def get_pos_cov(self):
        if self.kf_type == kalman_type.EKF6 or self.kf_type == kalman_type.UKF6:
            return self.kf.P[0:5:2, 0:5:2]
        if self.kf_type == kalman_type.EKF9 or self.kf_type == kalman_type.UKF9:
            return self.kf.P[0:8:3, 0:8:3]


    def get_pos_cov_XY(self):
        if self.kf_type == kalman_type.EKF6 or self.kf_type == kalman_type.UKF6:
            return self.kf.P[0:3:2, 0:3:2]
        if self.kf_type == kalman_type.EKF9 or self.kf_type == kalman_type.UKF9:
            return self.kf.P[0:5:3, 0:5:3]


    def kf_process_dist(self, loc):
        old_x = self.kf.x
        old_P = self.kf.P

        R = np.eye(len(loc["anchors"])) * (sigma_dist**2 * R_scale)
        dt = self.get_dt(loc)
        z = get_measurements_dist(loc)

        if self.kf_type == kalman_type.UKF6:
            self.kf.Q = Q_6(dt)
            self.kf.predict(dt=dt)
            self.kf.update(z, R=R, hx=Hx_6_dist, loc=loc)
        elif self.kf_type == kalman_type.UKF9:
            self.kf.Q = Q_9(dt)
            self.kf.predict(dt=dt)
            self.kf.update(z, R=R, hx=Hx_9_dist, loc=loc)
        elif self.kf_type == kalman_type.EKF6:
            self.kf.R = R
            self.kf.Q = Q_6(dt)
            self.kf.F = F_6(dt)
            self.kf.predict_update(z, HJacobian=HJacobian_6_dist, Hx=Hx_6_dist, args=loc, hx_args=loc)
        elif self.kf_type == kalman_type.EKF9:
            self.kf.R = R
            self.kf.Q = Q_9(dt)
            self.kf.F = F_9(dt)
            self.kf.predict_update(z, HJacobian=HJacobian_9_dist, Hx=Hx_9_dist, args=loc, hx_args=loc)
        else:
            raise ValueError("incorrect kalman filter type %d." % self.kf_type)

        dist = self.kf.mahalanobis

        if self.is_z_coord_negative() or dist > DST_GATE_LIMIT:
            if dist > DST_GATE_LIMIT:
                print("Warning: innovation DIST is too large: ", self.kf.y)
            self.kf.x = old_x
            self.kf.P = old_P

        # Smooth calculated position
        return self.post_smooth(self.get_position())


    def kf_process_alt(self, alt):
        old_x = self.kf.x
        old_P = self.kf.P

        R = np.eye(1) * (sigma_alt**2 * R_scale)
        dt = self.get_dt(alt)
        z = get_measurements_alt(alt)

        if self.kf_type == kalman_type.UKF6:
            self.kf.Q = Q_6(dt)
            self.kf.predict(dt=dt)
            self.kf.update(z, R=R, hx=Hx_6_alt)
        elif self.kf_type == kalman_type.UKF9:
            self.kf.Q = Q_9(dt)
            self.kf.predict(dt=dt)
            self.kf.update(z, R=R, hx=Hx_9_alt)
        elif self.kf_type == kalman_type.EKF6:
            self.kf.R = R
            self.kf.Q = Q_6(dt)
            self.kf.F = F_6(dt)
            self.kf.predict_update(z, HJacobian=HJacobian_6_alt, Hx=Hx_6_alt)
        elif self.kf_type == kalman_type.EKF9:
            self.kf.R = R
            self.kf.Q = Q_9(dt)
            self.kf.F = F_9(dt)
            self.kf.predict_update(z, HJacobian=HJacobian_9_alt, Hx=Hx_9_alt)
        else:
            raise ValueError("incorrect kalman filter type %d." % self.kf_type)

        dist = self.kf.mahalanobis

        if self.is_z_coord_negative() or dist > ALT_GATE_LIMIT:
            if dist > ALT_GATE_LIMIT:
                print("Warning: innovation ALT is too large: ", self.kf.y)
                print("         Z alt [%.3f]" % (z[0]))
                print("         X alt [%.3f]" % (self.kf.x[4]))
            self.kf.x = old_x
            self.kf.P = old_P

        # Smooth calculated position
        return self.post_smooth(self.get_position())


    def kf_process_vel(self, vel):
        old_x = self.kf.x
        old_P = self.kf.P

        R = np.eye(3) * (sigma_vel**2 * R_scale)
        dt = self.get_dt(vel)
        z = get_measurements_vel(vel)

        if self.kf_type == kalman_type.UKF6:
            self.kf.Q = Q_6(dt)
            self.kf.predict(dt=dt)
            self.kf.update(z, R=R, hx=Hx_6_vel)
        elif self.kf_type == kalman_type.UKF9:
            self.kf.Q = Q_9(dt)
            self.kf.predict(dt=dt)
            self.kf.update(z, R=R, hx=Hx_9_vel)
        elif self.kf_type == kalman_type.EKF6:
            self.kf.R = R
            self.kf.Q = Q_6(dt)
            self.kf.F = F_6(dt)
            self.kf.predict_update(z, HJacobian=HJacobian_6_vel, Hx=Hx_6_vel)
        elif self.kf_type == kalman_type.EKF9:
            self.kf.R = R
            self.kf.Q = Q_9(dt)
            self.kf.F = F_9(dt)
            self.kf.predict_update(z, HJacobian=HJacobian_9_vel, Hx=Hx_9_vel)
        else:
            raise ValueError("incorrect kalman filter type %d." % self.kf_type)

        dist = self.kf.mahalanobis

        if self.is_z_coord_negative() or dist > VEL_GATE_LIMIT:
            if dist > VEL_GATE_LIMIT:
                print("Warning: innovation VEL is too large: ", self.kf.y)
                print("         Z vel [%.3f,%.3f,%.3f]" % (z[0], z[1], z[2]))
                print("         X vel [%.3f,%.3f,%.3f]" % (self.kf.x[1], self.kf.x[3], self.kf.x[5]))
            self.kf.x = old_x
            self.kf.P = old_P

        # Smooth calculated position, i.e. Px=[0], Py=[2], Pz=[4]
        return self.post_smooth(self.get_position())


def get_anchors_coords(anchors):
    coords = []
    for anchor in anchors:
        coords += [anchor['pos']['coords']]

    return np.array(coords)


# Taken from https://nbviewer.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/08-Designing-Kalman-Filters.ipynb
def NEES(x, est, P):
    err = x - est
    return err.T @ inv(P) @ err


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

    if not args['--kf-type']:
        # Default kalman type
        kf_type = kalman_type.UKF6
    elif args['--kf-type'] == 'ukf6':
        kf_type = kalman_type.UKF6
    elif args['--kf-type'] == 'ekf6':
        kf_type = kalman_type.EKF6
    elif args['--kf-type'] == 'ukf9':
        kf_type = kalman_type.UKF9
    elif args['--kf-type'] == 'ekf9':
        kf_type = kalman_type.EKF9
    else:
        print("Error: incorrect kalman filter type: '%s'" % args['--kf-type'])
        sys.exit(-1)

    droneloc = drone_localization(kf_type, dt, post_smoother=post_smoother)

    # Plot anchors
    anchors_coords = get_anchors_coords(anchors)
    plt.scatter(anchors_coords[:, 0], anchors_coords[:, 1])

    neeses = []
    n = 0
    prev_true_coords = None
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

        vel = None
        if prev_true_coords is not None:
            vec = np.array(true_coords) - np.array(prev_true_coords)
            vel = vec / dt
        prev_true_coords = true_coords

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

        if not args['--fuse-velocity'] and not args['--fuse-altitude']:
            coords = droneloc.kf_process_dist(loc)
        elif args['--fuse-velocity'] and args['--fuse-altitude']:
            if n % 3 == 0:
                coords = droneloc.kf_process_dist(loc)
            elif n % 2 == 0 and vel is not None:
                coords = droneloc.kf_process_vel({'vel': vel})
            else:
                coords = droneloc.kf_process_alt({'alt': true_coords[2]})
        elif args['--fuse-velocity']:
            if n % 2 == 0:
                coords = droneloc.kf_process_dist(loc)
            else:
                coords = droneloc.kf_process_vel({'vel': vel})
        else:
            if n % 2 == 0:
                coords = droneloc.kf_process_dist(loc)
            else:
                coords = droneloc.kf_process_alt({'alt': true_coords[2]})

        if coords is None:
            continue

        # Normalized Estimated Error Squared of the position
        x = np.array(true_coords)
        x_est = np.array(coords)
        neeses.append(NEES(x, x_est, droneloc.get_pos_cov()))

        # Plot true drone position
        plt.plot(true_coords[0], true_coords[1], ',', color='g')

        # Plot filtered position
        plt.plot(coords[0], coords[1], ',', color='r')

        if n % 100 == 0:
            # Extract X (Px, Py) and P (Px, Py)
            plot_covariance((coords[0], coords[1]),
                            droneloc.get_pos_cov_XY(),
                            std=10, facecolor='g', alpha=0.3)
        n += 1


    # Normalized Estimated Error Squared mean
    nees_mean = np.mean(neeses)

    print('mean NEES is: %.4f, ' % nees_mean, end='')

    # Where 3 is a dimension of the position
    if nees_mean < 3:
        print('PASSED')
    else:
        print('FAILED')

    plt.axis('equal')
    plt.show()
