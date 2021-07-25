#!/usr/bin/env python

import math
import socket
import struct
import select
import os
import re

import numpy as np
from scipy.optimize import least_squares
from scipy.optimize import minimize
from simple_pid import PID
import collections
import config as cfg

from scipy.signal.signaltools import wiener
from scipy.signal import savgol_filter
#X_f, Y_f, Z_f = wiener(np.array([X_lse, Y_lse, Z_lse]))

import time


dwm_sock    = None
parrot_sock = None
plot_sock   = None

dwm_loc     = None
parrot_data = None

X_lse = collections.deque()
Y_lse = collections.deque()
Z_lse = collections.deque()
T = collections.deque()

X_filtered = []
Y_filtered = []
Z_filtered = []

hist_len_sec = 100000

total_pos = 0
total_calc = 0

#
# Welford's online algorithm
# https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
#
class welford_state:
    count = 0
    mean = 0
    m2 = 0

    def avg_welford(self, new_value):
        self.count += 1
        delta = new_value - self.mean
        self.mean += delta / self.count
        delta2 = new_value - self.mean
        self.m2 += delta * delta2

        return self.mean;

class avg_rate:
    welford = welford_state()
    ts = 0.0

    def __call__(self):
        rate = 0
        now = time.time()
        if self.ts:
            rate = 1.0 / (now - self.ts)
            rate = self.welford.avg_welford(rate)
        self.ts = now

        return rate

class drone_navigator():
    # Average update rate
    avg_rate = avg_rate()

    # PID tuning files, format is: float, float, float
    pid_x_tuning_file = "./pid_x.tuning"
    pid_y_tuning_file = "./pid_y.tuning"

    # Default PID config
    default_pid_components = (10, 30, 0.1)
    default_pid_limits = (-100, 100)

    def __init__(self, target_x, target_y):
        # Create commands sock
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        components = self.default_pid_components

        # Set desired landing coordinates
        x_pid = PID(Kp=components[0], Ki=components[1], Kd=components[2],
                    setpoint=target_x,
                    proportional_on_measurement=False)
        y_pid = PID(Kp=components[0], Ki=components[1], Kd=components[2],
                    setpoint=target_y,
                    proportional_on_measurement=False)

        # Control coeff limits
        x_pid.output_limits = self.default_pid_limits
        y_pid.output_limits = self.default_pid_limits

        self.start_time = time.time()

        self.sock = sock
        self.x_pid = x_pid
        self.y_pid = y_pid

    def _send_command(self, roll, pitch, yaw, throttle):
        # 4x signed chars
        buf = struct.pack("bbbb", int(roll), int(pitch), int(yaw), int(throttle))
        self.sock.sendto(buf, (cfg.UDP_COMMANDS_IP, cfg.UDP_COMMANDS_PORT))

    def _pid_tuning(self, pid, tuning_file):
        tunings = self.default_pid_components
        limits = self.default_pid_limits

        if os.path.exists(tuning_file):
            with open(tuning_file, "r") as file:
                line = file.readline()
                components = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", line)
                if len(components) >= 3:
                    # To floats
                    components = [float(f) for f in components]
                    tunings = components[0:3]
                    if len(components) >= 5:
                        print(limits)
                        limits = components[3:5]

        pid.tunings = tunings
        pid.output_limits = limits

    def navigate_drone(self, drone_x, drone_y):
        self._pid_tuning(self.x_pid, self.pid_x_tuning_file)
        self._pid_tuning(self.y_pid, self.pid_y_tuning_file)

        control_x = self.x_pid(drone_x)
        control_y = self.y_pid(drone_y)

        # Calculate update rate
        rate = self.avg_rate()

        # Parrot accepts in signed percentage, i.e. [-100, 100] range
        roll = int(control_x)
        pitch = int(control_y)
        self._send_command(roll, pitch, 0, 0)

        return roll, pitch, rate


def create_dwm_sock():
    # Create sock and bind
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((cfg.MCAST_GRP, cfg.MCAST_PORT))

    # Join group
    mreq = struct.pack("4sl", socket.inet_aton(cfg.MCAST_GRP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.setblocking(0)

    return sock

def create_parrot_sock():
    # Create parrot sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((cfg.UDP_TELEMETRY_IP, cfg.UDP_TELEMETRY_PORT))
    sock.setblocking(0)

    return sock

def create_plot_sock():
    # Create plot sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    return sock

def send_plot_data(sock, x, y, z, parrot_alt, ts, nr_anchors, roll, pitch, rate, navigator):
    x_pid = navigator.x_pid
    y_pid = navigator.y_pid

    # 1 double, 17 floats, 3 int32
    buf = struct.pack("dfffffffffffffffffiii",
                      ts, x, y, z, parrot_alt, rate,
                      x_pid.Kp, x_pid.Ki, x_pid.Kd,
                      x_pid.components[0], x_pid.components[1], x_pid.components[2],
                      y_pid.Kp, y_pid.Ki, y_pid.Kd,
                      y_pid.components[0], y_pid.components[1], y_pid.components[2],
                      roll, pitch, nr_anchors)
    sock.sendto(buf, (cfg.UDP_PLOT_IP, cfg.UDP_PLOT_PORT))

def receive_dwm_location_from_sock(sock):
    # Location header
    fmt = "iiihhiii"
    sz = struct.calcsize(fmt)
    buf = sock.recv(sz, socket.MSG_PEEK)

    # Unpack location header
    (x, y, z, pos_qf, pos_valid, ts_sec, ts_usec, nr_anchors) = struct.unpack(fmt, buf)

    print("ts:%ld.%06ld [%d,%d,%d,%u] " % (ts_sec, ts_usec, x, y, z, pos_qf), end='')

    location = {
        'calc_pos': {
            'x':  float(x) / 1000,
            'y':  float(y) / 1000,
            'z':  float(z) / 1000,
            'qf': pos_qf,
            'valid': pos_valid,
        },
        'ts': float("%ld.%06ld" % (ts_sec, ts_usec)),
        'anchors': [],
    }

    # Skip size of the location header
    off = sz

    # Read the whole location packet
    fmt = "iiihhihh"
    sz = sz + struct.calcsize(fmt) * nr_anchors
    buf = sock.recv(sz)

    # For each anchor
    for i in range(0, nr_anchors):
        (x, y, z, pos_qf, pos_valid, dist, addr, dist_qf) = struct.unpack_from(fmt, buf, off)
        off += struct.calcsize(fmt)

        print("#%u) a:0x%08x [%d,%d,%d,%u] d=%u,qf=%u " % (i, addr, x, y, z, pos_qf, dist, dist_qf), \
              end='')

        anchor = {
            'pos': {
                'x':  float(x) / 1000,
                'y':  float(y) / 1000,
                'z':  float(z) / 1000,
                'qf': pos_qf,
                'valid': pos_valid,
            },
            'dist': {
                'dist': float(dist) /1000,
                'addr': addr,
                'qf': dist_qf
            },
        }

        location['anchors'].append(anchor)

    print('')

    return location

def receive_parrot_data_from_sock(sock):
    fmt = "iiffff"
    sz = struct.calcsize(fmt)
    buf = sock.recv(sz)
    sec, usec, alt, roll, pitch, yaw = struct.unpack(fmt, buf)

    parrot_data = {
        'ts':    float("%ld.%06ld" % (sec, usec)),
        'alt':   alt,
        'roll':  roll,
        'pitch': pitch,
        'yaw':   yaw
    }

    print("parrot_data: ts=%.6f alt=%f roll=%f pitch=%f yaw=%f" % \
        (parrot_data['ts'], parrot_data['alt'], parrot_data['roll'], parrot_data['pitch'], parrot_data['yaw']))

    return parrot_data

def get_dwm_location_or_parrot_data():
    global dwm_sock, parrot_sock, dwm_loc, parrot_data

    if dwm_sock is None:
        dwm_sock = create_dwm_sock()
    if parrot_sock is None:
        parrot_sock = create_parrot_sock()

    timeout = 0
    received = False

    # Suck everything from the socket, we need really up-to-date data
    while (True):
        rd, wr, ex = select.select([dwm_sock, parrot_sock], [], [], timeout)
        if 0 == len(rd):
            if received:
                break
            else:
                # Wait for data
                timeout = None
                continue

        received = True
        timeout = 0

        if dwm_sock in rd:
            dwm_loc = receive_dwm_location_from_sock(dwm_sock)
        if parrot_sock in rd:
            parrot_data = receive_parrot_data_from_sock(parrot_sock)

    return dwm_loc, parrot_data

def find_anchor_by_addr(location, addr):
    for anchor in location['anchors']:
        if anchor['dist']['addr'] == addr:
            return anchor

    return None

def func1(X, loc):
    sum = 0
    for anch in loc["anchors"]:
        anchor_pos = np.array([anch["pos"]["x"], anch["pos"]["y"], anch["pos"]["z"]])
        dist = anch["dist"]["dist"]
        sum += (np.linalg.norm(X - anchor_pos) - dist) ** 2

    return sum

# grad is probably wrong, check it later
# btw it works fine without it
# this shit probably good if we have some anchors with non zero z coordinate
def grad_func1(X, la, lb, lc, ld):
    na = np.linalg.norm(X - A)
    nb = np.linalg.norm(X - B)
    nc = np.linalg.norm(X - C)
    nd = np.linalg.norm(X - D)

    ret = 2 * (1 - la / na) * (X - A) + 2 * (1 - lb / nb) * (X - B) + \
          2 * (1 - lc / nc) * (X - C) + 2 * (1 - ld / nd) * (X - D)

    return ret

def func2(X, la, lb, lc, ld):
    ret = np.linalg.norm(X - A) - la + np.linalg.norm(X - B) - lb  + \
            np.linalg.norm(X - C) - lc + np.linalg.norm(X - D) - ld
    return ret

def dfunc2(X, la, lb, lc ,ld):
    ret = 2 * (X - A) + 2 * (X - B) + 2 * (X - C) + 2 * (X - D)
    return ret

def func(X, la, lb, lc, ld):
    ret = np.array([np.linalg.norm(X - A) ** 2 - la ** 2,
                    np.linalg.norm(X - B) ** 2 - lb ** 2,
                    np.linalg.norm(X - C) ** 2 - lc ** 2,
                    np.linalg.norm(X - D) ** 2 - ld ** 2])
    return ret

def jac(X, la, lb, lc, ld):
    J = np.empty((4, 3))
    J[0, :] = 2 * (X - A)
    J[1, :] = 2 * (X - B)
    J[2, :] = 2 * (X - C)
    J[3, :] = 2 * (X - D)

    return J


assigned = False
def calc_pos(X0, loc):

    # all are the experiments
    #res = least_squares(func, X0, loss='soft_l1', jac=jac, bounds=([-3, -3, 0.0], [3, 3, 3]), args=(la, lb, lc, ld), verbose=1)

    #to make smooth path, but in general this is shit i think
    # lowb = X0-0.2
    # if lowb[2] < 0:
    #     lowb[2] = 0
    # upb = X0+0.2
    lowb = [-np.inf, -np.inf, 0]
    upb = [np.inf, np.inf, np.inf]

    start = time.time()
    res = least_squares(func1, X0, loss='cauchy', f_scale=0.001, bounds=(lowb, upb),
    #res = least_squares(func1, X0, bounds=(lowb, upb),
                        #args=(la, lb, lc, ld), verbose=1)
                        args=[loc], verbose=0)

    ##also decent and fast
    # res = minimize(func1, X0, method="L-BFGS-B", bounds=[(-math.inf, math.inf), (-math.inf, math.inf), (0, math.inf)],
    #                #options={'ftol': 1e-4, 'disp': True}, args=(la, lb, lc, ld))
    #                options={'ftol': 1e-4,'eps' : 1e-4, 'disp': False}, args=loc)

    # res = minimize(func1, X0, method="SLSQP", bounds=[(-math.inf, math.inf), (-math.inf, math.inf), (0, math.inf)],
    #           options={ 'ftol': 1e-5, 'disp': True}, args=loc)
    #           #options={'ftol': 1e-5,'eps' : 1e-8, 'disp': False}, args=loc)

    stop = time.time()
    print("calc time {}".format(stop - start))
    # experiments
    #res = minimize(func1, X0, method='BFGS', options={'xatol': 1e-8, 'disp': True}, args=(la, lb, lc, ld))
    #res = optimize.shgo(func1, bounds=[(-10, 10), (-10, 10), (0, 10)], args=(la, lb, lc, ld),n=200, iters=5, sampling_method='sobol')
    #res = minimize(func1, X0, method='BFGS', options={'disp': True}, args=(la, lb, lc, ld))
    return res.x

navigator = drone_navigator(cfg.LANDING_X, cfg.LANDING_Y)
plot_sock = create_plot_sock()

while True:
    print(">> get location from 4 anchors")

    loc, parrot_data = get_dwm_location_or_parrot_data()
    # if loc is None or len(loc['anchors']) != 4:
    #     continue

    print(">> got calculated position from the engine")

    x = loc['calc_pos']['x']
    y = loc['calc_pos']['y']
    z = loc['calc_pos']['z']
    qf = loc['calc_pos']['qf']
    ts = loc["ts"]

    if x == 0 and y == 0 and z == 0 and qf == 0:
        # Skip now, please FIXME math guru!
        continue

    parrot_alt = 0

    print(">> get distances")

    if not assigned:
        X0 = np.abs(np.array([x, y, z]))
        assigned = True

    X_calc = calc_pos(X0, loc)

    X0 = X_calc
    X_lse.append(X_calc[0])
    Y_lse.append(X_calc[1])
    #Z_lse.append(X_calc[2])
    if parrot_data is not None and (ts - parrot_data["ts"] < 2):
        parrot_alt = parrot_data["alt"]
        Z_lse.append(parrot_data["alt"])
    else:
        Z_lse.append(X_calc[2])
    T.append(ts)

    while ts - T[0] > hist_len_sec:
        X_lse.popleft()
        Y_lse.popleft()
        Z_lse.popleft()
        T.popleft()

    if len(X_lse) > 15:
        X_filtered = savgol_filter(X_lse, 15, 5, mode="nearest")
        Y_filtered = savgol_filter(Y_lse, 15, 5, mode="nearest")
        Z_filtered = savgol_filter(Z_lse, 15, 5, mode="nearest")
    elif len(X_lse) > 0:
        X_filtered = np.append(X_filtered, X_lse[-1])
        Y_filtered = np.append(Y_filtered, Y_lse[-1])
        Z_filtered = np.append(Z_filtered, Z_lse[-1])

    xf = X_filtered[-1]
    yf = Y_filtered[-1]
    zf = Z_filtered[-1]

    roll, pitch, rate = navigator.navigate_drone(xf, yf)

    f_pos = func1(np.array([x, y, z]), loc)
    c_pos = func1([xf, yf, zf], loc)
    print("POS: ", x, y , z, " func(pos): ", f_pos, " C :", xf, yf, zf, " func1(X_calc): ", c_pos)

    f_pos_norm = np.linalg.norm(f_pos)
    c_pos_norm = np.linalg.norm(c_pos)
    total_pos += f_pos_norm
    total_calc += c_pos_norm

    print("norm f(pos): ", f_pos_norm, " norm f(X_calc): ", c_pos_norm)
    print("total pos norm: ", total_pos, " total calc norm: ", total_calc)

    # Send all math output to the plot
    ts = time.time()
    nr_anchors = len(loc['anchors'])
    send_plot_data(plot_sock, xf, yf, zf, parrot_alt, ts,
                   nr_anchors, roll, pitch, rate, navigator)
