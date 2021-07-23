import math
import socket
import struct
import select
import os
import re

import matplotlib.pyplot as plt
from matplotlib.artist import Artist
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from scipy.optimize import least_squares
from scipy.optimize import minimize
from simple_pid import PID

import collections


from scipy.signal.signaltools import wiener
from scipy.signal import savgol_filter
#X_f, Y_f, Z_f = wiener(np.array([X_lse, Y_lse, Z_lse]))

import time

# Distances from DWM1001-server
MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5555

# Telemetry from drone
UDP_TELEMETRY_IP = '127.0.0.1'
UDP_TELEMETRY_PORT = 5556

# Commands to drone
UDP_COMMANDS_IP = '127.0.0.1'
UDP_COMMANDS_PORT = 5557

# Landing point in meters, middle of the landing platform
LANDING_X = 0.54 / 2
LANDING_Y = 0.54 / 2

dwm_sock    = None
parrot_sock = None

dwm_loc     = None
parrot_data = None

A = np.array([0.54,0.54,0.00]) # 2585
B = np.array([0.54,0.00,0.00]) # 262d
C = np.array([0.00,0.54,0.00]) # 260f
D = np.array([0.00,0.00,0.00]) # 2852


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

X_LIM = 7
Y_LIM = 7
Z_LIM = 7

rects = [[A, B, D, C]]

first_ts = 0

class dynamic_plot():
    # Time range in seconds
    min_x = 0
    max_x = 10

    # Distange range in meters
    min_y = -5
    max_y = 5

    # Static line
    lines2_y = 0.0

    # For cleaning
    previous_text = None

    def __init__(self, plot_title, x_label, y_label,
                 lines1_label, lines2_label, lines2_y):
        # Turn on plot interactive mode
        plt.ion()

        # Set up plot
        self.figure, self.ax = plt.subplots()

        # Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        self.ax.set(xlim=(self.min_x, self.max_x),
                    ylim=(self.min_y, self.max_y),
                    xlabel=x_label,
                    ylabel=y_label,
                    title=plot_title)

        # Enable grid
        self.ax.grid()

        # Create curves on the plot
        self.lines1, = self.ax.plot([],[], '-', label=lines1_label)
        self.lines2, = self.ax.plot([],[], '-', label=lines2_label)

        # Set other members
        self.lines2_y = lines2_y
        self.xdata  = []
        self.ydata  = []
        self.tsdata = []

    def _remove_outdated_data(self):
        now = time.time()

        diff = self.max_x - self.min_x
        ts = self.tsdata[0]

        while ts < now - diff:
            self.xdata.pop(0)
            self.ydata.pop(0)
            self.tsdata.pop(0)
            ts = self.tsdata[0]

    def update(self, xdata, ydata, text):
        self.xdata.append(xdata)
        self.ydata.append(ydata)
        self.tsdata.append(time.time())

        # Clean points which are not visible on the plot
        self._remove_outdated_data()

        # Following window
        if xdata >= self.max_x:
            diff = self.max_x - self.min_x
            self.max_x = xdata
            self.min_x = xdata - diff
            self.ax.set_xlim(self.min_x, self.max_x)

        # Update data (with the new _and_ the old points)
        self.lines1.set_xdata(self.xdata)
        self.lines1.set_ydata(self.ydata)

        self.lines2.set_xdata([self.min_x, self.max_x])
        self.lines2.set_ydata([self.lines2_y, self.lines2_y])

        # Set text
        if self.previous_text:
            Artist.remove(self.previous_text)
        self.previous_text = self.ax.text(0.0, 1.025, text, transform=self.ax.transAxes, \
                                          bbox=dict(facecolor='green', alpha=0.3))

        # Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.legend()

        # We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

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

        # Create PID plots
        pid_x_plot = dynamic_plot('PID X', 'Time (s)', 'Drone X distance (m)',
                                  'PID', 'target', target_x)
        pid_y_plot = dynamic_plot('PID Y', 'Time (s)', 'Drone Y distance (m)',
                                  'PID', 'target', target_y)

        self.start_time = time.time()
        self.pid_x_plot = pid_x_plot
        self.pid_y_plot = pid_y_plot

        self.sock = sock
        self.x_pid = x_pid
        self.y_pid = y_pid

    def _send_command(self, roll, pitch, yaw, throttle):
        # 4x signed chars
        buf = struct.pack("bbbb", int(roll), int(pitch), int(yaw), int(throttle))
        self.sock.sendto(buf, (UDP_COMMANDS_IP, UDP_COMMANDS_PORT))

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

        # Update plots
        pid_x_text = "Kp=%.2f Ki=%.2f Kd=%.2f   Update %.1fHz\n" \
                     "   %.2f    %.2f    %.2f\n" \
                     "x %5.2f  roll %d" % \
                     (self.x_pid.Kp, self.x_pid.Ki, self.x_pid.Kd, rate, \
                      self.x_pid.components[0], self.x_pid.components[1], self.x_pid.components[2], \
                      drone_x, roll)

        pid_y_text = "Kp=%.2f Ki=%.2f Kd=%.2f   Update %.1fHz\n" \
                     "   %.2f    %.2f    %.2f\n" \
                     "y %5.2f  pitch %d" % \
                     (self.y_pid.Kp, self.y_pid.Ki, self.y_pid.Kd, rate, \
                      self.y_pid.components[0], self.y_pid.components[1], self.y_pid.components[2], \
                      drone_y, pitch)

        ts = time.time() - self.start_time
        self.pid_x_plot.update(ts, drone_x, pid_x_text)
        self.pid_y_plot.update(ts, drone_y, pid_y_text)

def draw_scene(ax, X_filtered, Y_filtered, Z_filtered, ts, anch_cnt):
    global first_ts

    if first_ts == 0 and ts != first_ts:
        first_ts = ts

    if len(X_filtered) > 0:
        # FIXME: This needs to be not global 'plt' dependendent,
        # FIXME: otherwise it affects all other dynamic plot
        # FIXME: windows (e.g. PID). rpen
        # plt.plot(X_filtered, Y_filtered, Z_filtered, color='g')

        ax.scatter(X_filtered, Y_filtered, Z_filtered, color='r', s=0.8)
        ax.scatter(X_filtered[-1], Y_filtered[-1], Z_filtered[-1], color='b', s=5)

        ax.text2D(0.0, 1, "     x         y          z", transform=ax.transAxes)
        ax.text2D(0.0, 0.96, "{:7.2f} {:7.2f} {:7.2f}     {:7.3f}s    #{} anch".format(
                  X_filtered[-1], Y_filtered[-1], Z_filtered[-1], ts - first_ts, anch_cnt),
                  transform=ax.transAxes)

    if parrot_data is not None:
        ax.text2D(0.0, 0.86, " alt {:6.2f}m".format(parrot_data["alt"]), transform=ax.transAxes)
        ax.text2D(0.0, 0.82, "diff {:6.2f}m".format(Z_filtered[-1] - parrot_data["alt"]),
                  transform=ax.transAxes)


    ax.add_collection3d(Poly3DCollection(rects, color='g', alpha=0.5))
    ax.set_xlim3d(-X_LIM, X_LIM)
    ax.set_ylim3d(-Y_LIM, Y_LIM)
    ax.set_zlim3d(0, Z_LIM)


    plt.pause(0.000001)

def create_dwm_sock():
    # Create sock and bind
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((MCAST_GRP, MCAST_PORT))

    # Join group
    mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.setblocking(0)

    return sock

def create_parrot_sock():
    # Create parrot sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_TELEMETRY_IP, UDP_TELEMETRY_PORT))
    sock.setblocking(0)

    return sock

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
    lowb = [-math.inf, -math.inf, 0]
    upb = [math.inf, math.inf, math.inf]

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

fig1 = plt.figure()
plt.ion()
ax = fig1.add_subplot(111, projection='3d')

draw_scene(ax, X_filtered, Y_filtered, Z_filtered, 0, 0)

navigator = drone_navigator(LANDING_X, LANDING_Y)

while True:
    ax.cla()

    print(">> get location from 4 anchors")

    loc, parrot_data = get_dwm_location_or_parrot_data()
    # if loc is None or len(loc['anchors']) != 4:
    #     continue

    print(">> got calculated position from the engine")

    x = loc['calc_pos']['x']
    y = loc['calc_pos']['y']
    z = loc['calc_pos']['z']
    ts = loc["ts"]

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

    draw_scene(ax, X_filtered, Y_filtered, Z_filtered, ts, len(loc['anchors']))

    xf = X_filtered[-1]
    yf = Y_filtered[-1]
    zf = Z_filtered[-1]

    if navigator:
        navigator.navigate_drone(xf, yf)

    f_pos = func1(np.array([x, y, z]), loc)
    c_pos = func1([xf, yf, zf], loc)
    print("POS: ", x, y , z, " func(pos): ", f_pos, " C :", xf, yf, zf, " func1(X_calc): ", c_pos)

    f_pos_norm = np.linalg.norm(f_pos)
    c_pos_norm = np.linalg.norm(c_pos)
    total_pos += f_pos_norm
    total_calc += c_pos_norm

    print("norm f(pos): ", f_pos_norm, " norm f(X_calc): ", c_pos_norm)

    print("total pos norm: ", total_pos, " total calc norm: ", total_calc)
