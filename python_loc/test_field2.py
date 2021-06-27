import math
import socket
import struct
import select

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from scipy.optimize import least_squares
from scipy.optimize import minimize

import collections


from scipy.signal.signaltools import wiener
from scipy.signal import savgol_filter
#X_f, Y_f, Z_f = wiener(np.array([X_lse, Y_lse, Z_lse]))

import time

PARROT_IP = "127.0.0.1"
PARROT_PORT = 5556

MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5555

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

def draw_scene(ax, X_filtered, Y_filtered, Z_filtered, ts):
    global first_ts

    if first_ts == 0 and ts != first_ts:
        first_ts = ts

    if len(X_filtered) > 0:
        plt.plot(X_filtered, Y_filtered, Z_filtered, color='g')
        ax.scatter(X_filtered, Y_filtered, Z_filtered, color='r', s=0.8)
        ax.scatter(X_filtered[-1], Y_filtered[-1], Z_filtered[-1], color='b', s=5)

        ax.text2D(0.0, 1, "     x         y          z", transform=ax.transAxes)
        ax.text2D(0.0, 0.96, "{:7.2f} {:7.2f} {:7.2f}     {:7.3f}s".format(
                  X_filtered[-1], Y_filtered[-1], Z_filtered[-1], ts - first_ts), transform=ax.transAxes)

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
    sock.bind((PARROT_IP, PARROT_PORT))
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

draw_scene(ax, X_filtered, Y_filtered, Z_filtered, 0)

while True:
    ax.cla()

    print(">> get location from 4 anchors")

    loc, parrot_data = get_dwm_location_or_parrot_data()
    # if loc is None or len(loc['anchors']) != 4:
    #     continue

    if parrot_data is not None:
        print("## get parrot data: alt %f roll %f pitch %f yaw %f" % \
            (parrot_data['alt'], parrot_data['roll'], \
               parrot_data['pitch'], parrot_data['yaw']))

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

    draw_scene(ax, X_filtered, Y_filtered, Z_filtered, ts)

    xf = X_filtered[-1]
    yf = Y_filtered[-1]
    zf = Z_filtered[-1]
    f_pos = func1(np.array([x, y, z]), loc)
    c_pos = func1([xf, yf, zf], loc)
    print("POS: ", x, y , z, " func(pos): ", f_pos, " C :", xf, yf, zf, " func1(X_calc): ", c_pos)

    f_pos_norm = np.linalg.norm(f_pos)
    c_pos_norm = np.linalg.norm(c_pos)
    total_pos += f_pos_norm
    total_calc += c_pos_norm

    print("norm f(pos): ", f_pos_norm, " norm f(X_calc): ", c_pos_norm)

    print("total pos norm: ", total_pos, " total calc norm: ", total_calc)
