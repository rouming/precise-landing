import math
import socket
import struct

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from scipy.optimize import least_squares
from scipy.optimize import minimize


from scipy.signal.signaltools import wiener
from scipy.signal import savgol_filter
#X_f, Y_f, Z_f = wiener(np.array([X_lse, Y_lse, Z_lse]))

import time

MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5555
sock  = None

A = np.array([0.54,0.54,0.00]) # 2585
B = np.array([0.54,0.00,0.00]) # 262d
C = np.array([0.00,0.54,0.00]) # 260f
D = np.array([0.00,0.00,0.00]) # 2852

X = []
Y = []
Z = []

X_lse = []
Y_lse = []
Z_lse = []

def create_sock():
    # Create sock and bind
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((MCAST_GRP, MCAST_PORT))

    # Join group
    mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    return sock

def receive_location_from_sock(sock):
    buf = sock.recv(512)

    # Location header
    fmt = "iiihhiii"
    (x, y, z, pos_qf, pos_valid, ts_sec, ts_usec, nr_anchors) = struct.unpack_from(fmt, buf, 0)

    # Skip location header
    off = struct.calcsize(fmt)

    print("ts:%ld.%06ld [%d,%d,%d,%u] " % (ts_sec, ts_usec, x, y, z, pos_qf), end='')

    location = {
        'calc_pos': {
            'x':  float(x),
            'y':  float(y),
            'z':  float(z),
            'qf': pos_qf,
            'valid': pos_valid,
        },
        'ts': {
            'sec':  ts_sec,
            'usec': ts_usec,
        },
        'anchors': [],
    }

    # For each anchor
    fmt = "iiihhihh"
    for i in range(0, nr_anchors):
        (x, y, z, pos_qf, pos_valid, dist, addr, dist_qf) = struct.unpack_from(fmt, buf, off)
        off += struct.calcsize(fmt)

        print("#%u) a:0x%08x [%d,%d,%d,%u] d=%u,qf=%u " % (i, addr, x, y, z, pos_qf, dist, dist_qf), \
              end='')

        anchor = {
            'pos': {
                'x':  float(x),
                'y':  float(y),
                'z':  float(z),
                'qf': pos_qf,
                'valid': pos_valid,
            },
            'dist': {
                'dist': float(dist),
                'addr': addr,
                'qf': dist_qf
            },
        }

        location['anchors'].append(anchor)

    return location

def get_location():
    global sock
    if sock is None:
        sock = create_sock()

    return receive_location_from_sock(sock)

def find_anchor_by_addr(location, addr):
    for anchor in location['anchors']:
        if anchor['dist']['addr'] == addr:
            return anchor

    return None

def func1(X, la, lb, lc, ld):
    ret = (np.linalg.norm(X - A) - la) ** 2 + (np.linalg.norm(X - B) - lb) ** 2 + \
          (np.linalg.norm(X - C) - lc) ** 2 + (np.linalg.norm(X - D) - ld) ** 2
    return ret

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
def calc_pos(X0, la, lb, lc, ld):

    # all are the experiments
    #res = least_squares(func, X0, loss='soft_l1', jac=jac, bounds=([-3, -3, 0.0], [3, 3, 3]), args=(la, lb, lc, ld), verbose=1)

    #to make smooth path, but in general this is shit i think
    # lowb = X0-0.2
    # if lowb[2] < 0:
    #     lowb[2] = 0
    # upb = X0+0.2
    lowb = [-math.inf, -math.inf, 0]
    upb = [math.inf, math.inf, math.inf]


    res = least_squares(func1, X0, loss='cauchy', f_scale=0.001, bounds=(lowb, upb),
                        args=(la, lb, lc, ld), verbose=1)
    ##also decent and fast
    # res = minimize(func1, X0, method="L-BFGS-B", bounds=[(-math.inf, math.inf), (-math.inf, math.inf), (0, math.inf)],
    #                #options={'ftol': 1e-4, 'disp': True}, args=(la, lb, lc, ld))
    #                options={'ftol': 1e-4,'eps' : 1e-4, 'disp': True}, args=(la, lb, lc, ld))

    # res = minimize(func1, X0, method="SLSQP", bounds=[(-math.inf, math.inf), (-math.inf, math.inf), (0, math.inf)],
    #           #options={ 'ftol': 1e-5, 'disp': True}, args=(la, lb, lc, ld))
    #           options={'ftol': 1e-5,'eps' : 1e-4, 'disp': True}, args=(la, lb, lc, ld))

    # experiments
    #res = minimize(func1, X0, method='BFGS', options={'xatol': 1e-8, 'disp': True}, args=(la, lb, lc, ld))
    #res = optimize.shgo(func1, bounds=[(-10, 10), (-10, 10), (0, 10)], args=(la, lb, lc, ld),n=200, iters=5, sampling_method='sobol')
    #res = minimize(func1, X0, method='BFGS', options={'disp': True}, args=(la, lb, lc, ld))
    return res.x

total_pos = 0
total_calc = 0

rects = [[A, B, D, C]]
fig1 = plt.figure()
ax = fig1.add_subplot(111, projection='3d')
ax.add_collection3d(Poly3DCollection(rects, color='g', alpha=0.5))



while True:
    ax.cla()

    print(">> get location from 4 anchors")

    loc = get_location()
    if len(loc['anchors']) != 4:
        continue

    print(">> got calculated position from the engine")

    x = loc['calc_pos']['x']/1000
    y = loc['calc_pos']['y']/1000
    z = loc['calc_pos']['z']/1000
    X.append(x)
    Y.append(y)
    Z.append(z)

    print(">> get distances")

    la = find_anchor_by_addr(loc, 0x2585)['dist']['dist'] /1000
    lb = find_anchor_by_addr(loc, 0x262d)['dist']['dist'] /1000
    lc = find_anchor_by_addr(loc, 0x260f)['dist']['dist'] /1000
    ld = find_anchor_by_addr(loc, 0x28b9)['dist']['dist'] /1000

    if not assigned:
        X0 = np.abs(np.array([x, y, z]))
        assigned = True

    X_calc = calc_pos(X0, la, lb, lc, ld)
    #X_calc = calc_pos([0, 0, 0], la, lb, lc, ld)
    #X_calc = calc_pos(np.abs([x, y ,z]), la, lb, lc, ld)
    f_pos = func1([x, y ,z], la, lb, lc ,ld)
    c_pos = func1(X_calc, la, lb, lc ,ld)
    print("POS: ", x, y , z, " func(pos): ", f_pos, " C :", X_calc[0], X_calc[1], X_calc[2], " func1(X_calc): ", c_pos)

    f_pos_norm = np.linalg.norm(f_pos)
    c_pos_norm = np.linalg.norm(c_pos)
    total_pos += f_pos_norm
    total_calc += c_pos_norm

    print("norm f(pos): ", f_pos_norm, " norm f(X_calc): ", c_pos_norm)

    X0 = X_calc
    X_lse.append(X_calc[0])
    Y_lse.append(X_calc[1])
    Z_lse.append(X_calc[2])

    if len(X_lse) > 21:
        X_filtered = savgol_filter(X_lse, 21, 5)
        Y_filtered = savgol_filter(Y_lse, 21, 5)
        Z_filtered = savgol_filter(Z_lse, 21, 5)


        plt.plot(X_filtered, Y_filtered, Z_filtered, color='g')
        ax.scatter(X_filtered, Y_filtered, Z_filtered, color='r', s=0.8)
        ax.add_collection3d(Poly3DCollection(rects, color='g', alpha=0.5))
        ax.set_xlim3d(-8, 8)
        ax.set_ylim3d(-8, 8)
        ax.set_zlim3d(0, 10)

        plt.pause(0.01)

print("total pos norm: ", total_pos, " total calc norm: ", total_calc)

#X_lse, Y_lse, Z_lse = wiener(np.array([X_lse, Y_lse, Z_lse]))

# just some empiric values

