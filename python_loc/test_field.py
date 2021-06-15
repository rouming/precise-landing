import math

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from scipy.optimize import least_squares
from scipy.optimize import minimize
from scipy import optimize
from scipy import interpolate

from scipy.signal.signaltools import wiener
from scipy.signal import savgol_filter
#X_f, Y_f, Z_f = wiener(np.array([X_lse, Y_lse, Z_lse]))

anchors_id = ["a:0x00002585", "a:0x0000262d", "a:0x0000260f", "a:0x00002852"]
A = np.array([0.54,0.54,0.00]) # 2585
B = np.array([0.54,0.00,0.00]) # 262d
C = np.array([0.00,0.54,0.00]) # 260f
D = np.array([0.00,0.00,0.00]) # 2852

file1 = open('data/field-session-1/log.6', 'r')
count = 0

X = []
Y = []
Z = []

X_lse = []
Y_lse = []
Z_lse = []

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


    # res = least_squares(func1, X0, loss='cauchy', f_scale=0.001, bounds=(lowb, upb),
    #                     args=(la, lb, lc, ld), verbose=1)
    ##also decent and fast
    # res = minimize(func1, X0, method="L-BFGS-B", bounds=[(-math.inf, math.inf), (-math.inf, math.inf), (0, math.inf)],
    #                #options={'ftol': 1e-4, 'disp': True}, args=(la, lb, lc, ld))
    #                options={'ftol': 1e-4,'eps' : 1e-4, 'disp': True}, args=(la, lb, lc, ld))

    res = minimize(func1, X0, method="SLSQP", bounds=[(-math.inf, math.inf), (-math.inf, math.inf), (0, math.inf)],
              #options={ 'ftol': 1e-5, 'disp': True}, args=(la, lb, lc, ld))
              options={'ftol': 1e-5,'eps' : 1e-4, 'disp': True}, args=(la, lb, lc, ld))

    # experiments
    #res = minimize(func1, X0, method='BFGS', options={'xatol': 1e-8, 'disp': True}, args=(la, lb, lc, ld))
    #res = optimize.shgo(func1, bounds=[(-10, 10), (-10, 10), (0, 10)], args=(la, lb, lc, ld),n=200, iters=5, sampling_method='sobol')
    #res = minimize(func1, X0, method='BFGS', options={'disp': True}, args=(la, lb, lc, ld))
    return res.x

def get_dist(line, id):
    l = line.split(id+"[")[1]
    l = l.split("d=")[1]
    val = l.split(",")[0]

    return float(val)

total_pos = 0
total_calc = 0

while True:
    count += 1

    line = file1.readline()
    if not line:
        break

    if anchors_id[0] not in line:
        continue
    if anchors_id[1] not in line:
        continue
    if anchors_id[2] not in line:
        continue
    if anchors_id[3] not in line:
        continue

    r = line.split(" [")[1]
    r = r.split("]#")[0]

    vals = r.split(',')
    x = float(vals[0])/1000
    y = float(vals[1])/1000
    z = float(vals[2])/1000
    X.append(x)
    Y.append(y)
    Z.append(z)

    la = get_dist(line, 'a:0x00002585') /1000
    lb = get_dist(line, 'a:0x0000262d') /1000
    lc = get_dist(line, 'a:0x0000260f') /1000
    ld = get_dist(line, 'a:0x00002852') /1000

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

print("total pos norm: ", total_pos, " total calc norm: ", total_calc)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.plot(X, Y, Z)
ax.scatter(X, Y, Z, color='r',s=0.8)
rects = [[A, B, D, C]]
# Each rectangle is presented as a set of points
# array rects consist of rectangles
ax.add_collection3d(Poly3DCollection(rects, color='g', alpha=0.5))
#plt.show()

fig1 = plt.figure()
ax = fig1.add_subplot(111, projection='3d')
#X_lse, Y_lse, Z_lse = wiener(np.array([X_lse, Y_lse, Z_lse]))

# just some empiric values
X_lse = savgol_filter(X_lse, 21,5)
Y_lse = savgol_filter(Y_lse, 21,5)
Z_lse = savgol_filter(Z_lse, 21,5)


plt.plot(X_lse, Y_lse, Z_lse, color='g')
ax.scatter(X_lse, Y_lse, Z_lse, color='r',s=0.8)
ax.add_collection3d(Poly3DCollection(rects, color='g', alpha=0.5))
plt.show()
