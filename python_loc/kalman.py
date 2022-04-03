#!/usr/bin/env python3

import filterpy
import numpy as np

from filterpy.common import Q_discrete_white_noise
import filterpy.kalman
from scipy.linalg.special_matrices import block_diag

#sigma_a = 0.1
sigma_a = 0.125
sigma_r = 0.2
m_R_scale = 1
m_Q_scale = 1
m_z_damping_factor = 1
last_T = 0
initialized = False

m_tao_acc_sqrt = 0.3
m_tao_bias_sqrt = 0.01

def generate_F_6(T):
    F = np.array([[1, T, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0],
                  [0, 0, 1, T, 0, 0],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, T],
                  [0, 0, 0, 0, 0, 1]
                  ])
    return F

def generate_F_9(T):
    f = [[1, T, -T*T/2.0],
         [0, 1, -T],
         [0, 0, 1]]
    F = block_diag(f, f, f)

    return F

def generate_Q_6(T):
    #Q = Q_discrete_white_noise(dim=2, dt=T, var=sigma_a**2, block_size=3)
    q = [[T**4 / 3, T**3 / 2],
         [T**3 / 2, T**2]]
    qz = np.array(q) * m_z_damping_factor
    Q = block_diag(q, q, qz)
    Q *= sigma_a**2
    Q *= m_Q_scale

    return Q

def generate_Q_9(T):
    tao_acc = m_tao_acc_sqrt * m_tao_acc_sqrt
    tao_bias = m_tao_bias_sqrt * m_tao_bias_sqrt

    #Q = Q_discrete_white_noise(dim=2, dt=T, var=sigma_a**2, block_size=3)
    q = [[T**3/3.0*tao_acc+T**5/20.0*tao_bias,  T**2/2*tao_acc+T**4/8.0*tao_bias,  -T**3/6*tao_bias],
         [T**2/2.0*tao_acc+T**4/8.0*tao_bias ,  T*tao_acc+T**3/3*tao_bias,  -T**2/2*tao_bias],
         [-T**3/6.0*tao_bias,  -T**2/2*tao_bias,  T*tao_bias]]
    qz = np.array(q) * m_z_damping_factor
    Q = block_diag(q, q, qz)
    Q *= sigma_a**2
    Q *= m_Q_scale

    return Q

def generate_B_9(T):
    b = [[T*T/2.0,  0,  0],
         [T,  0,  0],
         [0,  0,  0]]
    B = block_diag(b, b, b)

    return B

def generate_H_6(Xk, loc):
    H = np.empty([0, 6])
    for anchor in loc['anchors']:
        coords = anchor["pos"]["coords"]
        anch_x = coords[0]
        anch_y = coords[1]
        anch_z = coords[2]

        anch = np.array([anch_x, 0, anch_y, 0, anch_z, 0])
        pos = np.array([Xk[0][0], 0, Xk[2][0], 0, Xk[4][0], 0])
        r = np.linalg.norm(pos - anch) + 1e-6
        h_row = (pos - anch) / r
        H = np.append(H, [h_row], axis=0)

    return H

def generate_H_9(Xk, loc):
    H = np.empty([0, 9])
    for anchor in loc['anchors']:
        coords = anchor["pos"]["coords"]
        anch_x = coords[0]
        anch_y = coords[1]
        anch_z = coords[2]

        anch = np.array([anch_x, 0, 0, anch_y, 0, 0, anch_z, 0, 0])
        pos = np.array([Xk[0][0], 0, 0, Xk[3][0], 0, 0, Xk[6][0], 0, 0])
        r = np.linalg.norm(pos - anch) + 1e-6
        h_row = (pos - anch) / r
        H = np.append(H, [h_row], axis=0)

    return H

def get_measurements(loc):
    ranges = []
    for anchor in loc['anchors']:
        dist = anchor["dist"]["dist"]
        ranges.append(dist)

    return np.array([ranges]).T

def get_u(nano_data):
    print(nano_data)
    u = [nano_data["acc"][0], 0, 0,
         nano_data["acc"][1], 0, 0,
         nano_data["acc"][2], 0, 0]
    return np.array([u]).T

def hx_func_6(x, loc):
    r_pred = []
    for anchor in loc['anchors']:
        coords = anchor["pos"]["coords"]
        anch_x = coords[0]
        anch_y = coords[1]
        anch_z = coords[2]

        anch = np.array([anch_x, anch_y, anch_z])
        pos = np.array([x[0][0], x[2][0], x[4][0]])
        r = np.linalg.norm(pos - anch) + 1e-6
        r_pred.append(r)

    return np.array([r_pred]).T

def hx_func_9(x, loc):
    r_pred = []
    for anchor in loc['anchors']:
        coords = anchor["pos"]["coords"]
        anch_x = coords[0]
        anch_y = coords[1]
        anch_z = coords[2]

        anch = np.array([anch_x, anch_y, anch_z])
        pos = np.array([x[0][0], x[3][0], x[6][0]])
        r = np.linalg.norm(pos - anch) + 1e-6
        r_pred.append(r)

    return np.array([r_pred]).T

def ekf_6(filter, loc):
    global last_T
    global initialized

    if not initialized:
        # F = generate_F(0.1)
        # Q = generate_Q(0.01, sigma_a)
        # coords = loc["pos"]["coords"]
        # x0 = coords[0]
        # y0 = coords[1]
        # z0 = coords[2]
        # Xk = np.array([[x0, 0, y0, 0, z0, 0]]).T
        last_T = loc["ts"]
        initialized = True
        T = 0.1
        # set cov of vel
        filter.P[1][1] = 0.1
        filter.P[3][3] = 0.1
        filter.P[5][5] = 0.1
    else:
        T = loc["ts"] - last_T
        last_T = loc["ts"]

    print("ekf6 T = ", T)
    old_x = filter.x
    old_P = filter.P
    R = np.eye(len(loc["anchors"])) * (sigma_r**2 * m_R_scale)

    filter.F = generate_F_6(T)
    filter.Q = generate_Q_6(T)

    filter.dim_z = len(loc['anchors'])

    filter.predict()

    #Xk, P = filterpy.kalman.predict(Xk, P, F, Q)
    z = get_measurements(loc)

    filter.R = R
    filter.update(z, HJacobian=generate_H_6, Hx=hx_func_6, args=loc, hx_args=loc)
    #Xk, P = filterpy.kalman.update(Xk, P, z, R, H)

    if filter.x[4][0] < 0:
        filter.x[4][0] = np.abs(filter.x[4][0])

    if np.any(np.abs(filter.y) > 2):
        print("innovation is too large: ", filter.y)
        filter.x = old_x
        filter.P = old_P
        return None
    Xk = filter.x

    return [Xk[0][0], Xk[2][0], Xk[4][0]]

def ekf_9(filter, loc, nano_data):
    global last_T
    global initialized

    if nano_data == None:
        return None
    if loc == None:
        return None

    if not initialized:
        # F = generate_F(0.1)
        # Q = generate_Q(0.01, sigma_a)
        # coords = loc["pos"]["coords"]
        # x0 = coords[0]
        # y0 = coords[1]
        # z0 = coords[2]
        # Xk = np.array([[x0, 0, y0, 0, z0, 0]]).T
        last_T = loc["ts"]
        initialized = True
        T = 0.1
    else:
        loc_ts = loc["ts"]
        nano_ts = nano_data["ts"]

        ts = max(loc_ts, nano_ts)

        if ts > last_T:
            T = ts - last_T
            last_T = ts
        else:
            return None

    print("ekf9 T = ", T)

    old_x = filter.x
    old_P = filter.P
    R = np.eye(len(loc["anchors"])) * (sigma_r**2 * m_R_scale)

    filter.F = generate_F_9(T)
    filter.Q = generate_Q_9(T)
    filter.B = generate_B_9(T)
    filter.dim_z = len(loc['anchors'])
    u = get_u(nano_data)
    filter.predict(u=u)

    #Xk, P = filterpy.kalman.predict(Xk, P, F, Q)
    z = get_measurements(loc)

    filter.R = R
    filter.update(z, HJacobian=generate_H_9, Hx=hx_func_9, args=loc, hx_args=loc)
    #Xk, P = filterpy.kalman.update(Xk, P, z, R, H)

    if False : #np.any(np.abs(filter.y) > 2):
        filter.x = old_x
        filter.P = old_P

    Xk = filter.x

    return [Xk[0][0], Xk[3][0], Xk[6][0]]

