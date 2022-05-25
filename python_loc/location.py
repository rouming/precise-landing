#!/usr/bin/env python3

"""Localization script with all the math

Usage:
  location.py --data-source <source>

Options:
  -h --help                  Show this screen
  --data-source <source>     Should be 'ble' or 'sock'
"""

from docopt import docopt
import time
import math
import socket
import struct
import select
import threading
import eventfd
import enum
import os
import re
import sys
import signal
from collections import namedtuple

import filterpy.kalman
from droneloc import drone_localization
from droneloc import smoother_type

import dwm1001_ble
import nano33ble

import numpy as np
from scipy.optimize import least_squares
from scipy.optimize import minimize
from simple_pid import PID
import config as cfg

dwm_fd      = None
nano33_fd   = None
parrot_sock = None

dwm_manager = None
nano33_manager = None
post_smoother = None
last_loc_anchors = None
last_loc_update_nr = 0

should_stop = False
stop_efd    = eventfd.EventFD()

PID_CONTROL_RATE_HZ = 2

LANDING_ALT           = 0.5
LANDING_SETTLING_SECS = 1
YAW_LANDING_ERROR     = 0.01

# How many times we can accept other data without
# major DWM location updates
UNRELIABLE_UPDATES    = 4

# From higher altitude the landing error is higher
# and then linearly decreases, i.e. on the 10m
# altitude error is 2.8m, but on the 1m error
# decreases to 0.1m
def XY_LANDING_ERROR(alt):
    err = 0.3*alt - 0.2
    return np.clip(err, 0.1, 3)

# The rate of descent depends on the altitude in order
# to have a faster descent if the altitude is high and
# a more accurate descent if drone is close to the ground.
# Here we return -20 min on <= 1m and -100 max on >= 5m
def descend_control_rate(alt):
    rate = -20*alt
    return np.clip(rate, -20, -100)

class dwm_source(enum.Enum):
    BLE = 0
    SOCK = 1

class parrot_event_type(enum.Enum):
    ALTITUDE = 0
    ATTITUDE = 1
    VELOCITY = 2
    POSITION = 3

pid_components = namedtuple('pid_components', 'Kp Ki Kd')

# Default PID config
xy_pid_comp  = pid_components(Kp=10,  Ki=0, Kd=50)
yaw_pid_comp = pid_components(Kp=200, Ki=0, Kd=100)
pid_limits   = (-100, 100)

DWM_DATA_SOURCE = dwm_source.BLE

def normalize_angle(x):
    x = x % (2 * math.pi)    # force in range [0, 2 pi)
    if x > math.pi:          # move to [-pi, pi)
        x -= 2 * math.pi
    return x

def ned_to_enu_coords(vec):
    return (vec[1], vec[0], -vec[2])

# Rotates anti-clockwise in XY plane
def rotate_vec_on_xy_angle(vec, xy_angle):
    rot = np.array([[np.cos(xy_angle), -np.sin(xy_angle),  0],
                    [np.sin(xy_angle),  np.cos(xy_angle),  0],
                    [               0,                 0,  1]])
    return np.dot(rot, np.array(vec))

class drone_navigator(threading.Thread):
    # Thread data
    in_pos = None
    in_yaw = None
    lock =  threading.Lock()

    # Target coords and angle
    target_x = 0
    target_y = 0
    target_yaw = 0

    # PID output
    pitch = 0
    roll = 0
    yaw = 0

    def __init__(self, target_x, target_y, target_yaw):
        # Create commands sock
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Set desired landing coordinates
        x_pid = PID(Kp=xy_pid_comp.Kp, Ki=xy_pid_comp.Ki, Kd=xy_pid_comp.Kd,
                    proportional_on_measurement=False)
        y_pid = PID(Kp=xy_pid_comp.Kp, Ki=xy_pid_comp.Ki, Kd=xy_pid_comp.Kd,
                    proportional_on_measurement=False)
        yaw_pid = PID(Kp=yaw_pid_comp.Kp, Ki=yaw_pid_comp.Ki, Kd=yaw_pid_comp.Kd,
                      setpoint=target_yaw,
                      proportional_on_measurement=False,
                      error_map=normalize_angle)

        # Control coeff limits
        x_pid.output_limits = pid_limits
        y_pid.output_limits = pid_limits
        yaw_pid.output_limits = pid_limits

        self.start_time = time.time()

        self.sock = sock
        self.x_pid   = x_pid
        self.y_pid   = y_pid
        self.yaw_pid = yaw_pid
        self.target_x = target_x
        self.target_y = target_y
        self.target_yaw = target_yaw

        super().__init__()

    def _send_command(self, roll, pitch, yaw, throttle):
        # 4x signed chars
        buf = struct.pack("bbbb", int(roll), int(pitch), int(yaw), int(throttle))
        self.sock.sendto(buf, (cfg.UDP_COMMANDS_IP, cfg.UDP_COMMANDS_PORT))

    def run(self):

        ready_to_land_ts = None

        while not should_stop:
            time.sleep(1/PID_CONTROL_RATE_HZ)

            self.lock.acquire()
            in_pos = self.in_pos
            in_yaw = self.in_yaw
            self.lock.release()

            if in_yaw is None:
                continue

            control_yaw = self.yaw_pid(in_yaw)
            control_x   = 0
            control_y   = 0
            control_thr = 0

            if in_pos is not None:
                # Drone yaw angle relative to the landing area
                yaw_angle = in_yaw - self.target_yaw

                # Target position in the drone frame
                target_pos = np.array([self.target_x, self.target_y, 0]) - \
                             np.array(in_pos)
                target_pos = rotate_vec_on_xy_angle(target_pos, yaw_angle)

                # Translate drone position into target frame
                drone_pos = -target_pos
                altitude = drone_pos[2]

                control_x = self.x_pid(drone_pos[0])
                control_y = self.y_pid(drone_pos[1])

                # Calculate XY error
                xy_error = np.abs(drone_pos[:2] - np.zeros(2))
                xy_ready = np.all(xy_error < XY_LANDING_ERROR(altitude))

                # Calculate yaw error
                yaw_error = np.abs(normalize_angle(yaw_angle))
                yaw_ready = np.all(yaw_error < YAW_LANDING_ERROR)

                # Make a decision if we descend or initiate landing
                if xy_ready and yaw_ready:
                    now = time.time()
                    if ready_to_land_ts is None:
                        ready_to_land_ts = now
                    elif now - ready_to_land_ts >= LANDING_SETTLING_SECS:
                        if altitude <= LANDING_ALT:
                            # Initiate landing. Clumsy, but no extra landing
                            # command is needed
                            control_thr = -128
                        else:
                            control_thr = descend_control_rate(altitude)

                        # Settle once again on the next descend iteration
                        ready_to_land_ts = None
                else:
                    ready_to_land_ts = None

                print("PID CONTROL X=%4d Y=%4d YAW=%4d, DP=(%.2f %.2f %.2f) YAW=(%.3f°) %s" %
                      (int(control_x), int(control_y), int(control_yaw),
                       drone_pos[0], drone_pos[1], drone_pos[2], in_yaw,
                       '| LAND' if control_thr < -100  else \
                       '| DESC' if control_thr else ''
                       ))

            # Parrot accepts in signed percentage, i.e. [-100, 100] range
            roll  = int(control_x)
            pitch = int(control_y)
            yaw   = int(control_yaw)
            thr   = int(control_thr)

            self._send_command(roll, pitch, yaw, thr)

            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw

    def navigate_drone(self, x, y, z):
        self.lock.acquire()
        self.in_pos = (x, y, z)
        self.lock.release()

    def orient_drone(self, yaw):
        self.lock.acquire()
        self.in_yaw = yaw
        self.lock.release()


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

def create_dwm_ble():
    global dwm_manager

    efd = eventfd.EventFD()
    eventfd_map = {cfg.TAG_ADDR: efd}
    dwm_manager = dwm1001_ble.DWMDeviceManager(adapter_name='hci0',
                                               predefined_anchors=cfg.ANCHORS,
                                               eventfd_map=eventfd_map)
    dwm_manager.start()

    return efd

def destroy_dwm_ble():
    if dwm_manager:
        dwm_manager.stop()

def create_nano33_ble():
    if DWM_DATA_SOURCE != dwm_source.BLE:
        # TODO: this is ugly
        return eventfd.EventFD()

    global nano33_manager
    nano33_manager = nano33ble.Nano33DeviceManager()
    device = nano33ble.Nano33Device(mac_address=cfg.NANO33_MAC,
                                    manager=nano33_manager)

    device.connect()
    nano33_manager.start()

    global nano33_device
    nano33_device = device

    return device.eventfd

def destroy_nano33_ble():
    global nano33_manager
    if nano33_manager:
        nano33_manager.stop()

def create_dwm_fd():
    if DWM_DATA_SOURCE == dwm_source.SOCK:
        return create_dwm_sock()
    return create_dwm_ble()

def destroy_dwm_fd():
    if DWM_DATA_SOURCE == dwm_source.BLE:
        destroy_dwm_ble()

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

def send_plot_data(sock, x, y, z, parrot_alt, ts, rate, nr_anchors, navigator, loc):
    x_pid = navigator.x_pid
    y_pid = navigator.y_pid

    addrs = [0x0000] * 4
    dists = [-1.0] * 4

    i = 0
    for anch in loc["anchors"]:
        addr = anch["addr"]
        dist = anch["dist"]["dist"]

        addrs[i] = addr
        dists[i] = dist
        i += 1

    buf = struct.pack("dffffffBffffffffffffffiiiHHHHffff",
                      ts,
                      x, y, z,
                      *loc['pos']['coords'], loc['pos']['valid'],
                      parrot_alt, rate,
                      x_pid.Kp, x_pid.Ki, x_pid.Kd,
                      x_pid.components[0], x_pid.components[1], x_pid.components[2],
                      y_pid.Kp, y_pid.Ki, y_pid.Kd,
                      y_pid.components[0], y_pid.components[1], y_pid.components[2],
                      navigator.roll, navigator.pitch, nr_anchors,
                      addrs[0], addrs[1], addrs[2], addrs[3],
                      dists[0], dists[1], dists[2], dists[3])
    sock.sendto(buf, (cfg.UDP_PLOT_IP, cfg.UDP_PLOT_PORT))


def receive_dwm_location_from_sock(sock):
    # Location header
    fmt = "iiihhiii"
    sz = struct.calcsize(fmt)
    buf = sock.recv(sz, socket.MSG_PEEK)

    # Unpack location header
    (x, y, z, pos_qf, pos_valid, ts_sec, ts_usec, nr_anchors) = struct.unpack(fmt, buf)

    location = {
        'pos': {
            'coords': [float(x) / 1000,
                       float(y) / 1000,
                       float(z) / 1000 ],
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

        anchor = {
            'addr': addr,
            'pos': {
                'coords': [float(x) / 1000,
                           float(y) / 1000,
                           float(z) / 1000],
                'qf': pos_qf,
                'valid': pos_valid,
            },
            'dist': {
                'dist': float(dist) / 1000,
                'qf': dist_qf
            },
        }

        location['anchors'].append(anchor)

    return location

def receive_dwm_location(dwm_fd):
    if DWM_DATA_SOURCE == dwm_source.SOCK:
        return receive_dwm_location_from_sock(dwm_fd)

    loc = {}
    tag = dwm_manager.find_device_by_node_addr(cfg.TAG_ADDR)
    if tag:
        loc = tag.get_location()
    else:
        print("Error: can't find tag by addr 0x%x" % cfg.TAG_ADDR)

    return loc

def receive_nano33_data():
    global nano33_device
    return nano33_device.get_data()

def receive_parrot_data_from_sock(sock):
    # Header
    fmt = "iii"
    sz = struct.calcsize(fmt)
    buf = sock.recv(sz, socket.MSG_PEEK)
    event, sec, usec = struct.unpack(fmt, buf)

    event_type = parrot_event_type(event)

    if event_type == parrot_event_type.ALTITUDE:
        fmt += "f"
        sz = struct.calcsize(fmt)
        buf = sock.recv(sz)
        _, _, _, alt  = struct.unpack(fmt, buf)
        if alt == 0.0:
            return None
        parrot_data = {'alt': alt}
    elif event_type == parrot_event_type.VELOCITY:
        fmt += "fff"
        sz = struct.calcsize(fmt)
        buf = sock.recv(sz)
        _, _, _, *vel  = struct.unpack(fmt, buf)
        vel = np.array(vel)
        if np.all(vel == 0.0):
            return None
        # Drone coordinates are in NED, but landing area is in ENU
        # rotated on LANDING_ANGLE
        vel = ned_to_enu_coords(vel)
        # Convert velocity in ENU to landing area coordinates:
        # rotates clockwise on LANDING_ANGLE
        vel = rotate_vec_on_xy_angle(vel, -cfg.LANDING_ANGLE)
        parrot_data = {'vel': vel}
    elif event_type == parrot_event_type.POSITION:
        fmt += "fff"
        sz = struct.calcsize(fmt)
        buf = sock.recv(sz)
        _, _, _, lat, lon, alt  = struct.unpack(fmt, buf)
        # TODO
        return None
    elif event_type == parrot_event_type.ATTITUDE:
        fmt += "fff"
        sz = struct.calcsize(fmt)
        buf = sock.recv(sz)
        _, _, _, roll, pitch, yaw  = struct.unpack(fmt, buf)
        parrot_data = {'att': [roll, pitch, yaw]}
    else:
        assert(0)
        return None

    parrot_data['ts'] = float("%ld.%06ld" % (sec, usec))

    return parrot_data

def is_dwm_location_reliable(loc):
    global last_loc_update_nr, last_loc_anchors

    anchors = loc['anchors']

    if len(anchors) < 2:
        # 1 anchor in always unreliable
        print("!!! Warning: got 1 anchor, consider as unreliable")
        return False

    if len(anchors) > 3:
        # 4 anchors and more are always reliable
        last_loc_anchors = anchors
        last_loc_anchors_nr = 0
        return True

    if last_loc_anchors is None:
        # For the first time
        last_loc_anchors = anchors
        last_loc_anchors_nr = 0
        return True

    if len(anchors) > len(last_loc_anchors):
        # More anchors than previously is always a good sign
        last_loc_anchors = anchors
        last_loc_anchors_nt = 0
        return True

    # Compare anchors addresses one by one
    for anch in anchors:
        for last_anch in last_loc_anchors:
            if anch['addr'] != last_anch['addr']:
                # A new anchor in a set is always a good sign
                last_loc_anchors = anchors
                last_loc_anchors_nr = 0
                return True

    last_loc_anchors = anchors

    if last_loc_anchors_nr < UNRELIABLE_UPDATES:
        # All anchors are the same, account similar update
        last_loc_anchors_nr += 1
        return True

    print("!!! Warning: got same %d anchors for %d number of updates, consider as unreliable" %
          (len(anchors), UNRELIABLE_UPDATES))
    return False

def print_location(loc):
    coords = loc['pos']['coords']
    # To mm, because we have all the logs in mm, not in m, argh
    coords = [int(v * 1000) for v in coords]
    print("ts:%.6f [%d,%d,%d,%u] " % \
          (loc['ts'], coords[0], coords[1], coords[2], loc['pos']['qf']),
          end='')
    i = 1
    for anch in loc['anchors']:
        coords = anch['pos']['coords']
        # To mm, because we have all the logs in mm, not in m, argh
        coords = [int(v * 1000) for v in coords]
        dist = int(anch['dist']['dist'] * 1000)
        print("#%u) a:0x%04x [%d,%d,%d,%u] d=%u,qf=%u " % \
              (i, anch['addr'], coords[0], coords[1], coords[2],
               anch['pos']['qf'], dist, anch['dist']['qf']), \
              end='')
        i += 1
    print('')

def get_dwm_location_or_parrot_data():
    global dwm_fd, nano33_fd, parrot_sock
    global stop_efd

    if dwm_fd is None:
        dwm_fd = create_dwm_fd()
    if nano33_fd is None:
        nano33_fd = create_nano33_ble()
    if parrot_sock is None:
        parrot_sock = create_parrot_sock()

    received = False
    dwm_loc     = None
    parrot_data = None
    nano_data   = None

    # Suck everything from the socket, we need really up-to-date data
    while True:
        # Wait infinitely if we don't have any data
        timeout = 0 if received else None

        rd, wr, ex = select.select([dwm_fd, nano33_fd, parrot_sock, stop_efd],
                                   [], [], timeout)
        if 0 == len(rd):
            break
        if stop_efd in rd:
            break;

        if dwm_fd in rd:
            loc = receive_dwm_location(dwm_fd)
            #print_location(loc)

            if is_dwm_location_reliable(loc):
                received = True
                dwm_loc = loc
        if parrot_sock in rd:
            parrot_data = receive_parrot_data_from_sock(parrot_sock)
            if parrot_data is not None:
                received = True
        if nano33_fd in rd:
            acc, attitude = receive_nano33_data()
            nano_data = {}
            nano_data["acc"] = acc # ax, ay, az, ts
            nano_data["attitude"] = attitude
            nano_data["ts"] = time.time()

    return dwm_loc, parrot_data, nano_data

def find_anchor_by_addr(location, addr):
    for anchor in location['anchors']:
        if anchor['addr'] == addr:
            return anchor

    return None

def sigint_handler(sig, frame):
    print(' You pressed Ctrl+C! Disconnecting all devices, please wait ...')
    global should_stop, stop_efd
    should_stop = True
    stop_efd.set()


if __name__ == '__main__':
    args = docopt(__doc__)
    if args['--data-source'] == 'sock':
        DWM_DATA_SOURCE = dwm_source.SOCK
    elif args['--data-source'] == 'ble':
        DWM_DATA_SOURCE = dwm_source.BLE
    else:
        print("Error: incorrect --data-source param, should be 'ble' or 'sock'")
        sys.exit(-1)

    navigator = drone_navigator(cfg.LANDING_X, cfg.LANDING_Y, cfg.LANDING_ANGLE)
    plot_sock = create_plot_sock()

    navigator.start()
    droneloc = drone_localization(post_smoother=post_smoother)

    signal.signal(signal.SIGINT, sigint_handler)

    # If DWM modules stop returning distances don't fuse altitude or velocity,
    # otherwise innovation (residual) starts growing
    without_dwm_data = 1<<32

    while True:
        loc, parrot_data, nano_data = get_dwm_location_or_parrot_data()
        if should_stop:
            break

        if parrot_data is not None and without_dwm_data < UNRELIABLE_UPDATES:
            without_dwm_data += 1

            if 'alt' in parrot_data:
                # Fuse altitude
                droneloc.kf_process_alt(parrot_data)
            elif 'vel' in parrot_data:
                # Fuse velocity
                droneloc.kf_process_vel(parrot_data)
            elif 'att' in parrot_data:
                # PID control orientation
                navigator.orient_drone(parrot_data['att'][2])

        if loc is None:
            continue

        without_dwm_data = 0

        # TODO: not used anymore
        parrot_alt = 0
        rate = 0

        # Invoke distance localization
        x, y, z = droneloc.kf_process_dist(loc)

        # PID control
        navigator.navigate_drone(x, y, z)

        # Send all math output to the plot
        ts = time.time()

        send_plot_data(plot_sock, x, y, z, parrot_alt, ts, rate,
                       len(loc['anchors']), navigator, loc)

    destroy_dwm_fd()
    destroy_nano33_ble()
