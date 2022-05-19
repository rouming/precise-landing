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

args        = None
dwm_fd      = None
nano33_fd   = None
parrot_sock = None
plot_sock   = None

dwm_manager = None

nano33_manager = None

should_stop = False
stop_efd    = eventfd.EventFD()

total_pos = 0
total_calc = 0

PID_CONTROL_RATE_HZ = 2

class dwm_source(enum.Enum):
    BLE = 0
    SOCK = 1

class parrot_event_type(enum.Enum):
    ALTITUDE = 0
    ATTITUDE = 1
    VELOCITY = 2
    POSITION = 3

post_smoother = None

DWM_DATA_SOURCE = dwm_source.BLE

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
            # Disable average, sometimes we have very high @rate
            # so the whole average goes mad, better to see rate in
            # momentum
            #rate = self.welford.avg_welford(rate)
        self.ts = now

        return rate

class drone_navigator(threading.Thread):
    # PID tuning files, format is: float float float [float, float]
    pid_tuning_file = "./pid.tuning"

    # Default PID config
    default_pid_components = (10, 30, 0.1)
    default_pid_limits = (-100, 100)

    # Thread data
    data = None
    lock =  threading.Lock()

    # XXX
    pitch = 0
    roll = 0
    rate = 0.0

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

        super().__init__()

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
                    if pid.Kp != tunings[0] or pid.Ki != tunings[1] or \
                       pid.Kd != tunings[2]:
                        pid.reset()
                    if len(components) >= 5:
                        limits = components[3:5]

        pid.tunings = tunings
        pid.output_limits = limits

    def run(self):
        while not should_stop:
            time.sleep(1/PID_CONTROL_RATE_HZ)

            self.lock.acquire()
            data = self.data
            self.lock.release()

            if data is None:
                continue

            (x, y) = data

            self._pid_tuning(self.x_pid, self.pid_tuning_file)
            self._pid_tuning(self.y_pid, self.pid_tuning_file)

            control_x = self.x_pid(x)
            control_y = self.y_pid(y)

            # Parrot accepts in signed percentage, i.e. [-100, 100] range
            roll = int(control_x)
            pitch = int(control_y)
            self._send_command(roll, pitch, 0, 0)

            self.roll = roll
            self.pitch = pitch
            self.rate = rate

    def navigate_drone(self, x, y):
        self.lock.acquire()
        self.data = (x, y)
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
        _, _, _, x, y, z  = struct.unpack(fmt, buf)
        if x == 0.0 and y == 0.0 and z == 0.0:
            return None
        parrot_data = {'vel': [x, y, z]}
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
        # TODO
        return None
    else:
        assert(0)
        return None

    parrot_data['ts'] = float("%ld.%06ld" % (sec, usec))

    return parrot_data

def is_dwm_location_reliable(loc):
    return len(loc['anchors']) >= 3

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
            print_location(loc)

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

    avg_rate = avg_rate()
    navigator = drone_navigator(cfg.LANDING_X, cfg.LANDING_Y)
    plot_sock = create_plot_sock()

    navigator.start()
    droneloc = drone_localization(post_smoother=post_smoother)

    signal.signal(signal.SIGINT, sigint_handler)

    while True:
        loc, parrot_data, nano_data = get_dwm_location_or_parrot_data()
        if should_stop:
            break
        if loc is None:
            continue

        parrot_alt = 0

        start = time.time()

        # Invoke localization
        x, y, z = droneloc.kf_process(loc)

        # Calculate update rate
        rate = avg_rate()

        # PID control
        navigator.navigate_drone(x, y)

        # Send all math output to the plot
        ts = time.time()

        send_plot_data(plot_sock, x, y, z, parrot_alt, ts, rate,
                       len(loc['anchors']), navigator, loc)

    destroy_dwm_fd()
    destroy_nano33_ble()
