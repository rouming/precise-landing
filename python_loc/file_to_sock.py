#!/usr/bin/env python3

"""File to socket

Usage:
  file_to_sock.py parse-log        --file <file>
  file_to_sock.py parse-trajectory --file <file> --anchor <anchor>... [--noise-std <sigma>] [--seed <seed>]

Options:
  -h --help              Show this screen
  --file <file>          File with logged data, e.g. data/field-session-2/log.6.leastsq.manual
  --anchor <anchor>      Specify anchor in the format 'addr_in_hex,x_mm,y_mm,z_mm',
                         e.g. '0x16e9,0,100,0'. At least 3 anchors should be specified.
  --noise-std <sigma>    Add gaussian noise in mm to the anchor calculated distance,
                         e.g. standard deviation of real a DWM1001 device can vary
                         from 20mm to 200mm.
  --seed <seed>          Seed value for the random generator. 0 is the default value.
"""

from docopt import docopt
import struct
import time
import socket
import ctypes
import sys
import numpy as np
import math

MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5555
MULTICAST_TTL = 2

PARROT_IP = "127.0.0.1"
PARROT_PORT = 5556

last_ts = 0
speed = 2

def create_parrot_sock():
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_parrot_data(parrot_sock, line):
    fields = line[line.find("alt") : -1].split(" ")
    fmt = "fffff"

    alt = float(fields[1])
    roll = float(fields[3])
    pitch = float(fields[5])
    yaw = float(fields[7])

    buff = ctypes.create_string_buffer(512)
    struct.pack_into(fmt, buff, 0, last_ts, alt, roll, pitch, yaw)
    parrot_sock.sendto(buff, (PARROT_IP, PARROT_PORT))

def create_dwm_sock():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                    socket.inet_aton("127.0.0.1"))

    return sock

def send_dwm_data(dwm_sock, loc):
    nr_anchors = len(loc['anchors'])

    fmt = "iiihhfi"
    buff = ctypes.create_string_buffer(512)

    pos = loc['pos']

    # (x, y, z, pos_qf, pos_valid, ts, nr_anchors)
    struct.pack_into(fmt, buff, 0,
                     *pos['coords'], pos['qf'], pos['valid'],
                     loc['ts'], nr_anchors)
    off = struct.calcsize(fmt)

    for anchor in loc['anchors']:
        fmt = "iiihhihh"
        coords = anchor["pos"]["coords"]

        # (x, y, z, pos_qf, pos_valid, dist, addr, dist_qf)
        struct.pack_into(fmt, buff, off,
                         *coords,
                         anchor["pos"]["qf"], anchor["pos"]["valid"],
                         anchor["dist"]["dist"], anchor["addr"],
                         anchor["dist"]["qf"])
        off += struct.calcsize(fmt)

    dwm_sock.sendto(buff, (MCAST_GRP, MCAST_PORT))

def parse_log_file(dwm_sock, parrot_sock, data_file):
    global last_ts, last_ts_s, last_ts_us

    while True:
        line = data_file.readline()
        if not line:
            break

        if line.startswith("## get parrot data:"):
            send_parrot_data(parrot_sock, line)
            continue

        if not line.startswith("ts:"):
            continue
        line = line.replace(" ", "")

        ts = line[line.find(":")+1 : line.find("[")]
        vals = line[line.find("[")+1 : line.find("]")]
        vals = vals.split(",")

        # Be aware that we have all saved coords and distances in mm
        loc = {
            'pos': {
                'coords': [int(vals[0]), int(vals[1]), int(vals[2])],
                'qf': int(vals[3]),
                'valid': True
            },
            'ts': ts,
        }

        anchor_cnt = 0
        start = 0
        anchors = []

        while (anchor_cnt < 4):
            start = line.find("a:", start)
            if start == -1:
                break

            addr = line[start+2 : line.find("[", start)]
            addr = int(addr, base=16)

            anchor_pos = line[line.find("[", start)+1 : line.find("]", start)]
            x, y, z, pos_qf = anchor_pos.split(",")
            pos_qf = int(pos_qf)

            dist = line[line.find("d=", start)+2 : line.find(",qf=", start)]
            dist = int(dist)

            dist_qf = line[line.find("qf=", start)+3 : line.find("#", start)]
            dist_qf = int(dist_qf)

            # Be aware that we have all saved coords and distances in mm
            anchor = {
                'addr': addr,
                'pos': {
                    'coords':  [int(x), int(y), int(z)],
                    'qf': loc['pos']['qf'],
                    'valid': loc['pos']['valid'],
                },
                'dist': {
                    'dist': int(dist),
                    'qf': dist_qf
                },
            }

            anchors.append(anchor)
            start += 1
            anchor_cnt += 1

        loc['anchors'] = anchors
        send_dwm_data(dwm_sock, loc)

        if last_ts != 0:
            delay = (float(ts) - last_ts) / speed
            print("cur time: {} delay: {}".format(float(ts), delay))
            time.sleep(delay)
        last_ts = float(ts)

def parse_trajectory_file(dwm_sock, args, data_file):
    anchors_args = args['--anchor']
    if len(anchors_args) < 3:
        print("Error: at least 3 anchors should be specified")
        sys.exit(-1)

    seed = 0
    if args['--seed']:
        seed = int(args['--seed'])

    np.random.seed(seed)

    noise_std = 0.0
    if args['--noise-std']:
        noise_std = float(args['--noise-std'])

    anchors = []
    for anchor_arg in anchors_args:
        arr = anchor_arg.split(',')
        if len(arr) != 4:
            print("Error: anchor %s format is incorrect" % anchor_arg)
            sys.exit(-1)

        anchors.append({
            'addr': int(arr[0], base=16),
            'pos': {
                'coords': [int(arr[1]), int(arr[2]), int(arr[3])],
                'qf': 100,
                'valid': True,
            },
        })

    while True:
        line = data_file.readline()
        if not line:
            break

        ts = time.time()
        coords = line.split(',')
        if len(coords) != 3:
            print("Error: trajectory file is incorrect format")
            sys.exit(-1)

        coords = [int(v) for v in coords]

        loc = {
            'pos': {
                'coords':  coords,
                'qf': 100,
                'valid': True,
            },
            'ts': ts,
        }

        for anchor in anchors:
            acoords = np.array(anchor['pos']['coords'])

            # Find distance from an anchor to a trajectory position
            vec = coords - acoords
            dist = np.sqrt(vec.dot(vec))
            dist += np.random.normal(0, noise_std)

            anchor['dist'] = {
                'dist': int(dist),
                'qf': 100,
            }

        loc['anchors'] = anchors
        send_dwm_data(dwm_sock, loc)

        # Frequency to duration
        delay = 1 / 5
        print("cur time: {} delay: {}".format(float(ts), delay))
        time.sleep(delay)

if __name__ == '__main__':
    args = docopt(__doc__)

    sock = create_dwm_sock()
    parrot_sock = create_parrot_sock()
    data_file = open(args['--file'], 'r')

    if args['parse-log']:
        parse_log_file(dwm_sock, parrot_sock, data_file)
    elif args['parse-trajectory']:
        parse_trajectory_file(dwm_sock, args, data_file)
