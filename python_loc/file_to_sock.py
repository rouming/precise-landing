import struct
import time
import socket
import ctypes
import numpy as np

anchors_id = ["a:0x00002585", "a:0x0000262d", "a:0x0000260f", "a:0x00002852"]
addr = [0x2585, 0x262d, 0x260f, 0x28b9]

A = [54, 54, 0] # 2585
B = [54,0,0] # 262d
C = [0,54,0] # 260f
D = [0,0,0] # 2852

def get_dist(line, id):
    l = line.split(id+"[")[1]
    l = l.split("d=")[1]
    val = l.split(",")[0]

    return int(val)

file1 = open('data/field-session-1/log.6', 'r')


MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5555

MULTICAST_TTL = 2

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)

last_ts = 0
speed = 2

while True:
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

    ts = line[line.find(":")+1 : line.find(" ")]
    ts_s , ts_us = ts.split(".")
    ts_s = int(ts_s)
    ts_us = int(ts_us)

    r = line.split(" [")[1]
    r = r.split("]#")[0]

    vals = r.split(',')
    x = int(vals[0])
    y = int(vals[1])
    z = int(vals[2])

    la = get_dist(line, 'a:0x00002585')
    lb = get_dist(line, 'a:0x0000262d')
    lc = get_dist(line, 'a:0x0000260f')
    ld = get_dist(line, 'a:0x00002852') # now it is 0x28b9

    nr_anchors = 4
    fmt = "iiihhiii"

    buff = ctypes.create_string_buffer(512)

    struct.pack_into(fmt, buff, 0, x, y, z, 0, 0, ts_s, ts_us, nr_anchors)
    off = struct.calcsize(fmt)

    fmt = "iiihhihh"
    struct.pack_into(fmt, buff, off, A[0], A[1], A[2], 100, 1, la, addr[0], 100)
    off += struct.calcsize(fmt)

    struct.pack_into(fmt, buff, off, B[0], B[1], B[2], 100, 1, lb, addr[1], 100)
    off += struct.calcsize(fmt)

    struct.pack_into(fmt, buff, off, C[0], C[1], C[2], 100, 1, lc, addr[2], 100)
    off += struct.calcsize(fmt)

    struct.pack_into(fmt, buff, off, D[0], D[1], D[2], 100, 1, ld, addr[3], 100)
    off += struct.calcsize(fmt)

    if last_ts != 0:
        delay = (float(ts) - last_ts) / speed
        print("cur time: {} delay: {}".format(float(ts), delay))
        time.sleep(delay)
        last_ts = float(ts)

    else:
        last_ts = float(ts)

    sock.sendto(buff, (MCAST_GRP, MCAST_PORT))

