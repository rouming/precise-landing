import struct
import time
import socket
import ctypes

file1 = open('data/field-session-2/log.1', 'r')

MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5555
MULTICAST_TTL = 2


PARROT_IP = "127.0.0.1"
PARROT_PORT = 5556

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)

parot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

last_ts = 0
last_ts_s = 0
last_ts_us = 0

speed = 2

def send_parrot_data(line):
    fields = line[line.find("alt") : -1].split(" ")
    fmt = "iiffff"

    alt = float(fields[1])
    roll = float(fields[3])
    pitch = float(fields[5])
    yaw = float(fields[7])

    buff = ctypes.create_string_buffer(512)
    struct.pack_into(fmt, buff, 0, last_ts_s, last_ts_us, alt, roll, pitch, yaw)
    parot_sock.sendto(buff, (PARROT_IP, PARROT_PORT))


while True:
    line = file1.readline()
    if not line:
        break

    if line.startswith("## get parrot data:"):
        send_parrot_data(line)
        continue

    if not line.startswith("ts:"):
        continue
    line = line.replace(" ", "")

    ts = line[line.find(":")+1 : line.find("[")]
    ts_s , ts_us = ts.split(".")
    ts_s = int(ts_s)
    ts_us = int(ts_us)

    vals = line[line.find("[")+1 : line.find("]")]
    vals = vals.split(",")
    pos_x = int(vals[0])
    pos_y = int(vals[1])
    pos_z = int(vals[2])
    pos_qf = int(vals[3])
    pos_valid = 1 # is it?

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
        anchor = {
            'pos': {
                'x':  int(x),
                'y':  int(y),
                'z':  int(z),
                'qf': pos_qf,
                'valid': pos_valid,
            },
            'dist': {
                'dist': int(dist),
                'addr': addr,
                'qf': dist_qf
            },
        }

        anchors.append(anchor)
        start += 1
        anchor_cnt += 1

    if (len(anchors) == 0):
        continue

    nr_anchors = len(anchors)  # is is not used

    fmt = "iiihhiii"
    buff = ctypes.create_string_buffer(512)

    # (x, y, z, pos_qf, pos_valid, ts_sec, ts_usec, nr_anchors)
    struct.pack_into(fmt, buff, 0, pos_x, pos_y, pos_z, pos_qf, pos_valid, ts_s, ts_us, nr_anchors)
    off = struct.calcsize(fmt)

    for anchor in anchors:
        fmt = "iiihhihh"

        # (x, y, z, pos_qf, pos_valid, dist, addr, dist_qf)
        struct.pack_into(fmt, buff, off,
                         anchor["pos"]["x"], anchor["pos"]["y"], anchor["pos"]["z"],
                         anchor["pos"]["qf"], anchor["pos"]["valid"],
                         anchor["dist"]["dist"], anchor["dist"]["addr"],
                         anchor["dist"]["qf"])
        off += struct.calcsize(fmt)

    if last_ts != 0:
        delay = (float(ts) - last_ts) / speed
        print("cur time: {} delay: {}".format(float(ts), delay))
        time.sleep(delay)
        last_ts = float(ts)
        last_ts_s = ts_s
        last_ts_us = ts_us

    else:
        last_ts = float(ts)

    sock.sendto(buff, (MCAST_GRP, MCAST_PORT))

