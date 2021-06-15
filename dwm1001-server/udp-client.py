import socket
import struct

MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5555

# Create sock and bind
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((MCAST_GRP, MCAST_PORT))

# Join group
mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

while True:
    buf = sock.recv(512)

    # Location header
    fmt = "iiihhiii"
    (x, y, z, pos_qf, pos_valid, ts_sec, ts_usec, nr_anchors) = struct.unpack_from(fmt, buf, 0)

    # Skip location header
    off = struct.calcsize(fmt)

    print("ts:%ld.%06ld [%d,%d,%d,%u] " % (ts_sec, ts_usec, x, y, z, pos_qf), end='')

    # For each anchor
    fmt = "iiihhihh"
    for i in range(0, nr_anchors):
        (x, y, z, pos_qf, pos_valid, dist, addr, dist_qf) = struct.unpack_from(fmt, buf, off)
        off += struct.calcsize(fmt)

        print("#%u) a:0x%08x [%d,%d,%d,%u] d=%u,qf=%u " % (i, addr, x, y, z, pos_qf, dist, dist_qf), \
              end='')

    print()
