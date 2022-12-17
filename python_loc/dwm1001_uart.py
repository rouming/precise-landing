#!/usr/bin/env python3

"""DWM1001 UART

Receives range data from DWM1001 by UART and streams it over socket

Usage:
  dwm1001_uart.py --dev <dev> [--stream-to-sock]

Options:
  -h --help              Show this screen
  --dev <dev>            Serial device
  --stream-to-sock       Stream DWM location to sock
"""

from docopt import docopt
import eventfd
import signal
import serial
import json
import config as cfg

from file_to_sock import send_dwm_data, create_dwm_sock

should_stop = False
stop_efd = eventfd.EventFD()

def signal_handler(sig, frame):
    print(' You pressed Ctrl+C! Disconnecting all devices ...')
    global should_stop, stop_efd
    should_stop = True
    stop_efd.set()

def coords_to_mm(coords):
    return [int(v * 1000) for v in coords]

def to_loc(obj):
    loc = {
        'addr': obj['uid'],
        'pos': {
            'coords': [0, 0, 0],
            'qf': 0,
            'valid': 0,
        },
        'ts': obj['utime'] / 1e6,
        'anchors': [],
    }

    for rng in obj['rngs']:
        coords = [0, 0, 0]
        valid = 0

        if 'uid' not in rng or \
           'rng' not in rng:
            # Invalid range
            continue

        addr = rng['uid']
        rng = rng['rng']

        if rng == 65535:
            # Invalid range, tag is too close to the anchor?
            continue

        if addr in cfg.ANCHORS:
            coords = cfg.ANCHORS[addr]
            coords = coords_to_mm(coords)
            valid = 1

        anchor = {
            'addr': addr,
            'pos': {
                'coords': coords,
                'qf': 100,
                'valid': valid,
            },
            'dist': {
                'dist': rng,
                'qf': 100
            },
        }

        loc['anchors'].append(anchor)

    return loc

if __name__ == '__main__':
    args = docopt(__doc__)
    signal.signal(signal.SIGINT, signal_handler)

    if args['--stream-to-sock']:
        dwm_sock = create_dwm_sock()

    ser = serial.Serial(args['--dev'], 115200)

    while not should_stop:
        try:
            line = ser.readline()
            obj = json.loads(line)
            if 'uid' not in obj or \
               'rngs' not in obj:
                # Invalid json
                continue
        except:
            continue

        loc = to_loc(obj)
        if args['--stream-to-sock']:
            send_dwm_data(dwm_sock, loc)

        print("ts=%.06f tag=0x%x " % (loc['ts'], loc['addr']), end='')
        for a in loc['anchors']:
            print("[0x%x: %.3fm] " % (a['addr'], int(a['dist']['dist']) / 1000.0),
                  end='')
        print("")

    ser.close()
