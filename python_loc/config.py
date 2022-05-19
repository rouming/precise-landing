#!/usr/bin/env python3

import numpy as np
import enum

# Distances from DWM1001-server
MCAST_GRP = '224.1.1.1'
MCAST_PORT = 5555

# Telemetry from drone
UDP_TELEMETRY_IP = '127.0.0.1'
UDP_TELEMETRY_PORT = 5556

# Commands to drone
UDP_COMMANDS_IP = '127.0.0.1'
UDP_COMMANDS_PORT = 5557

# Calculated math output to plot
UDP_PLOT_IP = '127.0.0.1'
UDP_PLOT_PORT = 5558

# Landing point in meters, middle of the landing platform
LANDING_X = 0.60 / 2
LANDING_Y = 0.60 / 2

TESTING_STAND = 2

if TESTING_STAND == 1:
    # Anchors coords (should create a rectangle with clockwise coords)
    ANCHORS = {
        0x2585: [0.60, 0.60, 0.00],
        0x262d: [0.60, 0.00, 0.00],
        0x28b9: [0.00, 0.00, 0.00],
        0x260f: [0.00, 0.60, 0.00],
    }

    # Tag
    TAG_ADDR = None # XXX ???????

    # Nano33Ble
    NANO33_MAC = "ed:26:b0:24:73:0c"

elif TESTING_STAND == 2:
    # Anchors coords (should create a rectangle with clockwise coords)
    ANCHORS = {
#        0x14d5: [0.000, 0.040, 0.000],
#        0x14d9: [0.000, 0.300, 0.000],
#        0x14ca: [0.040, 0.600, 0.005],
#        0x1337: [0.295, 0.600, 0.005],
#        0x0465: [0.605, 0.560, 0.000],
#        0x14c6: [0.605, 0.300, 0.000],
#        0x14c8: [0.570, 0.000, 0.005],
#        0x11cf: [0.300, 0.000, 0.005],

        0x14d5: [0.000, 0.000, 0.000],
        0x0465: [1.300, 0.000, 0.000],
        0x14c8: [1.300, 1.300, 0.000],
        0x14ca: [0.000, 1.300, 0.000],

    }

    #
    # Tags
    #

    # First
    TAG_ADDR = 0x16e9

    # Second
    #TAG_ADDR = 0x16fc

    #
    # Nano33BLE
    #
    NANO33_MAC = "e6:13:3e:cf:ea:78"

else:
    assert(0)
