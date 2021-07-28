#!/usr/bin/env python
import numpy as np

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

# Anchors coords (should create a rectangle with anti-clockwise coords)
ANCHORS = {
    0x2585: np.array([0.60,0.60,0.00]),
    0x262d: np.array([0.60,0.00,0.00]),
    0x28b9: np.array([0.00,0.00,0.00]),
    0x260f: np.array([0.00,0.60,0.00]),
}

# Tag
TAG_MAC = "c7:bc:67:9e:e0:98"

# Landing point in meters, middle of the landing platform
LANDING_X = 0.60 / 2
LANDING_Y = 0.60 / 2
