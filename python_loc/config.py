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

# Anchors coords
A = np.array([0.60,0.60,0.00]) # 2585
B = np.array([0.60,0.00,0.00]) # 262d
C = np.array([0.00,0.60,0.00]) # 260f
D = np.array([0.00,0.00,0.00]) # 28b9

# Landing point in meters, middle of the landing platform
LANDING_X = 0.60 / 2
LANDING_Y = 0.60 / 2
