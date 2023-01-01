#!/usr/bin/env python3

"""Plot data from location.py script

Usage:
  plot.py [(--plot3d [--plot-true-pos] [--only])] [--filter]

Options:
  -h --help              Show this screen
  --plot3d               Plot 3D scene
  --only                 Plot 3D scene only
  --plot-true-pos        Plot true poistion with green dashed line on a 3D scene
  --filter               Filter data
"""

from docopt import docopt
import socket
import struct
import select
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.artist import Artist
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from scipy.ndimage import uniform_filter1d
from scipy.signal import savgol_filter

import config as cfg

X_LIM = 2
Y_LIM = 2
Z_LIM = 2

parrot_data = None
args = None

class dynamic_dist_plot():
    # Time range in seconds
    min_x = 0
    max_x = 10

    # Distange range in meters
    min_y = -1
    max_y = 5

    # For cleaning
    previous_text = None

    def __init__(self, plot_title, x_label, y_label):
        # Turn on plot interactive mode
        plt.ion()

        # Set up plot
        self.figure, self.ax = plt.subplots()

        # Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        self.ax.set(xlim=(self.min_x, self.max_x),
                    ylim=(self.min_y, self.max_y),
                    xlabel=x_label,
                    ylabel=y_label,
                    title=plot_title)

        # Enable grid
        self.ax.grid()

        # Create curves on the plot
        self.lines = {}
        self.dists = {}

        # Set other members
        self.xdata  = []

    def _remove_outdated_data(self):
        width = (self.max_x - self.min_x) * 2
        first = self.xdata[0]
        last = self.xdata[-1]

        while first < last - width:
            self.xdata.pop(0)
            for addr in self.dists.keys():
                self.dists[addr].pop(0)

            first = self.xdata[0]

    def update(self, xdata, addrs, dists):
        self.xdata.append(xdata)

        for addr in addrs:
            # Add not yet existent addresses
            if addr != 0x0000 and addr not in self.lines:
                lines, = self.ax.plot([],[], '-', label="%04x" % addr)
                self.lines[addr] = lines
                self.dists[addr] = []

        for addr in self.dists.keys():
            dist = -1
            if addr in addrs:
                i = addrs.index(addr)

                dist = dists[i];

            # when we do not receive the len from some anchor
            # we just use the last value to plot
            if dist < 0 and len(self.dists[addr]):
                dist = self.dists[addr][-1]

            self.dists[addr].append(dist)

        # Clean points which are not visible on the plot
        self._remove_outdated_data()

        # Following window
        if xdata >= self.max_x:
            diff = self.max_x - self.min_x
            self.max_x = xdata
            self.min_x = xdata - diff
            self.ax.set_xlim(self.min_x, self.max_x)

        # Update data (with the new _and_ the old points)
        for addr in self.dists.keys():
            line = self.lines[addr]
            line.set_xdata(self.xdata)
            line.set_ydata(self.dists[addr])

        # Set text
        if self.previous_text:
            Artist.remove(self.previous_text)
        pairs = []
        for i in range(len(dists)):
            pairs.append("%x=%2.2f" % (addrs[i], dists[i]))
        text = ', '.join(pairs)
        self.previous_text = self.ax.text(0.015, 1.095, text, transform=self.ax.transAxes, \
                                          bbox=dict(facecolor='green', alpha=0.3))

        # Need both of these in order to rescale
        self.ax.autoscale_view()
        self.ax.legend()

        # We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

class dynamic_plot():
    # Time range in seconds
    min_x = 0
    max_x = 10

    # Distange range in meters
    min_y = -2
    max_y = 2

    # Static line
    lines2_y = 0.0

    # For cleaning
    previous_text = None

    def __init__(self, plot_title, x_label, y_label,
                 lines1_label, lines2_label, lines2_y):
        # Turn on plot interactive mode
        plt.ion()

        # Set up plot
        self.figure, self.ax = plt.subplots()

        # Autoscale on unknown axis and known lims on the other
        self.ax.set_autoscaley_on(True)
        self.ax.set(xlim=(self.min_x, self.max_x),
                    ylim=(self.min_y, self.max_y),
                    xlabel=x_label,
                    ylabel=y_label,
                    title=plot_title)

        # Enable grid
        self.ax.grid()

        # Create curves on the plot
        self.lines1, = self.ax.plot([],[], '-', label=lines1_label)
        self.lines2, = self.ax.plot([],[], '-', label=lines2_label)

        if args['--filter']:
            self.filtered1, = self.ax.plot([],[], '-', label="Savgol")
            self.filtered2, = self.ax.plot([],[], '-', label="Uniform")

        # Set other members
        self.lines2_y = lines2_y
        self.xdata  = []
        self.ydata  = []
        self.ydata_filtered1 = []
        self.ydata_filtered2 = []

    def _remove_outdated_data(self):
        width = (self.max_x - self.min_x) * 2
        first = self.xdata[0]
        last = self.xdata[-1]

        while first < last - width:
            self.xdata.pop(0)
            self.ydata.pop(0)
            if args['--filter']:
                self.ydata_filtered1.pop(0)
                self.ydata_filtered2.pop(0)
            first = self.xdata[0]

    def update(self, xdata, ydata, ydata_filtered1, ydata_filtered2, text):
        self.xdata.append(xdata)
        self.ydata.append(ydata)
        if args['--filter']:
            self.ydata_filtered1.append(ydata_filtered1)
            self.ydata_filtered2.append(ydata_filtered2)

        # Clean points which are not visible on the plot
        self._remove_outdated_data()

        # Following window
        if xdata >= self.max_x:
            diff = self.max_x - self.min_x
            self.max_x = xdata
            self.min_x = xdata - diff
            self.ax.set_xlim(self.min_x, self.max_x)

        # Update data (with the new _and_ the old points)
        self.lines1.set_xdata(self.xdata)
        self.lines1.set_ydata(self.ydata)

        self.lines2.set_xdata([self.min_x, self.max_x])
        self.lines2.set_ydata([self.lines2_y, self.lines2_y])

        if args['--filter']:
            self.filtered1.set_xdata(self.xdata)
            self.filtered1.set_ydata(self.ydata_filtered1)

            self.filtered2.set_xdata(self.xdata)
            self.filtered2.set_ydata(self.ydata_filtered2)

        # Set text
        if self.previous_text:
            Artist.remove(self.previous_text)
        self.previous_text = self.ax.text(0.0, 1.025, text, transform=self.ax.transAxes, \
                                          bbox=dict(facecolor='green', alpha=0.3))

        # Need both of these in order to rescale
        self.ax.autoscale_view()
        self.ax.legend()

        # We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()


first_ts = 0
def draw_3dscene(ax, X, Y, Z, true_X, true_Y, true_Z, parrot_alt, ts, anch_cnt):
    global first_ts
    global parrot_data

    if first_ts == 0 and ts != first_ts:
        first_ts = ts

    # Clear previous data on the 3d plot
    ax.cla()
    ax.set(xlabel='x', ylabel='y', zlabel='z', title='Localization')

    if len(X) > 0:
        # FIXME: This needs to be not global 'plt' dependendent,
        # FIXME: otherwise it affects all other dynamic plot
        # FIXME: windows (e.g. PID). rpen
        # plt.plot(X_filtered, Y_filtered, Z_filtered, color='g')

        ax.scatter(X, Y, Z, color='r', s=0.8)
        if args['--plot-true-pos']:
            ax.scatter(true_X, true_Y, true_Z, color='g', s=0.8)
        ax.scatter(X[-1], Y[-1], Z[-1], color='b', s=12)

        ax.text2D(0.0, 1, "     x         y          z", transform=ax.transAxes)
        ax.text2D(0.0, 0.96, "{:7.2f} {:7.2f} {:7.2f}     {:7.3f}s    #{} anch".format(
                  X[-1], Y[-1], Z[-1], ts - first_ts, anch_cnt),
                  transform=ax.transAxes)

    if parrot_alt:
        ax.text2D(0.0, 0.86, " alt {:6.2f}m".format(parrot_alt), transform=ax.transAxes)
        ax.text2D(0.0, 0.82, "diff {:6.2f}m".format(Z[-1] - parrot_alt),
                  transform=ax.transAxes)

    # TODO: draw anchors
    # ax.add_collection3d(Poly3DCollection(rects, color='g', alpha=0.5))
    ax.set_xlim3d(0, X_LIM)
    ax.set_ylim3d(0, Y_LIM)
    ax.set_zlim3d(0, Z_LIM)

    # WTF?
    plt.pause(0.000001)

def recv_location(sock):
    # Wait for data
    select.select([sock], [], [], None)

    # Suck everything, drop all packets except the last one
    buf = None
    dropped = -1
    while True:
        try:
            fmt = "dffffffBffffffffffffffiii"
            sz = struct.calcsize(fmt)
            buf = sock.recv(sz, socket.MSG_PEEK)
            *data, nr_anchors = struct.unpack(fmt, buf)
            fmt += 'H' * nr_anchors + 'f' * nr_anchors
            sz = struct.calcsize(fmt)
            buf = sock.recv(sz)
            data = struct.unpack(fmt, buf)
            dropped += 1
            continue
        except:
            # EAGAIN
            break

    return *data, dropped

if __name__ == '__main__':
    args = docopt(__doc__)

    # Remove toolback with buttons from plots. This is needed because
    # when drone is being controlled with the keyboard plots react
    # on button press.
    mpl.rcParams['toolbar'] = 'None'

    # Plot interactive
    plt.ion()

    if not args['--only']:
        # Create PID plots
        pid_x_plot = dynamic_plot('PID X', 'Time (s)', 'Drone X distance (m)',
                                  'PID', 'target', cfg.LANDING_X)
        pid_y_plot = dynamic_plot('PID Y', 'Time (s)', 'Drone Y distance (m)',
                                  'PID', 'target', cfg.LANDING_Y)

        # Create len plots
        len_plot = dynamic_dist_plot("Anchors dist", "Time (s)", "Drone distance (m)")

    # Create 3D plot
    if args['--plot3d']:
        fig3d = plt.figure()
        ax3d = fig3d.add_subplot(111, projection='3d')

    # Create plot sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((cfg.UDP_PLOT_IP, cfg.UDP_PLOT_PORT))
    sock.setblocking(False)

    X = []
    Y = []
    Z = []

    true_X = []
    true_Y = []
    true_Z = []

    start_ts = 0.0

    while True:
        (ts, x, y, z,
         true_x, true_y, true_z, true_pos_valid,
         parrot_alt, rate, xKp, xKi, xKd, xp, xi, xd,
         yKp, yKi, yKd, yp, yi, yd, roll, pitch, nr_anchors,
         *addrs_and_dists, dropped) = recv_location(sock)

        addrs = addrs_and_dists[:nr_anchors]
        dists = addrs_and_dists[nr_anchors:]

        X.append(x)
        Y.append(y)
        Z.append(z)

        true_X.append(true_x)
        true_Y.append(true_y)
        true_Z.append(true_z)

        X_f1 = 0.0
        X_f2 = 0.0
        Y_f1 = 0.0
        Y_f2 = 0.0

        if args['--filter']:
            moving_window = 15

            if len(X) < moving_window:
                continue

            X_filtered1 = savgol_filter(X, moving_window, 5, mode="nearest")
            X_filtered2 = uniform_filter1d(X, size=moving_window, mode="reflect")

            Y_filtered1 = savgol_filter(Y, moving_window, 5, mode="nearest")
            Y_filtered2 = uniform_filter1d(Y, size=moving_window, mode="reflect")

            X_f1 = X_filtered1[-1]
            X_f2 = X_filtered2[-1]
            Y_f1 = Y_filtered1[-1]
            Y_f2 = Y_filtered2[-1]

        if not args['--only']:
            # PID texts
            pid_x_text = "Kp=%.2f Ki=%.2f Kd=%.2f   Update %.1fHz  dropped %d\n" \
                         "   %.2f    %.2f    %.2f\n" \
                         "x %5.2f  roll %d" % \
                         (xKp, xKi, xKd, rate, dropped, xp, xi, xd, x, roll)

            pid_y_text = "Kp=%.2f Ki=%.2f Kd=%.2f   Update %.1fHz  dropped %d\n" \
                         "   %.2f    %.2f    %.2f\n" \
                         "y %5.2f  pitch %d" % \
                         (yKp, yKi, yKd, rate, dropped, yp, yi, yd, y, pitch)

            # Timestamp from 0
            if start_ts == 0.0:
                start_ts = ts
                ts -= start_ts

            # Update PID plots
            pid_x_plot.update(ts, x, X_f1, X_f2, pid_x_text)
            pid_y_plot.update(ts, y, Y_f1, Y_f2, pid_y_text)

            len_plot.update(ts, addrs, dists)

        # Draw 3d scene
        if args['--plot3d']:
            draw_3dscene(ax3d, X, Y, Z, true_X, true_Y, true_Z,
                         parrot_alt, ts, nr_anchors)
