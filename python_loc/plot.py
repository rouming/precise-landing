#!/usr/bin/env python

import socket
import struct
import select
import time
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.artist import Artist
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from scipy.ndimage.filters import uniform_filter1d
from scipy.signal import savgol_filter

import config as cfg

X_LIM = 3
Y_LIM = 3
Z_LIM = 3

DRAW_3D_SCENE = True
PLOT_FILTERED = False

parrot_data = None

class dynamic_len_plot():
    # Time range in seconds
    min_x = 0
    max_x = 10

    # Distange range in meters
    min_y = -1
    max_y = 5

    # For cleaning
    previous_text = None

    def __init__(self, plot_title, x_label, y_label,
                 lines1_label, lines2_label, lines3_label, lines4_label):
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
        self.lines3, = self.ax.plot([],[], '-', label=lines3_label)
        self.lines4, = self.ax.plot([],[], '-', label=lines4_label)


        # Set other members
        self.xdata  = []
        self.x2585_len = []
        self.x262d_len = []
        self.x28b9_len = []
        self.x260f_len = []

    def _remove_outdated_data(self):
        width = (self.max_x - self.min_x) * 2
        first = self.xdata[0]
        last = self.xdata[-1]

        while first < last - width:
            self.xdata.pop(0)
            self.x2585_len.pop(0)
            self.x262d_len.pop(0)
            self.x28b9_len.pop(0)
            self.x260f_len.pop(0)

            first = self.xdata[0]

    def update(self, xdata, x2585_new, x262d_new, x28b9_new, x260f_new):
        self.xdata.append(xdata)

        # when we do not receive the len from some anchor we just use the last value to plot
        if x2585_new < 0 and len(self.x2585_len) > 0:
            x2585_new = self.x2585_len[-1]
        if x262d_new < 0 and len(self.x262d_len) > 0:
            x262d_new = self.x262d_len[-1]
        if x28b9_new < 0 and len(self.x28b9_len) > 0:
            x28b9_new = self.x28b9_len[-1]
        if x260f_new < 0 and len(self.x260f_len) > 0:
            x260f_new = self.x260f_len[-1]

        self.x2585_len.append(x2585_new)
        self.x262d_len.append(x262d_new)
        self.x28b9_len.append(x28b9_new)
        self.x260f_len.append(x260f_new)

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
        self.lines1.set_ydata(self.x2585_len)

        self.lines2.set_xdata(self.xdata)
        self.lines2.set_ydata(self.x262d_len)

        self.lines3.set_xdata(self.xdata)
        self.lines3.set_ydata(self.x28b9_len)

        self.lines4.set_xdata(self.xdata)
        self.lines4.set_ydata(self.x260f_len)

        # Set text
        if self.previous_text:
            Artist.remove(self.previous_text)
        text = "0x2585=%2.2f, 0x262d=%2.2f\n 0x28b9=%2.2f, 0x260f=%2.2f," % \
               (x2585_new, x262d_new, x28b9_new, x260f_new)
        self.previous_text = self.ax.text(-0.025, 1.025, text, transform=self.ax.transAxes, \
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

        if PLOT_FILTERED:
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
            if PLOT_FILTERED:
                self.ydata_filtered1.pop(0)
                self.ydata_filtered2.pop(0)
            first = self.xdata[0]

    def update(self, xdata, ydata, ydata_filtered1, ydata_filtered2, text):
        self.xdata.append(xdata)
        self.ydata.append(ydata)
        if PLOT_FILTERED:
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

        if PLOT_FILTERED:
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
def draw_scene(ax, X, Y, Z, parrot_alt, ts, anch_cnt):
    global first_ts
    global parrot_data

    coords = list(cfg.ANCHORS.values())
    rects = [[coords[0], coords[1], coords[2], coords[3]]]

    if first_ts == 0 and ts != first_ts:
        first_ts = ts

    # Clear previous data on the 3d plot
    ax3d.cla()

    if len(X) > 0:
        # FIXME: This needs to be not global 'plt' dependendent,
        # FIXME: otherwise it affects all other dynamic plot
        # FIXME: windows (e.g. PID). rpen
        # plt.plot(X_filtered, Y_filtered, Z_filtered, color='g')

        ax.scatter(X, Y, Z, color='r', s=0.8)
        ax.scatter(X[-1], Y[-1], Z[-1], color='b', s=5)

        ax.text2D(0.0, 1, "     x         y          z", transform=ax.transAxes)
        ax.text2D(0.0, 0.96, "{:7.2f} {:7.2f} {:7.2f}     {:7.3f}s    #{} anch".format(
                  X[-1], Y[-1], Z[-1], ts - first_ts, anch_cnt),
                  transform=ax.transAxes)

    if parrot_alt:
        ax.text2D(0.0, 0.86, " alt {:6.2f}m".format(parrot_alt), transform=ax.transAxes)
        ax.text2D(0.0, 0.82, "diff {:6.2f}m".format(Z[-1] - parrot_alt),
                  transform=ax.transAxes)

    ax.add_collection3d(Poly3DCollection(rects, color='g', alpha=0.5))
    ax.set_xlim3d(-X_LIM, X_LIM)
    ax.set_ylim3d(-Y_LIM, Y_LIM)
    ax.set_zlim3d(0, Z_LIM)

    # WTF?
    plt.pause(0.000001)

def recv_math_output(sock):
    # Wait for data
    select.select([sock], [], [], None)

    # 1 double, 17 floats, 3 int32, 4 floats
    fmt = "dfffffffffffffffffiiiffff"
    sz = struct.calcsize(fmt)

    # Suck everything, drop all packets except the last one
    buf = None
    dropped = -1
    while True:
        try:
            buf = sock.recv(sz)
            dropped += 1
            continue
        except:
            # EAGAIN
            break

    data = struct.unpack(fmt, buf)
    return *data, dropped

if __name__ == '__main__':
    # Remove toolback with buttons from plots. This is needed because
    # when drone is being controlled with the keyboard plots react
    # on button press.
    mpl.rcParams['toolbar'] = 'None'

    # Plot interactive
    plt.ion()

    # Create PID plots
    pid_x_plot = dynamic_plot('PID X', 'Time (s)', 'Drone X distance (m)',
                              'PID', 'target', cfg.LANDING_X)
    pid_y_plot = dynamic_plot('PID Y', 'Time (s)', 'Drone Y distance (m)',
                              'PID', 'target', cfg.LANDING_Y)

    # create len plots
    len_plot = dynamic_len_plot("Anchors len", "Time (s)", "Drone distance (m)",
                                "0x2585", "0x262d", "0x28b9", "0x260f")

    # Create 3D plot
    if DRAW_3D_SCENE:
        fig3d = plt.figure()
        ax3d = fig3d.add_subplot(111, projection='3d')

    # Create plot sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((cfg.UDP_PLOT_IP, cfg.UDP_PLOT_PORT))
    sock.setblocking(False)

    X = []
    Y = []
    Z = []

    start_ts = 0.0

    while True:
        (ts, x, y, z, parrot_alt, rate, xKp, xKi, xKd, xp, xi, xd,
         yKp, yKi, yKd, yp, yi, yd, roll, pitch, nr_anchors,
         x2585_len, x262d_len, x28b9_len, x260f_len, dropped) = recv_math_output(sock)

        X.append(x)
        Y.append(y)
        Z.append(z)

        X_f1 = 0.0
        X_f2 = 0.0
        Y_f1 = 0.0
        Y_f2 = 0.0

        if PLOT_FILTERED:
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

        len_plot.update(ts, x2585_len, x262d_len, x28b9_len, x260f_len)

        # Draw 3d scene
        if DRAW_3D_SCENE:
            draw_scene(ax3d, X, Y, Z, parrot_alt, ts, nr_anchors)
