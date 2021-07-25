import time
import os
import re
import random
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.artist import Artist
from simple_pid import PID
import numpy

class dynamic_plot():
    # Time range in seconds
    min_x = 0
    max_x = 10

    # Distange range in meters
    max_y = 5
    min_y = -5

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

        # Set other members
        self.lines2_y = lines2_y
        self.xdata  = []
        self.ydata  = []
        self.tsdata = []

    def _remove_outdated_data(self):
        now = time.time()

        diff = self.max_x - self.min_x
        ts = self.tsdata[0]

        while ts < now - diff:
            self.xdata.pop(0)
            self.ydata.pop(0)
            self.tsdata.pop(0)
            ts = self.tsdata[0]

    def update(self, xdata, ydata, text):
        self.xdata.append(xdata)
        self.ydata.append(ydata)
        self.tsdata.append(time.time())

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

        # Set text
        if self.previous_text:
            Artist.remove(self.previous_text)
        self.previous_text = self.ax.text(0.0, 1.025, text, transform=self.ax.transAxes, \
                                          bbox=dict(facecolor='green', alpha=0.3))

        # Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.legend()

        # We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()

#
# Welford's online algorithm
# https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
#
class welford_state:
    count = 0
    mean = 0
    m2 = 0

    def avg_welford(self, new_value):
        self.count += 1
        delta = new_value - self.mean
        self.mean += delta / self.count
        delta2 = new_value - self.mean
        self.m2 += delta * delta2

        return self.mean;

class avg_rate:
    welford = welford_state()
    ts = 0.0

    def __call__(self):
        rate = 0
        now = time.time()
        if self.ts:
            rate = 1.0 / (now - self.ts)
            rate = self.welford.avg_welford(rate)
        self.ts = now

        return rate

class drone:
    def __init__(self):
        self.x = 4

    def update(self, control):
        self.x += control * 0.1

        # Simulate random displacement
        self.x += random.uniform(-0.1, +0.1)

        return self.x

LANDING_X = 0.54 / 2

if __name__ == '__main__':
    # Remove toolback with buttons from plots. This is needed because
    # when drone is being controlled with the keyboard plots react
    # on button press.
    mpl.rcParams['toolbar'] = 'None'

    pid_x_plot = dynamic_plot('PID X', 'Time (s)', 'Drone X distance (m)',
                              'PID', 'target', LANDING_X)

    # Write the created model into the main function
    drone = drone()
    drone_x = drone.x

    default_pid_components = (10, 30, 0.1)
    default_pid_limits = (-1, 1)

    # Set three parameters of PID and limit output
    pid = PID(Kp=default_pid_components[0],
              Ki=default_pid_components[1],
              Kd=default_pid_components[2],
              setpoint=LANDING_X,
              proportional_on_measurement=False)

    pid.output_limits = default_pid_limits

    # Used to set time parameters
    start_time = time.time()
    last_time = start_time

    # Visualize Output Results
    setpoint, y, x = [], [], []

    # Average update rate
    avg_rate = avg_rate()

    # Set System Runtime
    while time.time() - start_time < 100:
        # Setting the time variable dt
        current_time = time.time()
        dt = (current_time - last_time)

        tuning_file = "./pid.tuning"

        tunings = default_pid_components
        limits = default_pid_limits
        if os.path.exists(tuning_file):
            with open(tuning_file, "r") as file:
                line = file.readline()
                components = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", line)
                if len(components) >= 3:
                    # To floats
                    components = [float(f) for f in components]
                    tunings = components[0:3]
                    if len(components) >= 5:
                        limits = components[3:5]
        pid.tunings = tunings
        pid.output_limits = limits

        # PID update
        control = pid(drone_x)
        drone_x = drone.update(control)

        # Calculate update rate
        rate = avg_rate()

        # Visualize Output Results
        x += [current_time - start_time]
        y += [drone_x]
        setpoint += [pid.setpoint]

        last_time = current_time

        # Visualization of Output Results
        components = pid.components
        text = "Kp=%.2f Ki=%.2f Kd=%.2f    Update %.1fHz\n" \
               "   %.2f    %.2f    %.2f\n" \
               "x %5.2f    control %d" % \
            (pid.Kp, pid.Ki, pid.Kd, rate, \
             components[0], components[1], components[2], \
             drone_x, int(control))
        pid_x_plot.update(current_time - start_time, drone_x, text)
