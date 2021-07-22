import time
import random
import matplotlib.pyplot as plt
from matplotlib.artist import Artist
from simple_pid import PID
import numpy

class dynamic_plot():
    # Time range in seconds
    min_x = 0
    max_x = 10

    # Distange range in meters
    min_y = -5
    max_y = 5

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
        self.previous_text = self.ax.text(0.0, 1.07, text, transform=self.ax.transAxes, \
                                          bbox=dict(facecolor='green', alpha=0.3, pad=5))

        # Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        self.ax.legend()

        # We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()


class drone:
    def __init__(self):
        self.x = 4

    def update(self, control):
        self.x += control * 0.1;

        # Simulate random displacement
        self.x += random.uniform(-0.1, +0.1)

        return self.x

LANDING_X = 0.54 / 2

if __name__ == '__main__':

    pid_x_plot = dynamic_plot('PID X', 'Time (s)', 'Drone X distance (m)',
                              'PID', 'target', LANDING_X)

    # Write the created model into the main function
    drone = drone()
    drone_x = drone.x

    # Set three parameters of PID and limit output
    pid = PID(2, 0.01, 0.1, setpoint=LANDING_X)
    pid.output_limits = (-1, 1)
    pid.sample_time  = 0.1

    # Used to set time parameters
    start_time = time.time()
    last_time = start_time

    # Visualize Output Results
    setpoint, y, x = [], [], []

    # Set System Runtime
    while time.time() - start_time < 20:
        # Setting the time variable dt
        current_time = time.time()
        dt = (current_time - last_time)

        control = pid(drone_x)
        drone_x = drone.update(control)

        print("x %f  control %f" % (drone_x, control))

        # Visualize Output Results
        x += [current_time - start_time]
        y += [drone_x]
        setpoint += [pid.setpoint]

        last_time = current_time

        time.sleep(pid.sample_time)

        # Visualization of Output Results
        text = "x %5.2f    ctrl %.2f" % (drone_x, control)
        pid_x_plot.update(current_time - start_time, drone_x, text)
