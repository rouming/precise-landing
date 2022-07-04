#!/usr/bin/env python3

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from math import exp,sin,cos
from pylab import *

NUM_POINTS = 10000

mission_points = np.array(
    [[ 1,          1,          0.5       ],
     [-6,          6,          1,        ],
     [ 6,          6,          2,        ],
     [ 6,         -6,          3,        ],
     [-6,         -6,          4,        ],
     [-6,          6,          5,        ],
     [ 6,          6,          6,        ]]
)


mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = plt.axes(projection='3d')
a=0.05
b=0.10
# took the liberty of reducing the max value for th
# as it was giving you values of the order of e42
th=np.linspace(0, 50, NUM_POINTS)
#th=np.linspace(475, 500, NUM_POINTS)
x=a*exp(b*th)*cos(th)
y=a*exp(b*th)*sin(th)

NUM_POINTS = 4000

x = x[-NUM_POINTS:]
y = y[-NUM_POINTS:]

z=np.linspace(0, 8, NUM_POINTS)  # creating the z array with the same length as th
ax.plot3D(x, y, z, 'blue')  # adding z as an argument for the plot

# Sew two plots together
p = np.append(mission_points, [[x[-1], y[-1], z[-1]]], axis=0)
p = p.T
ax.plot3D(p[0:1][0], p[1:2][0], p[2:3][0], 'red')

# Extend mission points with helix points
x = list(reversed(x))
y = list(reversed(y))
z = list(reversed(z))

helix = []

i = 0
while i < NUM_POINTS:
    if z[i] < 2:
        break

    helix.append([x[i], y[i], z[i]])
    i += 40
    #i *= 1.05
    i = int(i)

p = np.append(mission_points, helix, axis=0)
p = p.T
ax.scatter(p[0:1][0], p[1:2][0], p[2:3][0], marker='o', color='black')

mission = ['takeoff']
for p in mission_points:
    mission.append('heading=%d,%d'  % (p[0]*1000, p[1]*1000))
    mission.append('pos=%d,%d,%d'  % (p[0]*1000, p[1]*1000, p[2]*1000))
for p in helix:
    mission.append('pos=%d,%d,%d'  % (p[0]*1000, p[1]*1000, p[2]*1000))
mission.append('land')

print("Number of mission actions: %d" % (len(mission)))

print("--mission %s" % (",".join(mission)))



plt.show()
