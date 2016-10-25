#!/usr/bin/env python2
# Author: Ellot Sayes
# Outputs a simulation of accumulative guassian localisation method

import math
from math import exp
from numpy import sqrt
from math import pi
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import matplotlib.pyplot as plt
import numpy as np

import operator

fig = plt.figure()
ax = fig.gca(projection='3d')
X = np.arange(-5, 15, 0.5)
Y = np.arange(-5, 15, 0.5)
X, Y = np.meshgrid(X, Y)

beacon_locations = [[10,0],[0,0],[0,10],[10,10]]
beacon_distances = [6, 7.5, 5, 9.5]

def guassian(origin, mean, std=0.5):
    R = np.sqrt((X-origin[0])**2 + (Y-origin[1])**2)
    return np.exp(-(R-mean)**2/(2*std**2))/(sqrt(2*pi)*std)

def accumulate():
    z = 0
    for i in range(len(beacon_locations)):
        z = z + guassian(beacon_locations[i], beacon_distances[i])
    return z

Z = accumulate()

a=np.argmax(Z)

print("X=" + str(X[0][a%40]))
print("Y=" + str(Y[a/40][0]))



surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
ax.set_zlim(-1.01, 1.01)

ax.zaxis.set_major_locator(LinearLocator(10))
ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

fig.colorbar(surf, shrink=0.5, aspect=5)

plt.show()
